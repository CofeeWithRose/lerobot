"""
HTTP server for controlling the SO101 robot arm.
Only one client can control the arm at a time (session-based lock).

Server-side control loop architecture:
- The server maintains a normalized target position for each joint [0, 1].
- A background thread runs at ~50Hz, continuously driving the robot toward
  the target position using robot.send_action().
- The frontend sends user interaction state (joystick deflection as "velocity")
  only when it changes — NOT on every frame.
- The server applies the velocity to the target each control tick, producing
  smooth incremental movement.

Usage:
    cd demo/web
    pip install flask
    python server.py
"""

import logging
import time
import uuid
import threading
from flask import Flask, request, jsonify, render_template, session

from robot_bridge import (
    initRobot,
    getAllMotorMoveRange,
    getRobotState,
    setRobotState,
    stand_up,
    disconnectRobot,
)

# ── Flask app ────────────────────────────────────────────────────────
app = Flask(__name__)
app.secret_key = "robot-arm-secret-key-change-me"

# ── Global state ─────────────────────────────────────────────────────
robot = None
move_ranges = None
robot_init_state = None
saved_arm_state = None  # arm position recorded when control is acquired

# Lock mechanism: only one client can control at a time
control_lock = threading.Lock()        # protects robot hardware access
control_owner = None                   # session id of the current controller
control_owner_lock = threading.Lock()  # protects control_owner read/write
LOCK_TIMEOUT = 300                     # seconds before auto-release
lock_acquired_at = 0

# ── Server-side control loop state ───────────────────────────────────
CONTROL_HZ = 50                        # control loop frequency
CONTROL_INTERVAL = 1.0 / CONTROL_HZ

# Normalized target position for each joint [0.0, 1.0]
target_normalized = {}
# Velocity (from joystick deflection) for each joint, in normalized-units/sec
# e.g. velocity["shoulder_pan"] = 0.3 means target moves +0.3/sec
velocity = {}
# Lock protecting target_normalized and velocity
state_lock = threading.Lock()

# Background control thread
_control_thread = None
_control_thread_running = False

# Scale factor: how fast the target moves per unit of joystick deflection.
# Joystick range is [-0.5, +0.5]. With VELOCITY_SCALE=0.8, full deflection
# moves the target at 0.4 normalized-units/sec (i.e. 40% of range per second).
VELOCITY_SCALE = 0.8

# Direction inversion map: joints where increasing normalized value corresponds
# to the opposite physical direction from what the UI expects.
# For SO101: encoder value increasing means shoulder_pan turns left (UI expects right),
# shoulder_lift goes down (UI expects up), elbow_flex retracts (UI expects extend).
# Set to -1.0 to invert, 1.0 to keep as-is.
JOINT_DIRECTION = {
    "shoulder_pan":  1.0,
    "shoulder_lift": 1.0,
    "elbow_flex":    -1.0,
    "wrist_flex":    1.0,
    "wrist_roll":    -1.0,
    "gripper":        1.0,
}


def _init_robot():
    """Initialize the robot arm (called once at startup or on re-acquire)."""
    global robot, move_ranges, robot_init_state
    robot = initRobot()
    move_ranges = getAllMotorMoveRange(robot)
    robot_init_state = getRobotState(robot)
    print("Robot initialized. Move ranges:", move_ranges)


def _init_target_from_robot():
    """Read current robot position and initialize target_normalized to match."""
    global target_normalized, velocity
    if robot is None or move_ranges is None:
        return
    obs = robot.get_observation()
    with state_lock:
        target_normalized = {}
        velocity = {}
        for joint_name, rng in move_ranges.items():
            key = joint_name + ".pos"
            if rng and key in obs:
                lo, hi = rng
                if hi != lo:
                    target_normalized[joint_name] = max(0.0, min(1.0,
                        (obs[key] - lo) / (hi - lo)))
                else:
                    target_normalized[joint_name] = 0.5
            else:
                target_normalized[joint_name] = 0.5
            velocity[joint_name] = 0.0
    print(f"Target initialized from robot: {target_normalized}")


def _start_control_loop():
    """Start the background control thread."""
    global _control_thread, _control_thread_running
    if _control_thread is not None and _control_thread.is_alive():
        return
    _control_thread_running = True
    _control_thread = threading.Thread(target=_control_loop, daemon=True)
    _control_thread.start()
    print("Control loop started.")


def _stop_control_loop():
    """Stop the background control thread."""
    global _control_thread_running
    _control_thread_running = False
    if _control_thread is not None:
        _control_thread.join(timeout=2.0)
    print("Control loop stopped.")


def _control_loop():
    """
    Background thread: runs at CONTROL_HZ, applies velocity to target,
    converts target to real joint values, and sends a single action frame.

    Anti-jitter design:
    - Always sends the current target position every tick (even when velocity
      is zero) so the servos receive a consistent stream of goal positions.
      Skipping frames causes servos to briefly lose their goal and spring
      back, producing visible "twitch" or "shudder".
    - Uses a longer lock timeout to avoid dropping frames when the polling
      endpoint briefly holds the hardware lock.
    - Caches the last-sent action so the polling endpoint can read state
      without needing to hit the hardware bus.
    """
    global target_normalized, _last_action_cache
    while _control_thread_running:
        t_start = time.perf_counter()

        if robot is None or move_ranges is None:
            time.sleep(CONTROL_INTERVAL)
            continue

        with state_lock:
            # Apply velocity to target (integrate over one tick)
            for joint_name in list(target_normalized.keys()):
                v = velocity.get(joint_name, 0.0)
                if abs(v) > 1e-6:
                    new_val = target_normalized[joint_name] + v * CONTROL_INTERVAL
                    target_normalized[joint_name] = max(0.0, min(1.0, new_val))

            # Build action dict from current target
            action = {}
            for joint_name, norm_val in target_normalized.items():
                if joint_name in move_ranges and move_ranges[joint_name] is not None:
                    lo, hi = move_ranges[joint_name]
                    action[joint_name + ".pos"] = lo + norm_val * (hi - lo)

        # Always send the current target to keep servos locked on position.
        # Use a generous timeout so we rarely skip a frame.
        if action:
            acquired = control_lock.acquire(timeout=CONTROL_INTERVAL * 0.8)
            if acquired:
                try:
                    robot.send_action(action)
                except Exception as e:
                    logging.exception("Control loop send_action error")
                finally:
                    control_lock.release()

        # Sleep to maintain target frequency
        elapsed = time.perf_counter() - t_start
        sleep_time = CONTROL_INTERVAL - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)


def _shutdown_robot():
    """Stop control loop, restore arm to saved position, and disconnect."""
    global robot, move_ranges, robot_init_state, saved_arm_state
    _stop_control_loop()
    if robot is not None:
        try:
            if saved_arm_state:
                print(f"Restoring arm to saved position: {saved_arm_state}")
                setRobotState(robot, saved_arm_state, move_ranges, expect_duration_ms=2000)
                print("Arm restored to saved position.")
            else:
                print("No saved position, returning to stand-up position...")
                stand_up(robot, move_ranges, expect_duration_ms=2000)
                print("Arm returned to stand-up position.")
            print("Disconnecting robot...")
            disconnectRobot(robot)
            print("Robot disconnected.")
        except Exception as e:
            logging.exception(f"Error during robot shutdown: {e}")
        finally:
            robot = None
            move_ranges = None
            robot_init_state = None
            saved_arm_state = None


def _get_session_id():
    """Get or create a unique session id for the current client."""
    if "sid" not in session:
        session["sid"] = str(uuid.uuid4())
    return session["sid"]


def _try_acquire(sid):
    """Try to acquire control. Returns (success, message)."""
    global control_owner, lock_acquired_at
    with control_owner_lock:
        now = time.time()
        # Auto-release if timed out
        if control_owner is not None and (now - lock_acquired_at) > LOCK_TIMEOUT:
            print(f"Control auto-released from {control_owner} (timeout)")
            control_owner = None

        if control_owner is None:
            control_owner = sid
            lock_acquired_at = now
            return True, "Control acquired"
        elif control_owner == sid:
            lock_acquired_at = now  # refresh
            return True, "Already in control"
        else:
            return False, "Another client is currently controlling the arm"


def _release(sid):
    """Release control."""
    global control_owner
    with control_owner_lock:
        if control_owner == sid:
            control_owner = None
            return True, "Control released"
        return False, "You are not the current controller"


def _check_control(sid):
    """Check if this session currently holds control."""
    global control_owner
    with control_owner_lock:
        now = time.time()
        if control_owner is not None and (now - lock_acquired_at) > LOCK_TIMEOUT:
            print(f"Control auto-released from {control_owner} (timeout in _check_control)")
            control_owner = None
            return False
        return control_owner == sid


def _refresh_lock(sid):
    """Refresh the lock timeout for the current controller (heartbeat)."""
    global lock_acquired_at
    with control_owner_lock:
        if control_owner == sid:
            lock_acquired_at = time.time()


def _get_normalized_state():
    """Get current robot state as both raw and normalized values."""
    if robot is None:
        return {}, {}
    obs = robot.get_observation()
    state = {k: round(obs[k], 3) for k in obs if k.endswith(".pos")}
    normalized = {}
    if move_ranges:
        for joint_name, rng in move_ranges.items():
            key = joint_name + ".pos"
            if key in state and rng:
                lo, hi = rng
                if hi != lo:
                    normalized[joint_name] = round((state[key] - lo) / (hi - lo), 4)
                else:
                    normalized[joint_name] = 0.5
    return state, normalized


# ── API Routes ───────────────────────────────────────────────────────

@app.route("/api/acquire", methods=["POST"])
def api_acquire():
    """Acquire exclusive control of the robot arm. Re-initializes robot if needed."""
    sid = _get_session_id()
    ok, msg = _try_acquire(sid)
    if ok:
        # If robot is not connected, re-initialize
        if robot is None:
            with control_lock:
                _init_robot()
        # Record current arm position so we can restore it on release
        with control_lock:
            global saved_arm_state
            saved_arm_state = getRobotState(robot)
            print(f"Saved arm position on acquire: {saved_arm_state}")
        # Initialize target from current robot position and start control loop
        _init_target_from_robot()
        _start_control_loop()
        return jsonify({"success": True, "message": msg, "sid": sid,
                        "move_ranges": {k: list(v) if v else None for k, v in move_ranges.items()} if move_ranges else {}}), 200
    return jsonify({"success": False, "message": msg, "sid": sid}), 423


@app.route("/api/release", methods=["POST"])
def api_release():
    """Release control: stop control loop, return arm to home, disconnect robot."""
    sid = _get_session_id()
    if not _check_control(sid):
        return jsonify({"success": False, "message": "You are not the current controller"}), 403

    # First, stop control loop, home the arm and disconnect (with hardware lock)
    with control_lock:
        _shutdown_robot()

    # Then release the session lock
    ok, msg = _release(sid)
    return jsonify({"success": ok, "message": msg}), 200 if ok else 403


@app.route("/api/status", methods=["GET"])
def api_status():
    """Get current control status and robot state."""
    sid = _get_session_id()
    is_controller = _check_control(sid)
    with control_owner_lock:
        locked = control_owner is not None
    state = {}
    ranges = {}
    if robot is not None:
        try:
            obs = robot.get_observation()
            state = {k: round(obs[k], 3) for k in obs if k.endswith(".pos")}
        except Exception as e:
            state = {"error": str(e)}
        if move_ranges:
            ranges = {k: list(v) if v else None for k, v in move_ranges.items()}
    return jsonify({
        "is_controller": is_controller,
        "locked": locked,
        "state": state,
        "move_ranges": ranges,
        "sid": sid,
    })


@app.route("/api/set_velocity", methods=["POST"])
def api_set_velocity():
    """
    Set the joystick velocity for incremental movement.
    Called only when user interaction changes (joystick moved/released),
    NOT on every frame.

    Body JSON: { "velocity": {"shoulder_pan": 0.3, "elbow_flex": -0.2, ...} }
    Velocity values are joystick deflection in [-0.5, +0.5].
    The server scales them and continuously applies to the target position.
    Send all-zeros (or empty) to stop movement.
    """
    sid = _get_session_id()
    if not _check_control(sid):
        return jsonify({"success": False, "message": "You do not have control. Acquire first."}), 403

    if robot is None:
        return jsonify({"success": False, "message": "Robot not connected"}), 500

    data = request.get_json(force=True)
    vel = data.get("velocity", {})

    valid_joints = set(move_ranges.keys()) if move_ranges else set()
    for k in vel:
        if k not in valid_joints:
            return jsonify({"success": False, "message": f"Unknown joint: {k}"}), 400

    # Refresh lock timeout on every interaction (heartbeat)
    _refresh_lock(sid)

    with state_lock:
        # Update velocity for specified joints; unspecified joints keep current velocity
        for joint_name, v in vel.items():
            # Scale joystick deflection to normalized-units/sec,
            # applying direction inversion for joints where physical direction
            # is opposite to the UI convention.
            direction = JOINT_DIRECTION.get(joint_name, 1.0)
            velocity[joint_name] = float(v) * VELOCITY_SCALE * direction

    return jsonify({"success": True})


@app.route("/api/set_gripper", methods=["POST"])
def api_set_gripper():
    """
    Set gripper to an absolute normalized target position.
    The control loop will drive the gripper to this position.
    Body JSON: { "value": 0.5 }
    """
    sid = _get_session_id()
    if not _check_control(sid):
        return jsonify({"success": False, "message": "You do not have control. Acquire first."}), 403

    if robot is None:
        return jsonify({"success": False, "message": "Robot not connected"}), 500

    data = request.get_json(force=True)
    value = float(data.get("value", 0.5))

    if value < 0.0 or value > 1.0:
        return jsonify({"success": False, "message": f"Gripper value must be in [0, 1], got {value}"}), 400

    # Refresh lock timeout on every interaction (heartbeat)
    _refresh_lock(sid)

    with state_lock:
        target_normalized["gripper"] = value

    return jsonify({"success": True})


@app.route("/api/emergency_stop", methods=["POST"])
def api_emergency_stop():
    """
    Emergency stop: immediately halt all movement and return the arm
    to the saved initial position (recorded at acquire time).
    The control loop is paused during the move, then restarted.
    """
    sid = _get_session_id()
    if not _check_control(sid):
        return jsonify({"success": False, "message": "You do not have control."}), 403

    if robot is None:
        return jsonify({"success": False, "message": "Robot not connected"}), 500

    # Refresh heartbeat
    _refresh_lock(sid)

    # 1. Immediately zero all velocities to stop movement
    with state_lock:
        for joint_name in velocity:
            velocity[joint_name] = 0.0

    # 2. Stop control loop so we can perform a smooth restore
    _stop_control_loop()

    data = request.get_json(force=True) if request.data else {}
    duration_ms = int(data.get("duration_ms", 2000))

    with control_lock:
        try:
            if saved_arm_state:
                print(f"Emergency stop: restoring arm to saved position: {saved_arm_state}")
                setRobotState(robot, saved_arm_state, move_ranges, expect_duration_ms=duration_ms)
                print("Emergency stop: arm restored to initial position.")
            else:
                print("Emergency stop: no saved position, performing stand_up instead.")
                stand_up(robot, move_ranges, expect_duration_ms=duration_ms)
                print("Emergency stop: arm returned to stand-up position.")
            obs = robot.get_observation()
            state = {k: round(obs[k], 3) for k in obs if k.endswith(".pos")}
        except Exception as e:
            logging.exception("Error during emergency stop")
            _init_target_from_robot()
            _start_control_loop()
            return jsonify({"success": False, "message": str(e)}), 500

    # 3. Re-initialize target from restored position and restart control loop
    _init_target_from_robot()
    _start_control_loop()
    return jsonify({"success": True, "state": state})


@app.route("/api/stand_up", methods=["POST"])
def api_stand_up():
    """Stop control loop, move the arm to the stand-up position, then restart."""
    sid = _get_session_id()
    if not _check_control(sid):
        return jsonify({"success": False, "message": "You do not have control."}), 403

    if robot is None:
        return jsonify({"success": False, "message": "Robot not connected"}), 500

    data = request.get_json(force=True) if request.data else {}
    duration_ms = int(data.get("duration_ms", 2000))

    # Stop control loop while performing stand_up
    _stop_control_loop()

    with control_lock:
        try:
            stand_up(robot, move_ranges, expect_duration_ms=duration_ms)
            obs = robot.get_observation()
            state = {k: round(obs[k], 3) for k in obs if k.endswith(".pos")}
        except Exception as e:
            logging.exception("Error during stand_up")
            # Restart control loop even on error
            _init_target_from_robot()
            _start_control_loop()
            return jsonify({"success": False, "message": str(e)}), 500

    # Re-initialize target from new position and restart control loop
    _init_target_from_robot()
    _start_control_loop()
    return jsonify({"success": True, "state": state})


@app.route("/api/get_state", methods=["GET"])
def api_get_state():
    """Get the current robot joint positions and target positions.

    Anti-jitter: acquiring the hardware lock here blocks the 50Hz control
    loop, which can cause frame drops and servo twitching.  We therefore
    use a very short timeout and, if we cannot acquire the lock, return
    the last-known state instead of touching the hardware bus.
    """
    if robot is None:
        return jsonify({"success": False, "message": "Robot not connected"}), 503

    # Refresh lock timeout on polling (heartbeat)
    sid = _get_session_id()
    if _check_control(sid):
        _refresh_lock(sid)

    acquired = control_lock.acquire(timeout=0.005)
    if acquired:
        try:
            state, normalized = _get_normalized_state()
            with state_lock:
                target = {k: round(v, 4) for k, v in target_normalized.items()}
            return jsonify({
                "success": True,
                "state": state,
                "normalized": normalized,
                "target": target,
            })
        except Exception as e:
            return jsonify({"success": False, "message": str(e)}), 500
        finally:
            control_lock.release()
    else:
        # Could not acquire lock — return target as best-effort state
        # to avoid blocking the control loop and causing servo jitter.
        with state_lock:
            target = {k: round(v, 4) for k, v in target_normalized.items()}
            # Synthesize state from target (approximate)
            synth_state = {}
            synth_normalized = {}
            for joint_name, norm_val in target_normalized.items():
                synth_normalized[joint_name] = round(norm_val, 4)
                if joint_name in move_ranges and move_ranges[joint_name] is not None:
                    lo, hi = move_ranges[joint_name]
                    synth_state[joint_name + ".pos"] = round(lo + norm_val * (hi - lo), 3)
        return jsonify({
            "success": True,
            "state": synth_state,
            "normalized": synth_normalized,
            "target": target,
        })


# ── Frontend ─────────────────────────────────────────────────────────

@app.route("/")
def index():
    return render_template("index.html")


# ── Main ─────────────────────────────────────────────────────────────
def _get_all_ips():
    """Get all non-loopback IPv4 addresses of this machine."""
    import socket
    ips = []
    try:
        import netifaces
        for iface in netifaces.interfaces():
            addrs = netifaces.ifaddresses(iface)
            if netifaces.AF_INET in addrs:
                for addr in addrs[netifaces.AF_INET]:
                    ip = addr.get("addr", "")
                    if ip and ip != "127.0.0.1":
                        ips.append((iface, ip))
    except ImportError:
        # Fallback: parse ifconfig output (macOS / Linux)
        import subprocess
        try:
            out = subprocess.check_output(
                ["ifconfig"], stderr=subprocess.DEVNULL, text=True
            )
            current_iface = ""
            for line in out.splitlines():
                if not line.startswith("\t") and not line.startswith(" "):
                    current_iface = line.split(":")[0]
                if "inet " in line:
                    parts = line.strip().split()
                    idx = parts.index("inet") + 1
                    ip = parts[idx]
                    if ip != "127.0.0.1":
                        ips.append((current_iface, ip))
        except Exception:
            pass
    # Fallback: socket method
    if not ips:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ips.append(("default", s.getsockname()[0]))
            s.close()
        except Exception:
            pass
    return ips

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Robot Arm HTTP Server")
    parser.add_argument("--host", default="0.0.0.0", help="Bind address (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=8022, help="Port (default: 8022)")
    args = parser.parse_args()

    _init_robot()
    all_ips = _get_all_ips()
    print("=" * 50)
    print("  Robot Arm HTTP Server")
    print(f"  Local:   http://localhost:{args.port}")
    for iface, ip in all_ips:
        print(f"  Network: http://{ip}:{args.port}  ({iface})")
    if not all_ips:
        print("  Network: (no network interfaces detected)")
    print("=" * 50)
    app.run(host=args.host, port=args.port, debug=False, threaded=True)
