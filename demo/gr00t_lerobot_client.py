"""GR00T policy client wired into the lerobot robot framework.

Usage example:

PYTHONPATH=/g/robot/Isaac-GR00T:/g/robot/lerobot/src \
  '/c/Users/Grape/.conda/envs/lerobot/python.exe' demo/gr00t_lerobot_client.py \
  --robot.type=so101_follower \
  --robot.port=COM4 \
  --robot.id=my_awesome_follower_arm \
  --robot.max_relative_target=8.0 \
  --server_host=192.168.31.103 \
  --server_port=5555 \
  --task_description='pick up the cup'

Optional camera configuration:

  --robot.cameras='{exterior: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, wrist: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30}}'
"""

from dataclasses import dataclass
import time
import warnings

import numpy as np

from gr00t.policy.server_client import PolicyClient
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig  # noqa: F401
from lerobot.configs import parser
from lerobot.robots import RobotConfig, so_follower  # noqa: F401
from lerobot.robots.robot import Robot
from lerobot.robots.utils import make_robot_from_config
from lerobot.utils.import_utils import register_third_party_plugins

_ARM_LIMITS_LOW = np.array([-1.91986, -1.74533, -1.69, -1.65806, -2.74385], dtype=np.float32)
_ARM_LIMITS_HIGH = np.array([1.91986, 1.74533, 1.69, 1.65806, 2.84121], dtype=np.float32)
_ARM_JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)
_GRIPPER_KEY = "gripper.pos"


@dataclass
class Gr00TLerobotClientConfig:
    robot: RobotConfig
    server_host: str = "localhost"
    server_port: int = 5555
    task_description: str = "拿起纸巾"
    control_freq: float = 30.0
    joint_delta_scale: float = 1.0
    exterior_camera_key: str = "exterior"
    wrist_camera_key: str = "wrist"
    copy_exterior_to_wrist: bool = True
    image_height: int = 480
    image_width: int = 640
    print_actions: bool = True


class Gr00TLerobotAdapter:
    def __init__(self, robot: Robot, cfg: Gr00TLerobotClientConfig):
        self.robot = robot
        self.cfg = cfg
        self._warned_missing_camera = False

    def _fallback_image(self) -> np.ndarray:
        if not self._warned_missing_camera:
            warnings.warn(
                "No lerobot camera image found for GR00T observation. Falling back to a black frame.",
                RuntimeWarning,
                stacklevel=2,
            )
            self._warned_missing_camera = True
        return np.zeros((self.cfg.image_height, self.cfg.image_width, 3), dtype=np.uint8)

    def get_raw_observation(self) -> dict:
        return self.robot.get_observation()

    def get_joint_radians(self, observation: dict) -> np.ndarray:
        q_deg = np.array([observation[f"{name}.pos"] for name in _ARM_JOINT_NAMES], dtype=np.float32)
        return np.deg2rad(q_deg).astype(np.float32)

    def get_gripper_norm(self, observation: dict) -> float:
        return float(np.clip(observation.get(_GRIPPER_KEY, 0.0) / 100.0, 0.0, 1.0))

    def get_camera_frame(self, observation: dict, camera_key: str) -> np.ndarray:
        if camera_key in observation:
            return np.asarray(observation[camera_key], dtype=np.uint8)
        return self._fallback_image()

    def build_gr00t_observation(self) -> tuple[dict, np.ndarray]:
        raw_obs = self.get_raw_observation()
        q_arm = self.get_joint_radians(raw_obs)
        gripper_norm = self.get_gripper_norm(raw_obs)
        exterior = self.get_camera_frame(raw_obs, self.cfg.exterior_camera_key)
        if self.cfg.copy_exterior_to_wrist and self.cfg.wrist_camera_key not in raw_obs:
            wrist = exterior
        else:
            wrist = self.get_camera_frame(raw_obs, self.cfg.wrist_camera_key)

        q7 = np.concatenate([q_arm, np.zeros(2, dtype=np.float32)]).astype(np.float32)
        obs = {
            "video": {
                "exterior_image_1_left": exterior[np.newaxis, np.newaxis],
                "wrist_image_left": wrist[np.newaxis, np.newaxis],
            },
            "state": {
                "joint_position": q7[np.newaxis, np.newaxis],
                "gripper_position": np.array([[[gripper_norm]]], dtype=np.float32),
            },
            "language": {
                "annotation.language.language_instruction": [[self.cfg.task_description]],
            },
        }
        return obs, q_arm

    def execute_action_chunk(self, actions: dict, q_current: np.ndarray) -> np.ndarray:
        n_steps = actions["joint_position"].shape[1]
        q = q_current.copy()

        for step_idx in range(n_steps):
            delta = actions["joint_position"][0, step_idx, :5].astype(np.float32)
            q_target = q + delta * self.cfg.joint_delta_scale
            q_target = np.clip(q_target, _ARM_LIMITS_LOW, _ARM_LIMITS_HIGH)

            gripper_norm = float(np.clip(actions["gripper_position"][0, step_idx, 0], 0.0, 1.0))
            joint_action = {
                f"{name}.pos": float(value)
                for name, value in zip(_ARM_JOINT_NAMES, np.rad2deg(q_target), strict=True)
            }
            joint_action[_GRIPPER_KEY] = gripper_norm * 100.0

            self.robot.send_action(joint_action)

            if self.cfg.print_actions:
                print(f"  [arm]     {np.round(q_target, 4)}")
                print(f"  [gripper] {gripper_norm:.4f}")

            q = q_target
            time.sleep(1.0 / self.cfg.control_freq)

        return q


def run_control_loop(robot: Robot, client: PolicyClient, cfg: Gr00TLerobotClientConfig):
    adapter = Gr00TLerobotAdapter(robot, cfg)

    print(f"[INFO] Task: {cfg.task_description}")
    print("[INFO] Starting control loop. Press Ctrl+C to stop.\n")

    client.reset()
    _, q = adapter.build_gr00t_observation()
    step = 0

    while True:
        t0 = time.time()
        gr00t_obs, _ = adapter.build_gr00t_observation()
        actions, _ = client.get_action(gr00t_obs)
        q = adapter.execute_action_chunk(actions, q)

        step += 1
        print(f"[INFO] Step {step} | joints: {q.round(4)} | elapsed: {time.time() - t0:.2f}s")


@parser.wrap()
def main(cfg: Gr00TLerobotClientConfig):
    print(f"[INFO] Connecting GR00T server {cfg.server_host}:{cfg.server_port} ...")
    client = PolicyClient(host=cfg.server_host, port=cfg.server_port)
    if not client.ping():
        raise ConnectionError(
            f"Unable to reach GR00T server at {cfg.server_host}:{cfg.server_port}. "
            "Start the GR00T policy server first."
        )

    robot = make_robot_from_config(cfg.robot)
    robot.connect()

    print("[INFO] GR00T server connected.")
    try:
        run_control_loop(robot, client, cfg)
    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    register_third_party_plugins()
    main()