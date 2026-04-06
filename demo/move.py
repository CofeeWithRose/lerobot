import logging
import math
import time
from lerobot.motors import MotorNormMode, MotorCalibration
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

# 初始化robot配置并创建robot对象

# 连接机械臂（不进行校准，保持当前位置为初始位置）

def initRobot():
    print("正在初始化机械臂...")
    config = SO101FollowerConfig(
    id="my_awesome_follower_arm",
    port="COM4",
    use_degrees=True,
    max_relative_target=8.0,  # 限制单次相对目标，避免“跳变”导致冲击
    )
    robot = SO101Follower(config)
    # 这里可以添加任何需要的初始化步骤，例如移动到安全位置等
    print("机械臂初始化完成！")
    robot.connect(calibrate=False)
    return robot

def cal_to_native_range(cal: MotorCalibration, norm_mode: MotorNormMode):
    """将校准数据转换为归一化值域 (min, max)。"""
    if norm_mode == MotorNormMode.RANGE_0_100:
        return (0.0, 100.0)
    mid = (cal.range_min + cal.range_max) / 2
    half = abs(cal.range_max - cal.range_min) / 2
    deg_half = half * 360.0 / 4095.0
    return (-deg_half, deg_half)


def getAllMotorMoveRange(robot):
    print("正在获取所有电机的移动范围...")
    move_ranges = {}
    for motor_name, motor in robot.bus.motors.items():
        cal = robot.bus.calibration.get(motor_name)
        norm_mode = motor.norm_mode
        if cal is not None:
            move_ranges[motor_name] = cal_to_native_range(cal, norm_mode)
        else:
            move_ranges[motor_name] = None
        print(f"{motor_name}: {move_ranges[motor_name]}")
    return move_ranges

# 根据目标位置，每次发送一个动作，并持续一段时间，直到达到目标位置.
def _clamp_targets(action: dict, move_ranges: dict) -> dict:
    """将 action 中的目标值 clamp 到关节限位，越界时输出警告。"""
    targets = {}
    for k, v in action.items():
        motor_name = k.removesuffix(".pos")
        if motor_name in move_ranges and move_ranges[motor_name] is not None:
            lo, hi = move_ranges[motor_name]
            if v < lo or v > hi:
                logging.warning(
                    f"_clamp_targets: {k}={v:.3f} 超出限位 [{lo:.3f}, {hi:.3f}]，已 clamp。"
                )
            v = max(lo, min(hi, v))
        targets[k] = v
    return targets


def _calc_min_duration_ms(targets: dict, starts: dict, max_rel, hz: int) -> int:
    """根据移动距离和每帧限幅计算最小插值时长（ms）。"""
    max_dist = max((abs(targets[k] - starts[k]) for k in targets), default=0.0)
    if max_rel is not None and max_rel > 0:
        min_from_speed_ms = math.ceil(max_dist / (max_rel * hz) * 1000)
    else:
        min_from_speed_ms = 0
    return max(min_from_speed_ms, math.ceil(2000 / hz), 100)


def _calc_waypoint(targets: dict, starts: dict, obs: dict, alpha: float, max_rel, tol_deg: float) -> dict:
    """计算当前帧的发送目标点，在 max_relative_target 内 clamp，接近终点时直接用目标值。"""
    distance = {k: (targets[k] - obs[k]) for k in targets}
    if max_rel is None or max_rel <= 0:
        return distance
    waypoint = {}
    for k in targets:
        cur = obs.get(k, distance[k])
        # 仅当实际位置已在容差内时才直接发目标值，否则始终 clamp
        if abs(distance[k]) <= tol_deg:
            waypoint[k] = targets[k]
        else:
            waypoint[k] = cur + max(-max_rel, min(max_rel, range_abs_clamp( 0.2 *distance[k], tol_deg)))
    return waypoint


def range_abs_clamp(value, min_val):
    """将 value clamp 到 [-inf, -min_val] U [min_val, inf]，即绝对值至少为 min_val。"""
    if abs(value) <= min_val:
        return min_val if value >= 0 else -min_val
    return value


def _all_reached(obs: dict, targets: dict, tol_deg: float) -> bool:
    """检查所有关节是否均在容差内到位。"""
    return all(abs(obs.get(k, float("inf")) - targets[k]) < tol_deg for k in targets)


def send_action_timed(robot, action: dict, expect_duration_ms: int,
                      move_ranges: dict, max_duration_ms: int = 10000, hz: int = 50,
                      tol_deg: float = 6.0) -> bool:
    """线性插值驱动关节到目标位置，以实际到位为完成条件。

    Args:
        robot:              机械臂对象
        action:             目标动作，格式 {"joint.pos": value, ...}
        expect_duration_ms: 预期运动时长（ms），用于线性插值节奏
        move_ranges:        各关节限位字典，用于 clamp 目标值
        max_duration_ms:    硬超时（ms），超出后强制退出并报警
        hz:                 控制频率
        tol_deg:            到位容差（°或%），所有关节均在容差内视为到位

    Returns:
        True  — 所有关节在 max_duration_ms 内到位
        False — 超时未到位
    """
    interval = 1.0 / hz
    max_rel = getattr(robot.config, "max_relative_target", None)

    obs = robot.get_observation()
    starts = {k: obs.get(k, v) for k, v in action.items()}
    targets = _clamp_targets(action, move_ranges)
    print(f"send_action_timed: 目标值 {targets}")

    min_duration_ms = _calc_min_duration_ms(targets, starts, max_rel, hz)
    if expect_duration_ms < min_duration_ms:
        max_dist = max((abs(targets[k] - starts[k]) for k in targets), default=0.0)
        logging.warning(
            f"send_action_timed: expect_duration_ms={expect_duration_ms} 过小"
            f"（最小值 {min_duration_ms}ms，基于 max_dist={max_dist:.2f}, "
            f"max_relative_target={max_rel}, hz={hz}），已自动重置。"
        )
        expect_duration_ms = min_duration_ms

    T = expect_duration_ms / 1000.0
    T_max = max_duration_ms / 1000.0

    print(f"正在发送动作 {action}，预估持续时间 {expect_duration_ms}ms...")
    t_start = time.perf_counter()

    while True:
        elapsed = time.perf_counter() - t_start
        obs = robot.get_observation()

        if elapsed >= T_max:
            not_reached = {
                k: {"当前": round(obs.get(k, float("nan")), 3), "目标": round(targets[k], 3), "误差": round(abs(obs.get(k, float("nan")) - targets[k]), 3)}
                for k in targets
                if abs(obs.get(k, float("inf")) - targets[k]) >= tol_deg
            }
            logging.warning(f"send_action_timed: 超时 {max_duration_ms}ms，以下关节未到位：{not_reached}")
            return False

        alpha = min(elapsed / T, 1.0)
        waypoint = _calc_waypoint(targets, starts, obs, alpha, max_rel, tol_deg)
        robot.send_action(waypoint)

        if _all_reached(obs, targets, tol_deg):
            print(f"到位！实际耗时 {(time.perf_counter() - t_start) * 1000:.0f}ms")
            return True

        time.sleep(interval)


def send_action_timed_with_normalized(robot, action: dict, expect_duration_ms: int,
                                    move_ranges: dict, max_duration_ms: int = 10000, hz: int = 50, tol_deg: float = 6.0) -> bool:
    """send_action_timed 的包装函数，接受归一化目标值（0-1.0），并自动转换为实际值域。"""
    normalized_action = {}
    for k, v in action.items():
        if k in move_ranges:
            min_val, max_val = move_ranges[k]
            normalized_action[k+".pos"] = min_val + v * (max_val - min_val)
        else:
            raise ValueError(f"send_action_timed_with_normalized: 未知的关节 {k}，无法归一化")
    return send_action_timed(robot, normalized_action, expect_duration_ms, move_ranges, max_duration_ms, hz, tol_deg)



def check_overload(act_name, target, actual, load, current, temperature, step_info=""):
    """检查过载条件，抛出异常以触发安全停机。"""
    print(
        f"    [监测] {act_name} 目标 {target:.2f}° -> 实际 {actual:.2f}°, "
        f"负载 {load * 0.1:.1f}%, 电流 {current * 6.5:.0f}mA, 温度 {temperature}°C {step_info}"
    )
    if abs(load) > 80.0:
        raise RuntimeError(f"{act_name} 负载过高: {load * 0.1:.1f}% > 80.0%")
    if abs(current) > 15.0:
        raise RuntimeError(f"{act_name} 电流过高: {current * 6.5:.0f}mA > 15.0mA")
    if temperature > 80:
        raise RuntimeError(f"{act_name} 温度过高: {temperature}°C > 80°C")


def getRobotState(robot):
    obs = robot.get_observation()
    state = {k: obs[k] for k in obs if k.endswith(".pos")}
    print(f"当前机械臂状态: {state}")
    return state

def setRobotState(robot, target_state, move_ranges, expect_duration_ms=2000):
    print(f"正在设置机械臂状态到 {target_state}...")
    success = send_action_timed(robot, target_state, expect_duration_ms, move_ranges=move_ranges)
    if not success:
        print("⚠️ 设置状态超时，可能未完全到位，请检查机械臂状态！")
    else:
        print("状态设置完成！")


def stand_up(robot, move_ranges, expect_duration_ms=3000):
    print("正在将机械臂移动到站立状态...")
    safe_position = {
        "shoulder_pan": 0.7,  # 朝向中间
        "shoulder_lift": 0.5, # 尽量抬高肩部 1为抬高向前
        "elbow_flex": 0.3,    # 尽量伸直肘部 0为伸直
        "wrist_flex": 0.8,    # 尽量向下手腕 0为向上
        "wrist_roll": 0.5,
        "gripper": 0.5,  # 打开
    }
    send_action_timed_with_normalized(robot, safe_position, expect_duration_ms, move_ranges=move_ranges)
    print("机械臂已站立！")

robot = initRobot()
move_ranges = getAllMotorMoveRange(robot)
robot_init_state = getRobotState(robot)
print("所有电机的移动范围已获取！", move_ranges)
# 将elbow_flex移动到最大范围的负方向，持续500ms
elbow_flex_range = move_ranges.get("elbow_flex")
if elbow_flex_range is not None:
    # target_pos = elbow_flex_range[0]  # 负方向的极限位置
    # action = {"elbow_flex.pos": target_pos}
    # send_action_timed(robot, action, expect_duration_ms=500, move_ranges=move_ranges)     
    stand_up(robot, move_ranges, expect_duration_ms=500)  
    normalized_action = { "gripper": 0.0 }  # 归一化目标值，0.0 对应负方向极限
    send_action_timed_with_normalized(robot, normalized_action, expect_duration_ms=500, move_ranges=move_ranges)  
    
    normalized_action = { "gripper": 0.0, "elbow_flex": 0.5, "shoulder_pan": 0.0 }  # 归一化目标值，0.0 对应负方向极限
    send_action_timed_with_normalized(robot, normalized_action, expect_duration_ms=500, move_ranges=move_ranges) 
    
    normalized_action = { "gripper": 0.3}  # 归一化目标值，0.0 对应负方向极限
    send_action_timed_with_normalized(robot, normalized_action, expect_duration_ms=500, move_ranges=move_ranges)  # 每次移动后等待0.5秒  
    print("移动完成！")

setRobotState(robot, robot_init_state, move_ranges)  # 恢复初始状态
robot.disconnect()