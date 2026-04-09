"""
SO-ARM101 + GR00T N1.6-DROID 推理客户端
=====================================
机械臂: SO-ARM101（6舵机：5臂关节 + 1夹爪）
Embodiment: OXE_DROID（关节空间控制，需两个相机）

注意： nvidia/GR00T-N1.6-DROID 拉取 7 个关节的动作。
      SO-ARM101 只有5个臂关节，输入时用零布丁第 6、7个关节位置。
      输出动作只取前 5 个分量发送到机械臂。

使用步骤：
  1. 修改 "===== 用户配置区 =====" 中的参数
  2. 在 MyRobot 类中填入您的 SDK 调用（有4处 TODO）
  3. 先启动服务器（Terminal 1），再运行本脚本（Terminal 2）

启动服务器：
  HF_HUB_OFFLINE=1 uv run python gr00t/eval/run_gr00t_server.py \\
      --model-path nvidia/GR00T-N1.6-DROID \\
      --embodiment-tag OXE_DROID \\
      --host 0.0.0.0 --port 5555

运行客户端：
  uv run python my_robot_client.py
"""

import time
import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation
from gr00t.policy.server_client import PolicyClient


# =====================================================================
# ===== 用户配置区 =====
# =====================================================================

SERVER_HOST = "192.168.31.103"   # 推理服务器 IP（本机填 localhost，远程填 IP）
SERVER_PORT = 5555          # 与服务器 --port 一致

TASK_DESCRIPTION = "pick up the cup"  # 任务语言描述（英文）

CONTROL_FREQ = 30           # 控制频率 Hz
URDF_PATH    = "so_arm101.urdf"  # URDF 路径（相对于运行目录）

# =====================================================================


# =====================================================================
# 正向运动学（FK）—— 基于 so_arm101.urdf，无外部依赖
# 关节顺序: shoulder_pan → shoulder_lift → elbow_flex → wrist_flex → wrist_roll
# 末端参考坐标系: gripper_frame_link（URDF 中的固定辅助坐标系）
# =====================================================================

# SO-ARM101 关节角度限位（弧度）
_ARM_LIMITS_LOW  = np.array([-1.91986, -1.74533, -1.69,    -1.65806, -2.74385])
_ARM_LIMITS_HIGH = np.array([ 1.91986,  1.74533,  1.69,     1.65806,  2.84121])
_GRIPPER_LOW, _GRIPPER_HIGH = -0.174533, 1.74533


def _make_tf(xyz, rpy):
    """从 URDF origin (xyz, rpy) 构造 4×4 齐次变换矩阵。
    URDF rpy 约定: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    """
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    R = np.array([
        [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [-sp,    cp*sr,             cp*cr            ],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = xyz
    return T


def _rotz(q):
    """绕 Z 轴旋转 q 弧度的 4×4 齐次变换矩阵（all SO-ARM101 joints use axis z）。"""
    T = np.eye(4)
    T[0, 0] =  np.cos(q);  T[0, 1] = -np.sin(q)
    T[1, 0] =  np.sin(q);  T[1, 1] =  np.cos(q)
    return T


# 各关节 origin 固定变换（直接从 so_arm101.urdf 提取）
_TF_ORIGINS = [
    _make_tf([ 0.0388353, -8.97657e-09,  0.0624],    [ 3.14159,      4.18253e-17, -3.14159]),  # shoulder_pan
    _make_tf([-0.0303992, -0.0182778,   -0.0542],    [-1.5708,      -1.5708,       0      ]),  # shoulder_lift
    _make_tf([-0.11257,   -0.028,        1.73763e-16], [-3.63608e-16,  8.74301e-16,  1.5708]),  # elbow_flex
    _make_tf([-0.1349,     0.0052,       3.62355e-17], [ 4.02456e-15,  8.67362e-16, -1.5708]),  # wrist_flex
    _make_tf([ 5.55112e-17,-0.0611,      0.0181],    [ 1.5708,        0.0486795,    3.14159]),  # wrist_roll
    _make_tf([-0.0079,    -0.000218121, -0.0981274], [ 0,             3.14159,      0      ]),  # gripper_frame (fixed)
]


def forward_kinematics(q: np.ndarray) -> np.ndarray:
    """正向运动学：5个臂关节角度 → 末端 4×4 变换矩阵。
    q: shape=(5,)，单位弧度，顺序 [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll]
    """
    T = np.eye(4)
    for i in range(5):
        T = T @ _TF_ORIGINS[i] @ _rotz(q[i])
    T = T @ _TF_ORIGINS[5]   # gripper_frame 固定偏移
    return T


def fk_to_pose(q: np.ndarray) -> np.ndarray:
    """返回末端位姿 [x, y, z, roll, pitch, yaw]（单位：米 / 弧度）。"""
    T   = forward_kinematics(q)
    xyz = T[:3, 3]
    rpy = Rotation.from_matrix(T[:3, :3]).as_euler("xyz")
    return np.concatenate([xyz, rpy]).astype(np.float32)


def inverse_kinematics(target_pose: np.ndarray, initial_q: np.ndarray) -> np.ndarray:
    """数值逆运动学：目标末端位姿 → 关节角度。
    target_pose: [x, y, z, roll, pitch, yaw]
    initial_q:   shape=(5,)，用当前关节角度作为初值（迭代收敛更快）
    返回:         shape=(5,) 目标关节角度，已裁剪到关节限位内
    """
    t_xyz = target_pose[:3]
    t_rot = Rotation.from_euler("xyz", target_pose[3:]).as_matrix()

    def cost(q):
        T     = forward_kinematics(q)
        p_err = np.linalg.norm(T[:3, 3] - t_xyz) * 10.0   # 位置误差（权重×10）
        r_err = np.linalg.norm(T[:3, :3] - t_rot)          # 姿态误差
        return p_err + r_err

    bounds = list(zip(_ARM_LIMITS_LOW, _ARM_LIMITS_HIGH))
    result = minimize(cost, initial_q, method="L-BFGS-B", bounds=bounds,
                      options={"maxiter": 300, "ftol": 1e-7})
    return result.x.astype(np.float32)


# =====================================================================
# 机器人接口（根据您的 SDK 填写 4 处 TODO）
# =====================================================================

class MyRobot:
    """SO-ARM101 控制接口。替换 TODO 部分为您的 SDK 调用。"""

    def __init__(self):
        # TODO 1/4：初始化 SDK，连接机械臂
        # 例如使用 lerobot SO101Follower:
        #   from lerobot.common.robots.so101_follower import SO101Follower
        #   self.arm = SO101Follower(port="/dev/ttyUSB0", id="my_arm")
        #   self.arm.connect()
        #   self.arm.calibrate_if_needed()
        self._sim_q        = np.zeros(5, dtype=np.float32)  # 模拟关节状态（调试用）
        self._sim_gripper  = 0.0

    # ------------------------------------------------------------------
    def get_arm_joint_angles(self) -> np.ndarray:
        """读取5个臂关节角度，shape=(5,)，单位弧度。
        顺序：shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll

        TODO 2/4：替换为实际 SDK
        """
        # 示例（lerobot）:
        #   obs = self.arm.get_observation()
        #   return np.array([
        #       obs["observation.state"][0],  # shoulder_pan
        #       obs["observation.state"][1],  # shoulder_lift
        #       obs["observation.state"][2],  # elbow_flex
        #       obs["observation.state"][3],  # wrist_flex
        #       obs["observation.state"][4],  # wrist_roll
        #   ], dtype=np.float32)
        return self._sim_q.copy()  # 删除此行，替换为上方真实调用

    def get_gripper_angle(self) -> float:
        """读取夹爪关节角度，单位弧度，范围 [_GRIPPER_LOW, _GRIPPER_HIGH]。

        TODO 3/4：替换为实际 SDK
        """
        # 示例（lerobot）:
        #   obs = self.arm.get_observation()
        #   return float(obs["observation.state"][5])  # gripper
        return self._sim_gripper  # 删除此行，替换为上方真实调用

    def get_camera_frame(self) -> np.ndarray:
        """读取相机图像，shape=(H, W, 3)，dtype=uint8，RGB 格式。
        OXE_WIDOWX 只需 1 个相机（第三视角 / 外部固定相机）。

        TODO 4/4：替换为实际相机 SDK
        """
        # 示例（OpenCV）:
        #   import cv2
        #   ret, frame = self.cap.read()        # BGR
        #   return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # 转 RGB
        #
        # 示例（RealSense）:
        #   frames  = self.pipeline.wait_for_frames()
        #   color   = frames.get_color_frame()
        #   return np.asanyarray(color.get_data())  # 已是 RGB
        return np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)  # 删除此行，替换为真实调用

    # ------------------------------------------------------------------
    def set_arm_joint_angles(self, q: np.ndarray):
        """发送5个臂关节角度目标，shape=(5,)，单位弧度。
        注意：已在调用处裁剪到限位内，SDK 侧可再加软件限位。
        """
        # 示例（lerobot）:
        #   action = {"action": np.concatenate([q, [self.get_gripper_angle()]])}
        #   self.arm.send_action(action)
        self._sim_q = q.copy()   # 删除此行，替换为真实调用
        print(f"  [arm]     {np.round(q, 4)}")

    def set_gripper(self, angle: float):
        """发送夹爪目标角度，单位弧度。"""
        # 示例（lerobot）：上方 set_arm_joint_angles 里一起发送即可
        self._sim_gripper = float(np.clip(angle, _GRIPPER_LOW, _GRIPPER_HIGH))
        print(f"  [gripper] {self._sim_gripper:.4f} rad")


# =====================================================================
# 观测封装（OXE_DROID 格式）
# state keys: joint_position (7D), gripper_position (1D)  shape=(1,1,D)
# video keys: exterior_image_1_left, wrist_image_left       shape=(1,1,H,W,3)
# SO101 适配：5 关节副与两个零补充为 7，夹爪归一化到 [0,1]
# =====================================================================

def build_observation(robot: MyRobot) -> dict:
    q_arm     = robot.get_arm_joint_angles()  # (5,)
    q_gripper = robot.get_gripper_angle()      # scalar (rad)
    image     = robot.get_camera_frame()       # (H, W, 3) uint8
    # OXE_DROID 需要两个相机，腕部相机用同一张图填充（没有腕部相机时）
    # 如果有腕部相机，替换 wrist_image 即可
    wrist_image = image

    # SO101 动作彤8个分量，抜陈到 OXE_DROID 7关节格式（后两位填 0）
    q7 = np.concatenate([q_arm, np.zeros(2)]).astype(np.float32)  # (7,)

    # 夹爪归一化到 [0, 1]
    gripper_norm = float(np.clip(
        (q_gripper - _GRIPPER_LOW) / (_GRIPPER_HIGH - _GRIPPER_LOW), 0.0, 1.0
    ))

    return {
        "video": {
            "exterior_image_1_left": image[np.newaxis, np.newaxis],       # (1,1,H,W,3)
            "wrist_image_left":      wrist_image[np.newaxis, np.newaxis],  # (1,1,H,W,3)
        },
        "state": {
            "joint_position":   q7[np.newaxis, np.newaxis],                         # (1,1,7)
            "gripper_position": np.array([[[gripper_norm]]], dtype=np.float32),     # (1,1,1)
        },
        "language": {
            "annotation.language.language_instruction": [[TASK_DESCRIPTION]],
        },
    }


# =====================================================================
# 动作执行
# action keys: joint_position (7D 相对增量), gripper_position (1D 绝对)
# 只取前 5 个分量发送到 SO101
# =====================================================================

def execute_actions(robot: MyRobot, actions: dict, q_current: np.ndarray) -> np.ndarray:
    """逐步执行一个 action chunk（32步），返回执行后的关节角度。"""
    n_steps = actions["joint_position"].shape[1]  # 32

    q = q_current.copy()  # (5,)
    for i in range(n_steps):
        # OXE_DROID 动作是相对关节增量，只取前 5 个分量
        delta = actions["joint_position"][0, i, :5].astype(np.float32)  # (5,)
        q_target = q + delta

        # 关节限位安全裁剪
        q_target = np.clip(q_target, _ARM_LIMITS_LOW, _ARM_LIMITS_HIGH)

        # 夹爪：反归一化回弧度
        g_norm  = float(np.clip(actions["gripper_position"][0, i, 0], 0.0, 1.0))
        g_angle = g_norm * (_GRIPPER_HIGH - _GRIPPER_LOW) + _GRIPPER_LOW

        robot.set_arm_joint_angles(q_target)
        robot.set_gripper(g_angle)

        q = q_target
        time.sleep(1.0 / CONTROL_FREQ)

    return q


# =====================================================================
# 主循环
# =====================================================================

def run_control_loop(robot: MyRobot, client: PolicyClient):
    print(f"[INFO] 任务: {TASK_DESCRIPTION}")
    print("[INFO] 开始控制循环，按 Ctrl+C 退出...\n")

    client.reset()
    q = robot.get_arm_joint_angles()
    step = 0

    while True:
        t0 = time.time()

        obs        = build_observation(robot)
        actions, _ = client.get_action(obs)

        q = execute_actions(robot, actions, q)

        step += 1
        print(f"[INFO] Step {step} | 当前关节: {q.round(4)} | 耗时: {time.time()-t0:.2f}s")


def main():
    print(f"[INFO] 连接服务器 {SERVER_HOST}:{SERVER_PORT} ...")
    client = PolicyClient(host=SERVER_HOST, port=SERVER_PORT)

    if not client.ping():
        raise ConnectionError(
            f"无法连接推理服务器 {SERVER_HOST}:{SERVER_PORT}\n\n"
            "请先在另一个终端启动服务器：\n\n"
            "  uv run python gr00t/eval/run_gr00t_server.py \\\n"
            "      --model-path nvidia/GR00T-N1.6-3B \\\n"
            "      --embodiment-tag OXE_WIDOWX \\\n"
            "      --host 0.0.0.0 --port 5555\n"
        )
    print("[INFO] 服务器连接成功！")

    robot = MyRobot()
    try:
        run_control_loop(robot, client)
    except KeyboardInterrupt:
        print("\n[INFO] 已退出。")


if __name__ == "__main__":
    main()
