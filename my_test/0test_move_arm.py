"""
测试机械臂是否可以运动到指定位置
"""
"""
实验数据：
1. 当p=1.5时，垂直于地面
"""
import sys
core_path = "../core"
sys.path.append(core_path)

# 导入 core 目录下的模块（示例）
import numpy as np
import time
from wrap_z1 import z1_arm


# ======================== 用户直接在这里设置目标位姿 ========================
# 末端姿态（单位：rad）
r = 0.0
p = 0.0
yaw = 0.0

# 末端位置（单位：m，基坐标系）
x = 0.46
y = 0.0
z = 0.0

TARGET_POSE = np.array([r, p, yaw, x, y, z], dtype=float)
# ===========================================================================

if __name__ == "__main__":
    print("[INFO] Target pose (r, p, yaw, x, y, z):")
    print(TARGET_POSE)

    # 初始化机械臂
    robot = z1_arm()
    robot.move_init_pose()
    time.sleep(1.0)

    try:
        # 直接执行末端控制
        robot(TARGET_POSE)

        # 如果 SDK 支持，读取并打印当前末端位姿
        current_pose = robot.get_current_pose()
        print("[SUCCESS] Current end-effector pose:")
        print(current_pose)

    except Exception as e:
        raise RuntimeError(
            f"[ERROR] Failed to move to target pose: {e}"
        )

    print("End.")
