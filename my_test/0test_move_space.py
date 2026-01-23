import sys
import numpy as np
import time

core_path = "../core"
sys.path.append(core_path)

# 导入 core 目录下的模块
from wrap_z1 import z1_arm

def move_and_wait(robot, r, p, yaw, x, y, z, delay=2.0):
    """封装移动函数，打印目标并等待"""
    target = np.array([r, p, yaw, x, y, z], dtype=float)
    print(f"[ACTION] Moving to: r={r}, p={p}, y={yaw} | Target: {target}")
    robot(target)
    time.sleep(delay)

if __name__ == "__main__":
    # 1. 初始化机械臂
    robot = z1_arm()
    robot.move_init_pose()
    
    # 2. 初始等待 10 秒
    print("[INFO] Initialization complete. Sleeping for 10 seconds...")
    time.sleep(15.0)

    # 固定位置坐标 (x, y, z) 设为 0（根据你代码中的初始逻辑）
    pos_x, pos_y, pos_z = 0.4, 0.0, 0.5

    # 3. 依次对 r, p, y 进行 -1, 1 变换
    # 坐标索引与其名称对应关系
    axes = [("Roll", 0), ("Pitch", 1), ("Yaw", 2)]

    try:
        for name, index in axes:
            print(f"\n--- Testing Axis: {name} ---")
            
            # 状态: -1
            current_params = [0.0, 0.0, 0.0]
            current_params[index] = -0.5
            move_and_wait(robot, *current_params, pos_x, pos_y, pos_z)

            # 状态: 回归 0
            current_params[index] = 0.0
            move_and_wait(robot, *current_params, pos_x, pos_y, pos_z)

            # 状态: 1
            current_params[index] = 0.5
            move_and_wait(robot, *current_params, pos_x, pos_y, pos_z)

            # 状态: 回归 0
            current_params[index] = 0.0
            move_and_wait(robot, *current_params, pos_x, pos_y, pos_z)

        print("\n[SUCCESS] All movements completed.")
        robot.move_init_pose()

    except Exception as e:
        print(f"[ERROR] Motion failed: {e}")

    print("End.")