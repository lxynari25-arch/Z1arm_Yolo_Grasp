# 简化后的机械臂与夹爪测试脚本（移除相机与YOLO相关代码）
import sys
core_path = "/home/unitree/lxy_arm/robot_grasp/core"
sys.path.append(core_path)

# 导入 core 目录下的模块（示例）
import numpy as np
import time

import signal
import sys
import time
import argparse
from pathlib import Path
from wrap_z1 import z1_arm
from utils import *



FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]

# 全局变量，用于信号处理
robot = None

def signal_handler(signum, frame):
    """处理 Ctrl+C 信号，优雅地关闭程序"""
    print("\n检测到Ctrl+C，正在返回初始位置...")
    if robot is not None:
        try:
            robot.move_init_pose()
            print("机械臂已返回初始位置")
        except Exception as e:
            print(f"错误：移动机械臂时出现异常 {e}")
    print("程序已退出")
    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)

    time.sleep(10)

    robot = z1_arm()
    r, p, yaw = 0.0, 1.0, 0.0
    x, y, z = 0.43359, 0.2, 0.34225
    move_command = np.array([r, p, yaw, x, y, z], dtype=float)
    robot(move_command, -2)
    time.sleep(3)
    robot(move_command, 2)


    

    print("动作执行完成，程序结束")
