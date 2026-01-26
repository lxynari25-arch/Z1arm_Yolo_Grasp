# 实现眼在手上的抓取流程
import signal
import sys
import time
import argparse
import numpy as np
from ultralytics import YOLO
from realsense2 import *
from wrap_z1 import z1_arm
from calibrate import Calibration
from pathlib import Path
from utils import *

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]

# 全局变量，用于信号处理
robot = None

def signal_handler(signum, frame):
    """处理Ctrl+C信号，优雅地关闭程序"""
    print("\n检测到Ctrl+C，正在返回初始位置...")
    if robot is not None:
        try:
            robot.move_init_pose()
            print("机械臂已返回初始位置")
        except Exception as e:
            print(f"错误：移动机械臂时出现异常 {e}")
    print("程序已退出")
    sys.exit(0)

def create_parser():
    print_and_write(None, "------------------------config------------------------", 33)
    parser = argparse.ArgumentParser(description='GRASP')                                      
    parser.add_argument('--date', type=str, default='test', help='calibration file')
    parser.add_argument('--debug', type=bool, default=False, help='enable debug mode to save images')
    parser.add_argument('--mode', type=int, default=2, choices=[1, 2],
                        help='运动模式: 1=固定姿态, 2=Z使用预设姿态拍照')
    return parser

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    
    
    parser = create_parser()
    args = parser.parse_args()

    # 初始化模型和机器人
    print("初始化模型和机械臂")
    model = YOLO(f"/home/unitree/lxy_arm/robot_grasp/ultralytics/yolov8n.pt", verbose=False)
    robot = z1_arm()
    realcam = Realsense2(0, 640, 480, 30)

    # 获取标定数据
    cal_data = np.load(f'{ROOT}/../output/imgSave/{args.date}/data.npy', allow_pickle=True).item()
    RT_cam2gri = cal_data['RT_cam2gripper']
    RT_obj2cam = np.identity(4)
    RT_gri2base = np.identity(4)

    # 移动到初始位姿
    if args.mode == 1:
        robot.move_init_pose()   
    elif args.mode == 2:
        robot.labelrun("upcamera")
    time.sleep(2)

    for img in realcam:
        rgb_img, depth_vis_img, depth_frame = img
        if len(np.array(rgb_img).shape) == 3:
            results = model(rgb_img, verbose=False)[0]
            img_xy = cal_center(rgb_img, results.names, results.boxes.data.tolist())
            if img_xy:
                print(f"img_xy: {img_xy}")
                real_xyz = realcam.pixel2point(img_xy)
                print(f"real_xyz: {real_xyz}")

                RT_obj2cam[:3, 3] = np.array(real_xyz)
                RT_gri2base = Calibration.Cartesian2Homogeneous(robot.get_current_pose())
                RT_obj2base = RT_gri2base.dot(RT_cam2gri.dot(RT_obj2cam))
                print(f"RT_obj2base: {RT_obj2base}")

                if args.debug:
                    save_debug_images(rgb_img, depth_vis_img, depth_frame)

                # 根据运动模式设置 r, p, y
                x, y, z = RT_obj2base[:3, 3]
                if args.mode == 1:
                    # 固定姿态模式
                    r, p, yaw = 0.0, 0.0, 0.0
                elif args.mode == 2:
                    r, p, yaw = 0.0, 1.0, 0.0

                move_command = np.array([r, p, yaw, x, y, z], dtype=float)
                print(f"\n[运动执行] 执行movej运动指令：")
                print(f"  -> 完整指令: {move_command}")
                robot(move_command)
                time.sleep(3)
                if args.mode == 1:
                    robot.move_init_pose()
                elif args.mode == 2:
                    robot.labelrun("upcamera")

    print("End.")
