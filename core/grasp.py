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
from config_manager import config_manager

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
    
    # 显示可用的配置
    available_configs = config_manager.list_available_configs()
    print_and_write(None, "可用的模型配置:", 33)
    for config_name, description in available_configs.items():
        print_and_write(None, f"  - {config_name}: {description}", 33)
    print_and_write(None, "")
    
    parser = argparse.ArgumentParser(description='GRASP')                                      
    parser.add_argument('--date', type=str, default='test', help='calibration file')
    parser.add_argument('--debug', type=bool, default=True, help='enable debug mode to save images')
    parser.add_argument('--mode', type=int, default=1, choices=[1, 2],
                        help='运动模式: 1=固定姿态, 2=使用预设姿态拍照')
    parser.add_argument('--config', type=str, default=None,
                        help='模型配置名称（如: yolo26n-objv1-150, yolo11n-object365-pen等）')
    return parser

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)

    parser = create_parser()
    args = parser.parse_args()
    
    # 处理配置切换
    if args.config:
        try:
            config_manager.set_current_config(args.config)
            print_and_write(None, f"已切换到配置: {args.config}", 32)
        except ValueError as e:
            print_and_write(None, f"错误: {e}", 31)
            sys.exit(1)
    
    # 打印当前配置信息
    config_manager.print_current_config()
    
    # 从配置文件获取模型路径
    yolo_path = config_manager.get_model_path()
    object_class_id = config_manager.get_class_id()
    object_name = config_manager.get_object_name()
    
    # 初始化模型和机器人
    print("初始化模型和机械臂")
    model = YOLO(yolo_path, verbose=False)

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
            # 置信度阈值修改了！！！
            results = model(rgb_img, conf = 0.01, verbose=False)[0]
            img_xy, names, confidence = cal_center(rgb_img, results.names, results.boxes.data.tolist(), object_class_id)
            if img_xy:
                print("**************************识别到物体**************************")
                print(f"img_xy: {img_xy}")
                real_xyz = realcam.pixel2point(img_xy)
                print(f"real_xyz: {real_xyz}")
                print(f"识别的物体名称为：{names}，置信度为：{confidence}")

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
                print(f"[运动执行] 执行movej运动指令：")
                print(f"完整指令: {move_command}")
                robot(move_command, -1)
                time.sleep(1)
                # robot.close_gripper()
                # 爪子闭合！！！
                robot(move_command, 1)
                if args.mode == 1:
                    robot.move_init_pose()
                elif args.mode == 2:
                    robot.labelrun("upcamera")

    print("End.")
