"""
彩色相机和深度相机测试程序，可以捕捉连续且匹配的图像对
"""
import numpy as np
import cv2
import time
import os
import datetime
from pathlib import Path
import sys

core_path = "/home/unitree/lxy_arm/robot_grasp/core"
sys.path.append(core_path)

# 将 Realsense2 类所在目录添加到系统路径
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

# 导入 Realsense2 类
from realsense2 import Realsense2

def save_realsense_data(rgb_img, depth_vis_img, depth_img, save_dir, timestamp):
    """
    保存相机数据到指定目录（接收最新的帧数据，而非从对象读取）
    
    Args:
        rgb_img: RGB图像数据
        depth_vis_img: 可视化深度图
        depth_img: 原始深度数据
        save_dir: 保存目录路径
        timestamp: 时间戳字符串
    """
    # 确保保存目录存在
    os.makedirs(save_dir, exist_ok=True)
    
    # 构造文件路径
    rgb_path = os.path.join(save_dir, f"rgb_{timestamp}.png")
    depth_vis_path = os.path.join(save_dir, f"depth_vis_{timestamp}.png")
    depth_raw_path = os.path.join(save_dir, f"depth_raw_{timestamp}.npy")
    
    try:
        # 检查数据有效性
        if rgb_img is None or depth_img is None:
            print(f"[{timestamp}] 数据无效，跳过保存")
            return
        
        # 保存RGB图像 (BGR格式)
        cv2.imwrite(rgb_path, rgb_img)
        
        # 保存可视化深度图（如果没有则生成）
        if depth_vis_img is None:
            depth_vis_img = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_img, alpha=0.1),
                cv2.COLORMAP_JET
            )
        cv2.imwrite(depth_vis_path, depth_vis_img)
        
        # 保存原始深度数据
        np.save(depth_raw_path, depth_img)
        
        print(f"[{timestamp}] 数据保存成功:")
        print(f"  - RGB: {rgb_path}")
        print(f"  - 深度可视化: {depth_vis_path}")
        print(f"  - 原始深度: {depth_raw_path}")
        
    except Exception as e:
        print(f"[{timestamp}] 保存数据时出错: {e}")

def main():
    # 配置参数
    SAVE_DIR = "/home/unitree/lxy_arm/robot_grasp/test_img"
    INTERVAL = 3  # 采集间隔（秒）
    CAMERA_ID = 0
    WIDTH = 640
    HEIGHT = 480
    FPS = 30
    
    print(f"开始采集Realsense数据，每隔{INTERVAL}秒保存一次")
    print(f"保存目录: {SAVE_DIR}")
    print("按 Ctrl+C 停止采集...")
    
    # 初始化相机
    try:
        realcam = Realsense2(
            id=CAMERA_ID,
            rgb_width=WIDTH,
            rgb_height=HEIGHT,
            rgb_fps=FPS,
            is_depth=True,
            is_colormap=True,
            is_filter=True,
            is_fps=False  # 关闭FPS显示
        )
        # 等待相机稳定并缓存几帧数据
        print("相机初始化中...")
        time.sleep(2)
        
        # 先读取几帧丢弃，确保后续拿到最新数据
        for _ in range(5):
            next(realcam)
            time.sleep(0.01)
            
    except Exception as e:
        print(f"相机初始化失败: {e}")
        return
    
    try:
        # 主循环：每隔3秒采集一次（使用迭代器获取最新同步帧）
        while True:
            # 记录采集开始时间（避免sleep导致间隔不准）
            start_time = time.time()
            
            # 生成时间戳（精确到毫秒）
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            
            # 关键：通过__next__获取最新的同步帧（RGB+深度）
            try:
                rgb_img, depth_vis_img, depth_img = next(realcam)
                # 保存最新的帧数据
                save_realsense_data(rgb_img, depth_vis_img, depth_img, SAVE_DIR, timestamp)
            except StopIteration:
                print(f"[{timestamp}] 无法获取相机帧数据")
                continue
            
            # 计算实际需要sleep的时间（补偿数据采集/保存耗时）
            elapsed = time.time() - start_time
            sleep_time = max(0, INTERVAL - elapsed)
            time.sleep(sleep_time)
            
    except KeyboardInterrupt:
        print("\n采集已停止")
    except Exception as e:
        print(f"程序异常: {e}")
    finally:
        print("程序结束")

if __name__ == "__main__":
    main()