"""
测试yolo是否可以得到正确的物体中心点坐标
"""
import cv2
import numpy as np
import sys
core_path = "/home/unitree/lxy_arm/robot_grasp/core"
sys.path.append(core_path)

def mark_point_on_image(image_path, coord, save_path=None):
	"""
	在图像上用红点标记指定坐标
	:param image_path: 图像路径
	:param coord: [x, y] 坐标
	:param save_path: 保存路径（可选）
	:param show: 是否显示图像（可选）
	"""
	img = cv2.imread(image_path)
	if img is None:
		raise FileNotFoundError(f"无法读取图像: {image_path}")
	x, y = int(coord[0]), int(coord[1])
	cv2.circle(img, (x, y), radius=5, color=(0, 0, 255), thickness=-1)  # 红色圆点
	if save_path:
		cv2.imwrite(save_path, img)

# 示例用法：
# mark_point_on_image('test.jpg', [100, 200], save_path='marked.jpg')

if __name__ == "__main__":
	path = "/home/unitree/lxy_arm/robot_grasp/output/imgSave/debug/rgb_image_20260122_142321_019.png"
	xy = [238, 200]
	mark_point_on_image(path, xy, save_path='/home/unitree/lxy_arm/robot_grasp/output/imgSave/debug/marked.jpg')
