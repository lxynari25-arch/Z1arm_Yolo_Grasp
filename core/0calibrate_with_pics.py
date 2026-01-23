"""
用标定产生的图片和坐标直接计算矩阵，无需拍照
"""
from calibrate import Calibration
if __name__ == "__main__":
    dir_path = "/home/unitree/lxy_arm/robot_grasp/output/imgSave/Folder_2026_01_16_15_19_38"
    Calibration(size=(9, 6), size_step=2.5, img_num=16, cal_path=dir_path)()