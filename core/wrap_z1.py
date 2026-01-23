import numpy as np
import sys
import time
import random
from utils import print_and_write
from scipy.spatial.transform import Rotation as R

sys.path.append("/home/unitree/lxy_arm/robot_grasp/core/unitree/z1_sdk/lib")
import unitree_arm_interface


class z1_arm(object):
    def __init__(self):
        try:
            self.arm = unitree_arm_interface.ArmInterface(hasGripper=True)
        except Exception as e:
            # 处理异常的代码
            print_and_write(None, f"Z1 connect failed, check if the connection is correct", 31)
            sys.exit()
        self.init_robot()
        self.armModel = self.arm._ctrlComp.armModel
        self.gripper_pos = 0.0
        self.jnt_speed = 2.0
        self.calibrate_space_file = "/home/unitree/lxy_arm/robot_grasp/core/lfn_data.txt"
        self.action_index = 0  # 追踪当前执行到文件的第几行
        


    def __call__(self, action):
        """Calls the move(action) method with given arguments to perform grasp."""
        return self.move(action)

    def init_robot(self):
        self.armState = unitree_arm_interface.ArmFSMState
        self.arm.loopOn()
        self.move_init_pose()

    def move_init_pose(self):
        self.arm.backToStart()

    """
    读取宇树z1动作文件，让机械臂逐行运动。格式为（r, p, y, x, y, z）
    """
    def calibrate_action(self):
        try:
            with open(self.calibrate_space_file, "r") as f:
                # 读取所有行并过滤掉空行
                lines = [line.strip() for line in f.readlines() if line.strip()]
                
            if not lines:
                print_and_write(None, "Error: The data file is empty.", 31)
                return

            # 按顺序获取当前索引对应的行，到达末尾时循环回开始
            current_line = lines[self.action_index % len(lines)]
            
            # 按空格分割字符串
            parts = current_line.split()
            
            # 检查是否包含 6 个数值
            if len(parts) == 6:
                try:
                    # 将字符串转换为浮点数列表 [roll, pitch, yaw, x, y, z]
                    action = [float(val) for val in parts]
                    print(f"Successfully selected pose (line {self.action_index + 1}): {action}")
                    
                    time.sleep(1)
                    # 执行动作
                    self.arm.MoveJ(action, self.gripper_pos, self.jnt_speed)
                    
                    # 等待动作完成（根据需要调整时间）
                    time.sleep(1)
                    
                    # 更新索引，准备执行下一行
                    self.action_index += 1
                    
                except ValueError:
                    print(f"Error: Selected line contains invalid numbers: {current_line}")
            else:
                print(f"Error: Selected line has wrong length (expected 6): {current_line}")
                    
        except FileNotFoundError:
            print_and_write(None, f"Error: File {self.calibrate_space_file} not found.", 31)
        except Exception as e:
            print(f"An unexpected error occurred: {e}")

    def move(self, action):
        self.arm.MoveJ(action, self.gripper_pos, self.jnt_speed)
    
    def labelrun(self, label):
        self.arm.labelRun(label)

    """
    返回(x, y, z, r, p, y)格式的末端位姿
    """
    def get_current_pose(self):
        # get current joint angles from arm state
        q_current = self.arm.lowstate.getQ()
        # compute forward kinematics to end-effector (link index 6)
        T = self.armModel.forwardKinematics(q_current, 6)
        position = T[0:3, 3]
        rotation = R.from_matrix(T[0:3, 0:3])
        roll, pitch, yaw = rotation.as_euler('xyz', degrees=False)
        return (float(position[0]), float(position[1]), float(position[2]),
                float(roll), float(pitch), float(yaw))

