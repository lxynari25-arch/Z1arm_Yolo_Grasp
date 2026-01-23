import sys
sys.path.append("/home/unitree/lxy_arm/robot_grasp/core/unitree/z1_sdk/lib")
import unitree_arm_interface
import time
import numpy as np
np.set_printoptions(precision=3, suppress=True)

import faulthandler
faulthandler.enable()

print("Press ctrl+\ to quit process.")

arm =  unitree_arm_interface.ArmInterface(hasGripper=True)
armState = unitree_arm_interface.ArmFSMState
arm.loopOn()


arm.backToStart()
arm.loopOff()