import sys
sys.path.append("../lib")
import unitree_arm_interface
import time
import numpy as np

print("Press ctrl+\ to quit process.")
time.sleep(10)

np.set_printoptions(precision=3, suppress=True)
arm = unitree_arm_interface.ArmInterface(hasGripper=True)
armModel = arm._ctrlComp.armModel
arm.setFsmLowcmd()

duration = 1000

for i in range(0, duration):
    arm.gripperQ = i/duration-1
    arm.setGripperCmd(arm.gripperQ, arm.gripperQd, arm.gripperTau)
    arm.sendRecv()# udp connection
    time.sleep(arm._ctrlComp.dt)

# arm.loopOn()
# arm.backToStart()
# arm.loopOff()
