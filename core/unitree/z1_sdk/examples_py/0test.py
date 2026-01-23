import sys
sys.path.append("../lib")
import unitree_arm_interface
import time
import numpy as np
np.set_printoptions(precision=3, suppress=True)

print("Press ctrl+\ to quit process.")

arm =  unitree_arm_interface.ArmInterface(hasGripper=True)
armState = unitree_arm_interface.ArmFSMState
arm.loopOn()

arm.labelRun("forward")
arm.startTrack(armState.SAVESTATE)

arm.backToStart()
arm.loopOff()

