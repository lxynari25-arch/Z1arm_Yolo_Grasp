import sys
sys.path.append("../lib")
import unitree_arm_interface
import time
import numpy as np

np.set_printoptions(precision=4, suppress=True)

def main():
    arm = unitree_arm_interface.ArmInterface(hasGripper=True)

    # single-shot: request one status update from controller
    arm.sendRecv()

    # read current joint angles (Vec6)
    q = arm.lowstate.getQ()

    # compute forward kinematics -> 4x4 homogeneous matrix
    T = arm._ctrlComp.armModel.forwardKinematics(q)

    # convert homogeneous matrix to posture vector (rpyxyz) via binding
    posture = unitree_arm_interface.homoToPosture(T)

    # print results
    print("Homogeneous transform T:\n", T)
    print("Posture (rpyxyz):", posture)

if __name__ == '__main__':
    main()
