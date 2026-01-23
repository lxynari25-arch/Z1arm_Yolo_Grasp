import sys
import time
import numpy as np

sys.path.append("../lib")
import unitree_arm_interface


def wait_until_stable(arm, vel_thresh=1e-3, pos_delta_thresh=1e-4, stable_seconds=1.0):
    dt = getattr(arm._ctrlComp, 'dt', 0.01)
    required = max(1, int(stable_seconds / dt))
    prev_q = arm.lowstate.getQ().copy()
    stable = 0
    while stable < required:
        q = arm.lowstate.getQ()
        qd = arm.lowstate.getQd()
        if np.linalg.norm(qd) < vel_thresh and np.linalg.norm(q - prev_q) < pos_delta_thresh:
            stable += 1
        else:
            stable = 0
        prev_q = q.copy()
        time.sleep(dt)


def main():
    arm = unitree_arm_interface.ArmInterface(hasGripper=True)
    arm.loopOn()
    time.sleep(0.2)

    # 切换到关节控制模式
    arm.setFsm(unitree_arm_interface.ArmFSMState.JOINTCTRL)
    time.sleep(0.5)

    armModel = arm._ctrlComp.armModel

    print("Interactive MoveJ: robot will move to a random target each loop.")
    print("After motion stops, press Enter to generate the next random target.")
    print("Press Ctrl+C to exit.")

    try:
        while True:
            # 随机生成末端目标位置（根据工作空间调整范围）
            #arm.MoveJ([0.5,0.1,0.1,0.5,-0.2,0.5], gripper_pos, jnt_speed)
            target_pos = np.array([
                np.random.uniform(0.45, 0.55),   # x
                np.random.uniform(0.05, 0.15),  # y
                np.random.uniform(0.10, 0.15)    # z
            ])
            target_rpy = np.array([0.5,-0.2,0.5])
            posture_target = np.hstack([target_pos, target_rpy])

            print('\n===================================')
            print('Target end-effector posture (cmd):', posture_target)

            # 使用 MoveJ 直接按末端姿态运动
            try:
                arm.MoveJ(posture_target, 0.5)
            except Exception as e:
                print('MoveJ call failed:', e)
                break

            # 等待机械臂静止
            wait_until_stable(arm, stable_seconds=1.0)

            # 读取并显示当前末端位姿
            q_final = arm.lowstate.getQ().copy()
            T_fk = armModel.forwardKinematics(q_final, 6)
            pose_fk = unitree_arm_interface.homoToPosture(T_fk)
            print('Motion complete. Current end-effector posture:', pose_fk)

            # 等待用户按回车继续，Ctrl+C 退出
            try:
                input('Press Enter for next random target (Ctrl+C to quit)')
            except EOFError:
                # In some environments input() may raise EOFError; treat as continue
                continue

    except KeyboardInterrupt:
        print('\nInterrupted by user, exiting.')
    finally:
        arm.backToStart()
        arm.loopOff()


if __name__ == '__main__':
    main()
