#!/usr/bin/env python3
import rclpy
import DR_init
import time
from onrobot import RG


# 로봇 설정 상수 (필요에 따라 수정)
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
# 이동 속도 및 가속도 (필요에 따라 수정)
VELOCITY = 40
ACC = 60

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import (
        set_tool, set_tcp, get_tool, get_tcp,
        ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS,
        get_robot_mode, set_robot_mode
    )

    # Tool과 TCP 설정시 매뉴얼 모드로 변경해서 진행
    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)


    time.sleep(1.0)  # 설정 안정화 대기

    # 설정된 상수 출력
    print("#" * 50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {get_tcp()}")
    print(f"ROBOT_TOOL: {get_tool()}")
    print(f"ROBOT_MODE 0:수동, 1:자동 : {get_robot_mode()}")
    print(f"VELOCITY: {VELOCITY}")
    print(f"ACC: {ACC}")
    print("#" * 50)


def perform_task_once():
    """Home -> 파지전3 -> 파지전2 -> 파지전1 -> 파지"""
    gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

    from DSR_ROBOT2 import posx, posj, movej, movel, mwait, amovej, wait

    # Home
    JReady = posj([0, 0, 90, 0, 90, 0])
    movej(JReady, vel=VELOCITY, acc=ACC)
    gripper.open_gripper()
    mwait()

    # 파지전 위치 (Joint)
    pre_grip = posx([423.741, -229.421, 173.196, 141.769, 179.242, 142.081])
    movel(pre_grip, vel=VELOCITY, acc=ACC)
    gripper.set_target_width(160)
    mwait()

    # 파지 위치 (Cartesian)
    grip = posx([423.741, -229.421, 72.565, 141.769, 179.242, 142.081])
    movel(grip, vel=VELOCITY, acc=ACC)
    wait(1.0)
    gripper.close_gripper()
    wait(1.0)

    # 파지전 위치 (Joint)
    pre_grip = posx([423.741, -229.421, 173.196, 141.769, 179.242, 142.081])
    movel(pre_grip, vel=VELOCITY, acc=ACC)
    gripper.set_target_width(160)
    mwait()

    #플레이싱 전 위치 
    pre_place = posj([-20.954, 15.683, 104.247, 80.223, 107.752, -32.848])
    movej(pre_place,vel=VELOCITY, acc=ACC)
    wait(10.0)
    mwait()

    # 상대좌표 이동
    # 파지전 위치 (Joint)
    pre_grip = posx([423.741, -229.421, 173.196, 141.769, 179.242, 142.081])
    movel(pre_grip, vel=VELOCITY, acc=ACC)
    gripper.set_target_width(160)
    mwait()


    print("[TASK] movej -> Home")
    movej(JReady, vel=VELOCITY, acc=ACC)
    mwait()


    print("[TASK] DONE")


def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("move_basic", namespace=ROBOT_ID)

    # DR_init에 노드 설정
    DR_init.__dsr__node = node

    try:
        # 초기화는 한 번만 수행
        initialize_robot()

        # ✅ 1회 수행 후 종료
        perform_task_once()

    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()