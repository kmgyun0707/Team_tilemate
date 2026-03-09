#!/usr/bin/env python3
import time
import rclpy
import DR_init
from onrobot import RG

# ----------------------------
# 로봇 설정 상수
# ----------------------------
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"

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

    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    time.sleep(1.0)

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


def move_relative(dx: float, dy: float, dz: float, dw: float=0.0, dp: float=0.0, dr:float=0.0):
    from DSR_ROBOT2 import posx, posj, movej, movel, mwait, wait, DR_BASE, get_current_posx

    cur, _ = get_current_posx(DR_BASE)
    target = [
        cur[0] + dx,
        cur[1] + dy,
        cur[2] + dz,
        cur[3] + dw,
        cur[4] + dp,
        cur[5] + dr,
    ]
    movel(posx(target), ref=DR_BASE, vel=30, acc=30)
    mwait()
    print(get_current_posx(DR_BASE))


def move_to_tile_place_position(placement_index: int):
    from DSR_ROBOT2 import (
        posj,
        movej,
        movel,
        mwait,
        get_current_posx,
        DR_BASE,
        posx,
    )



    # 기준 자세
    pre_place = posx([380.745, 77.112, 179.766,48.90013885498047, 179.9677276611328, 48.96963119506836])


    tile_wid = 70.0  # mm

    tile_offsets = {
        1: (-tile_wid,  tile_wid),   # 좌상
        2: (0.0,        tile_wid),   # 중상
        3: (tile_wid,   tile_wid),   # 우상
        4: (-tile_wid,  0.0),        # 좌중
        5: (0.0,        0.0),        # 중앙
        6: (tile_wid,   0.0),        # 우중
        7: (-tile_wid, -tile_wid),   # 좌하
        8: (0.0,       -tile_wid),   # 중하
        9: (tile_wid,  -tile_wid),   # 우하
    }

    if placement_index not in tile_offsets:
        raise ValueError(f"invalid placement_index: {placement_index}")

    dx, dz = tile_offsets[placement_index]

    print(
        f"[PLACE_TILE] placement_index={placement_index}, dx={dx:.3f}, dz={dz:.3f}"
    )

    # 2) 배치 전 기준 위치
    movel(pre_place, vel=VELOCITY, acc=ACC, ref=DR_BASE)
    mwait()

    # # 3) 타일 위치로 상대 이동
    # move_relative(dx, 100.0, dz)


def perform_task_once(i:int):
    """Home -> 툴 파지전 -> 툴 파지 -> 뒤로 빼기 -> 배치전 가운데 -> 배치 가운데"""
    print(f"[TASK] INDEX: {i} TILE  START! ")
    gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

    from DSR_ROBOT2 import posx, posj, movej, movel, mwait, wait, DR_BASE

    # ----------------------------
    # 기준 자세
    # ----------------------------
    JReady = posj([0, 0, 90, 0, 90, 0])

    # ----------------------------
    # 툴 파지전
    # ----------------------------
    tool_pre_grip_j = posx([238.763, -373.264, 431.064, 57.57767868041992, -179.96800231933594, -122.38433074951172])

    # ----------------------------
    # 툴 파지 위치
    # ----------------------------
    tool_grip_x = posx([238.763, -373.264, 331.064 - 205.0, 57.57767868041992, -179.96800231933594, -122.38433074951172])

    # ----------------------------
    # 툴 잡고 뒤로 빼기
    # ----------------------------
    retreat_x = posx([237.073, -227.493, 331.064- 205.0, 57.57767868041992, -179.96800231933594, -122.38433074951172])


    print("[TASK] 1. movej -> Home")
    movej(JReady, vel=VELOCITY, acc=ACC)
    gripper.open_gripper()
    mwait()
    wait(1.0)

    print("180도 회전")
    move_relative(0.0, 0.0, 0.0, 180.0)


    print("[TASK] 2. tool pick pre-position (joint)")
    movel(tool_pre_grip_j, vel=VELOCITY, acc=ACC, ref=DR_BASE)
    mwait()
    wait(0.5)

    print("[TASK] 5. tool grip position (cartesian fine approach)")
    movel(tool_grip_x, vel=VELOCITY, acc=ACC, ref=DR_BASE)
    mwait()
    wait(0.5)

    print("[TASK] 6. close gripper")
    gripper.close_gripper()
    wait(1.5)

    print("[TASK] 7. retreat with tool (joint)")
    movel(retreat_x, vel=VELOCITY, acc=ACC, ref=DR_BASE)
    mwait()
    wait(0.5)

    move_relative(0.0, 0.0, 0.0, -180.0)

    print("[TASK] 12. move to place center (cartesian fine pose)")
    move_to_tile_place_position(i) #1~9


    # print("[TASK] 13. bending motion")
    # move_relative(0.0, 0.0, 0.0, dw=10.0)

    print("[TASK] 15. return Home")
    movej(JReady, vel=VELOCITY, acc=ACC)
    mwait()

    print(f"[TASK] INDEX: {i} TILE  DONE")


def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("move_basic", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        initialize_robot()
        for i in range(1,10,1):
            perform_task_once(i)

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