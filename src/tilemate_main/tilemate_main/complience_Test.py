#!/usr/bin/env python3
import rclpy
import DR_init
import time
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

# stop/pause (단독 스크립트에서 최소 구현)
STOP_SOFT = False
PAUSE = False

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


# ----------------------------
# Robot init
# ----------------------------
def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import (
        set_tool, set_tcp, get_tool, get_tcp,
        ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS,
        get_robot_mode, set_robot_mode,wait
    )

    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    ###########
    time.sleep(1.0)
    wait(1.0)

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


# ----------------------------
# Pause/Stop helpers
# ----------------------------
def wait_if_paused():
    global PAUSE, STOP_SOFT
    from DSR_ROBOT2 import wait
    while PAUSE and not STOP_SOFT:
        # time.sleep(0.05)
        wait(0.05)


def sleep_interruptible(sec: float, dt: float = 0.05) -> bool:
    global STOP_SOFT
    from DSR_ROBOT2 import wait
    t0 = time.time()
    while (time.time() - t0) < float(sec):
        if STOP_SOFT:
            return False
        wait_if_paused()
        # time.sleep(float(dt))
        wait(float(dt))
    return True


def check_abort() -> bool:
    global STOP_SOFT
    if STOP_SOFT:
        print("[ABORT] stop_soft=True")
        return True
    wait_if_paused()
    return bool(STOP_SOFT)


# ----------------------------
# Compliance/Force helpers
# ----------------------------
def disable_compliance():
    """힘/컴플라이언스 해제(에러나도 무시)"""
    from DSR_ROBOT2 import release_force, release_compliance_ctrl
    try:
        release_force()
    except Exception:
        pass
    try:
        release_compliance_ctrl()
    except Exception:
        pass


def read_ft_guess():
    """
    환경마다 함수명이 달라서 여러 후보를 순차 시도.
    반환: (fx, fy, fz, tx, ty, tz) 또는 None
    """
    import DSR_ROBOT2 as dr

    candidates = [
        # 흔한 후보들
        "get_external_force", "get_ext_force",
        "get_tool_force", "get_current_force",
        "get_external_wrench", "get_tool_wrench",
    ]

    for name in candidates:
        fn = getattr(dr, name, None)
        if callable(fn):
            try:
                v = fn()
                # (values, status) 형태인 경우
                if isinstance(v, (list, tuple)) and len(v) == 2 and isinstance(v[0], (list, tuple)):
                    v = v[0]
                if isinstance(v, (list, tuple)) and len(v) >= 6:
                    return tuple(float(x) for x in v[:6])
            except Exception:
                pass
    return None


# ----------------------------
# Smart twist compaction (with FT logging)
# ----------------------------
def smart_twist_compaction(timeout_s=20.0, log_dt=0.2):
    """
    접촉 탐색 + joint6 비비기(왕복) + 힘 로그 출력
    Returns: (ok, depth)
    """
    global STOP_SOFT
    STX_Z = 20

    STX_ROL = 10
    STX_PIT = 10
    STX_YAW = 10
    FZ=40.0 # 13 -> 20 -> 30

    print(f"설정값 Fz: {FZ} Stx_z: {STX_Z} Stx_roll: {STX_ROL} Stx_pitch: {STX_PIT} Stx_yaw: {STX_YAW}")

    from DSR_ROBOT2 import (
        set_ref_coord,
        task_compliance_ctrl,
        set_desired_force,
        check_force_condition,
        get_current_posx,
        get_current_posj,
        amovej,
        wait,
        DR_FC_MOD_REL,
        DR_AXIS_Z,
        DR_BASE,
        DR_TOOL,
        release_force,
        release_compliance_ctrl,
    )

    wait_if_paused()
    if STOP_SOFT:
        return (False, 0.0)


    # TOOL 기준 컴플라이언스 + 아래로 누르는 힘 설정
    set_ref_coord(DR_TOOL)
    task_compliance_ctrl(stx=[3000, 3000, STX_Z, STX_ROL, STX_PIT, STX_YAW], time=0.0)
    wait(0.2)
    set_desired_force(
        fd=[0, 0, 30.0, 0, 0, 0],
        dir=[0, 0, 1, 0, 0, 0],
        mod=DR_FC_MOD_REL
    )

    t0 = time.time()
    last_log = 0.0
    touched = False
    contact_z = 0.0
    contact_joint = None

    try:
        # -------------------------
        # 1) 접촉 탐색
        # -------------------------
        while (time.time() - t0) < float(timeout_s):
            wait_if_paused()
            if STOP_SOFT:
                return (False, 0.0)

            now = time.time()
            if (now - last_log) >= float(log_dt):
                ft = read_ft_guess()
                if ft:
                    fx, fy, fz, tx, ty, tz = ft
                    print(f"[FT][SEARCH] Fx={fx:7.2f} Fy={fy:7.2f} Fz={fz:7.2f} | "
                          f"Tx={tx:7.2f} Ty={ty:7.2f} Tz={tz:7.2f}")
                last_log = now

            # 원본 로직 유지
            if check_force_condition(DR_AXIS_Z, min=0, max=10.0) == -1:
                test,_ = get_current_posx(DR_BASE)
                print(f"압착전 좌표{test}")
                touched = True
                contact_pos, _ = get_current_posx(DR_BASE)
                contact_z = float(contact_pos[2])
                contact_joint = get_current_posj()
                print(f"[CONTACT] touched=True  contact_z={contact_z:.3f}")
                release_force()
                break

            wait(1.0)

        if not touched or contact_joint is None:
            print("[CONTACT] failed (timeout/no joint)")
            return (False, 0.0)

        # -------------------------
        # 2) 비비기(조인트6 왕복)
        # -------------------------
        
        

        set_desired_force(
            fd=[0, 0, FZ, 0, 0, 0], 
            dir=[0, 0, 1, 0, 0, 0],
            mod=DR_FC_MOD_REL
        )

        press_t0 = time.time()
        direction_flag = 1
        last_switch = time.time()
        last_log = 0.0

        while (time.time() - press_t0) < 2.0:
            if check_abort():
                return (False, 0.0)

            now = time.time()
            if (now - last_log) >= float(log_dt):
                ft = read_ft_guess()
                if ft:
                    fx, fy, fz, tx, ty, tz = ft
                    print(f"[FT][TWIST ] Fx={fx:7.2f} Fy={fy:7.2f} Fz={fz:7.2f} | "
                          f"Tx={tx:7.2f} Ty={ty:7.2f} Tz={tz:7.2f}")
                last_log = now

            if time.time() - last_switch > 0.5:
                direction_flag *= -1
                last_switch = time.time()

            target_joint = list(contact_joint)
            target_joint[5] = contact_joint[5] + (10.0 * direction_flag)  # joint6
            #amovej(target_joint, vel=80, acc=80)
            wait(0.1)

        # 원래 접촉 조인트로 복귀
        amovej(contact_joint, vel=40, acc=40)

        if not sleep_interruptible(1.0):
            return (False, 0.0)

        final_pos, _ = get_current_posx(DR_BASE)
        final_z = float(final_pos[2])
        depth = float(contact_z - final_z)

        print(f"[RESULT] final_pose={final_pos} depth={depth:.3f}")

        return (True, depth)

    finally:
        disable_compliance()
        wait(0.1)


# ----------------------------
# Main task
# ----------------------------
def perform_task_once():



    """Home -> pre_place -> smart_twist -> Home"""
    gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

    from DSR_ROBOT2 import posj, movej, mwait, wait, get_current_posx, DR_BASE,movel,posx

    def move_relative(dx: float, dy: float, dz: float):
        cur, _ = get_current_posx(DR_BASE)
        target = [cur[0] + dx, cur[1] + dy, cur[2] + dz, cur[3], cur[4], cur[5]]
        movel(posx(target), ref=DR_BASE, vel=30, acc=30)
        mwait()

    # # Home
    # JReady = posj([0, 0, 90, 0, 90, 0])
    # movej(JReady, vel=VELOCITY, acc=ACC)
    # mwait()

    # # 플레이싱 전 위치 (Joint)

    # pre_place = posj([-20.954, 15.683, 104.247, 80.223, 107.752, -32.848])
    # movej(pre_place, vel=VELOCITY, acc=ACC)
    # mwait()

    # move_relative(0.0,0.0,-100.0)

    # # ✅ 압착(스마트 트위스트) 실행
    # print("[SMART_TWIST] start")
    # ok, depth = smart_twist_compaction(timeout_s=20.0, log_dt=0.2)
    # print(f"[SMART_TWIST] ok={ok}, depth={depth:.3f}")

    # wait(1.0)
    # gripper.open_gripper()
    # wait(1.0)

    # print("[TASK] movej -> Home")
    # movej(JReady, vel=VELOCITY, acc=ACC)
    # mwait()

    # print("[TASK] DONE")

    JReady    = posj([0, 0, 90, 0, 90, 0])
    pre_place = posj([-25.894,29.976,114.271,67.522,103.095,-54.547])

    tile_wid = 70.0  # 타일 너비 (mm)

    tile_offsets = {
        1: (-tile_wid,  tile_wid),  # 좌상
        2: (   0.0,  tile_wid),  # 중상
        3: ( tile_wid,  tile_wid),  # 우상
        4: (-tile_wid,    0.0),  # 좌중
        5: (   0.0,    0.0),  # 중앙
        6: ( tile_wid,    0.0),  # 우중
        7: (-tile_wid, -tile_wid),  # 좌하
        8: (   0.0, -tile_wid),  # 중하
        9: ( tile_wid, -tile_wid),  # 우하
        # 1: ( tile_wid,  tile_wid),
        # 2: ( tile_wid,    0.0),
        # 3: ( tile_wid, -tile_wid),
    }

    for tile_num in range(1, 10):
        print(f"\n{'='*40}")
        print(f"[TILE {tile_num}/9] 시작")
        print(f"{'='*40}")

        dx, dz = tile_offsets[tile_num]

        # 1. 홈 이동
        movej(JReady, vel=VELOCITY, acc=ACC)
        mwait()

        # wait(1.0)
        # gripper.close_gripper()
        # wait(1.0)

        # 2. pre_place 이동 (Joint, 특이점 회피)
        movej(pre_place, vel=VELOCITY, acc=ACC)
        mwait()

        # 3. 타일 부착 위치로 이동 (x: 좌우, z: 높이)
        move_relative(dx, 0.0, dz)

        # 4. 압착 (smart_twist 내부에서 벽 접근 처리)
        print(f"[TILE {tile_num}] SMART_TWIST 시작")
        ok, depth = smart_twist_compaction(timeout_s=20.0, log_dt=1.0)
        print(f"[TILE {tile_num}] ok={ok}, depth={depth:.3f}")

        # 5. 그리퍼 오픈 (타일 릴리즈)
        # wait(1.0)
        # gripper.open_gripper()
        # wait(1.0)

        move_relative(0.0, -30.0, 0.0)

        # 6. 홈 복귀
        print(f"[TILE {tile_num}] 홈 복귀")
        movej(JReady, vel=VELOCITY, acc=ACC)
        mwait()

        print(f"[TILE {tile_num}/9] 완료")

    print("\n[TASK] 전체 9개 타일 완료!")


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("move_basic", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        initialize_robot()
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