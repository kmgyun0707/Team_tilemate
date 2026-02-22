#타일 배치 + 경로 생성 예제 코드입니다.
# 작업영역 4점과 pick 위치를 입력으로 받아서,
# 1) 작업영역의 BBox 계산
# 2) BBox를 격자로 분할하여 Occupancy Grid 생성
# 3) Greedy 알고리즘으로 겹침 없이 타일 배치 (배치 중심점 + 그리드 좌표)
#!/usr/bin/env python3
"""
ROS2 + DSR_ROBOT2
Tile placement (Occupancy Grid + Greedy) + Waypoint path (Pick -> Place -> Pick)

- 입력: 작업영역 4점(posx), pick_above(출발 위치)
- 출력: 단계별 print 로그 + 배치 중심점 + waypoint(posx) 리스트
- 실행: 생성된 waypoint 순서대로 movel 수행
"""

import math
import time
import rclpy
import DR_init

# ----------------------------
# 로봇 설정
# ----------------------------
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

VELOCITY = 40
ACC = 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


# ----------------------------
# 입력값 (사용자 제공)
# ----------------------------
P1 = [364, 193, 200, 34, -179, 34]
P2 = [579, 193, 200, 7, 178, 7]
P3 = [367, 8,   200, 107, -180, 107]
P4 = [577, -9,  200, 173, -178, 173]

pick_above = [344, -101, 200, 50, 179, 140]  # 60x60 타일 출발 위치 (posx)

TILE_SIZE_MM = 60 # 타일 크기 (mm 단위, 정사각형)
TILE_QTY = 4  # 필요시 증가


# ----------------------------
# 유틸 (배치/경로)
# ----------------------------
def compute_bbox(points_xy): # 작업영역 4점으로 BBox 계산 (xmin, xmax, ymin, ymax)
    xs = [p[0] for p in points_xy]
    ys = [p[1] for p in points_xy]
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    return xmin, xmax, ymin, ymax


def generate_grid(xmin, xmax, ymin, ymax, unit_mm): # 작업영역을 unit_mm 단위의 격자로 분할하여 Occupancy Grid 생성
    x_cells = int(math.floor((xmax - xmin) / unit_mm)) # 격자 셀 수 계산 (x, y 방향)
    y_cells = int(math.floor((ymax - ymin) / unit_mm)) 
    occ = [[False] * x_cells for _ in range(y_cells)] # Occupancy Grid 초기화 (False: 빈 공간, True: 점유된 공간)
    return x_cells, y_cells, occ


# def greedy_place_tiles(xmin, xmax, ymin, ymax, occ, unit_mm, tile_size_mm, tile_qty, start="Top-Left", scan="Zigzag"): # 겹침 없이 타일 배치 (Greedy)
#     x_cells = len(occ[0]) if occ else 0
#     y_cells = len(occ) if occ else 0
#     if x_cells <= 0 or y_cells <= 0:
#         return [], {"failed": tile_qty}

#     GAP_MM = 5.0  # 원하는 간격(mm). 0이면 딱 붙음
#     effective_size = tile_size_mm + GAP_MM
#     w_cells = int(math.ceil(effective_size / unit_mm))
#     footprint_mm = w_cells * unit_mm
#     margin = tile_size_mm / 2.0

#     candidates = []
#     gy_range = range(0, y_cells)
#     if start.startswith("Bottom"):
#         gy_range = reversed(range(0, y_cells))

#     for gy in gy_range:
#         row = list(range(0, x_cells))
#         if scan == "Zigzag":
#             if (gy % 2) == 1:
#                 row = list(reversed(row))
#         for gx in row:
#             candidates.append((gx, gy))

#     placed = []
#     failed = 0

#     for i in range(tile_qty):
#         found = False
#         for gx, gy in candidates:
#             if gx + w_cells > x_cells or gy + w_cells > y_cells:
#                 continue

#             cx = xmin + (gx * unit_mm) + (footprint_mm / 2.0)
#             cy = ymax - (gy * unit_mm) - (footprint_mm / 2.0)

#             if (cx - margin) < xmin or (cx + margin) > xmax or (cy - margin) < ymin or (cy + margin) > ymax:
#                 continue

#             ok = True
#             for yy in range(gy, gy + w_cells):
#                 for xx in range(gx, gx + w_cells):
#                     if occ[yy][xx]:
#                         ok = False
#                         break
#                 if not ok:
#                     break

#             if ok:
#                 for yy in range(gy, gy + w_cells):
#                     for xx in range(gx, gx + w_cells):
#                         occ[yy][xx] = True

#                 placed.append({"index": i + 1, "center_x": cx, "center_y": cy, "gx": gx, "gy": gy})
#                 found = True
#                 break

#         if not found:
#             failed += 1

#     return placed, {"failed": failed}

def greedy_place_tiles(
    xmin, xmax, ymin, ymax,
    occ, unit_mm,
    tile_size_mm, tile_qty,
    start="Top-Left", scan="Zigzag"
):
    """겹침 없이 타일을 탐욕적으로 배치 (Greedy) + 열/행 정렬(피치 스텝)"""

    x_cells = len(occ[0]) if occ else 0
    y_cells = len(occ) if occ else 0
    if x_cells <= 0 or y_cells <= 0:
        return [], {"failed": tile_qty}

    # ===== 간격/피치 설정 =====
    GAP_MM = 5.0  # 타일 간격(mm) / 0이면 딱 붙음
    pitch_mm = tile_size_mm + GAP_MM

    # 피치를 격자 단위로 양자화(정렬 기준)
    pitch_cells = max(1, int(round(pitch_mm / unit_mm)))
    pitch_mm_q = pitch_cells * unit_mm  # 실제 배치 피치(mm)

    # 점유(footprint)도 피치 기준으로 고정 -> 항상 열/행 정렬됨
    w_cells = pitch_cells
    footprint_mm = pitch_mm_q

    # 실제 타일이 작업영역 밖으로 나가지 않게(타일 자체 기준)
    margin = footprint_mm / 2.0 

    # ===== 후보 생성: 1셀 스캔 금지, pitch_cells 스텝으로만 스캔 =====
    candidates = []
    gy_list = list(range(0, y_cells, pitch_cells))
    if start.startswith("Bottom"):
        gy_list = list(reversed(gy_list))

    for gy in gy_list:
        gx_list = list(range(0, x_cells, pitch_cells))

        # Zigzag는 "행 인덱스" 기준으로 뒤집기
        if scan == "Zigzag":
            row_idx = (gy // pitch_cells)
            if (row_idx % 2) == 1:
                gx_list = list(reversed(gx_list))

        for gx in gx_list:
            candidates.append((gx, gy))

    placed = []
    failed = 0

    for i in range(tile_qty):
        found = False

        for gx, gy in candidates:
            # 그리드 범위 체크
            if gx + w_cells > x_cells or gy + w_cells > y_cells:
                continue

            # 중심점(피치 기준 정렬)
            cx = xmin + (gx * unit_mm) + (footprint_mm / 2.0)
            cy = ymax - (gy * unit_mm) - (footprint_mm / 2.0)

            # 실제 타일이 BBox 밖으로 나가면 제외
            if (cx - margin) < xmin or (cx + margin) > xmax or (cy - margin) < ymin or (cy + margin) > ymax:
                continue

            # 점유(겹침) 체크
            ok = True
            for yy in range(gy, gy + w_cells):
                for xx in range(gx, gx + w_cells):
                    if occ[yy][xx]:
                        ok = False
                        break
                if not ok:
                    break

            if ok:
                # 점유 마킹
                for yy in range(gy, gy + w_cells):
                    for xx in range(gx, gx + w_cells):
                        occ[yy][xx] = True

                placed.append({"index": i + 1, "center_x": cx, "center_y": cy, "gx": gx, "gy": gy})
                found = True
                break

        if not found:
            failed += 1

    return placed, {"failed": failed}

def generate_waypoints(pick_posx, placed_centers, fixed_z=None, fixed_rpy=None): # Pick -> Place -> Pick 형태의 Waypoint 경로 생성
    waypoints = []

    px, py, pz, pr, pp, pyaw = pick_posx
    if fixed_z is None:
        fixed_z = pz
    if fixed_rpy is None:
        fixed_rpy = (pr, pp, pyaw)

    r, p, y = fixed_rpy

    waypoints.append([px, py, fixed_z, r, p, y])

    for t in placed_centers:
        cx, cy = t["center_x"], t["center_y"]
        waypoints.append([cx, cy, fixed_z, r, p, y])
        waypoints.append([px, py, fixed_z, r, p, y])

    return waypoints


def run_tile_pipeline_prints(): # 타일 배치/경로 생성 + 단계별 print 로그
    print("========== Tile Placement Pipeline ==========")

    print("[0단계] 입력값")
    print(f" - Pick(출발): {pick_above}")
    print(f" - Tile: {TILE_SIZE_MM}x{TILE_SIZE_MM} mm, qty={TILE_QTY}")
    print(f" - Map points (XY): P1={P1[:2]}, P2={P2[:2]}, P3={P3[:2]}, P4={P4[:2]}")

    print("\n[1단계] 작업영역(BBox) 계산")
    xmin, xmax, ymin, ymax = compute_bbox([P1[:2], P2[:2], P3[:2], P4[:2]])
    print(f" - xmin={xmin}, xmax={xmax}, ymin={ymin}, ymax={ymax}")
    print(f" - width={xmax-xmin} mm, height={ymax-ymin} mm")

    print("\n[2단계] Occupancy Grid 생성")
    unit_mm = 5 # 격자 단위 (mm), 타일 크기에 비해 충분히 작은 값으로 설정 (예: 5mm)
    x_cells, y_cells, occ = generate_grid(xmin, xmax, ymin, ymax, unit_mm)
    print(f" - unit={unit_mm} mm")
    print(f" - grid size = {x_cells} x {y_cells} (cells)")

    print("\n[2-2단계] Greedy Placement (겹침 없이 배치)")
    placed, info = greedy_place_tiles(
        xmin, xmax, ymin, ymax,
        occ, unit_mm,
        tile_size_mm=TILE_SIZE_MM,
        tile_qty=TILE_QTY,
        start="Top-Left",
        scan="Zigzag"
    )
    if placed:
        print(f" - placed: {len(placed)} tiles")
        for t in placed:
            print(f"   * tile#{t['index']}: center=({t['center_x']:.3f}, {t['center_y']:.3f})  cell=({t['gx']},{t['gy']})")
    else:
        print(" - placed: 0 tiles")

    if info["failed"] > 0:
        print(f" - failed: {info['failed']} tiles (공간 부족/조건 불만족)")

    print("\n[3단계] Waypoint 경로 생성 (Pick -> Place -> Pick)")
    waypoints = generate_waypoints(pick_above, placed)
    print(f" - waypoints count = {len(waypoints)}")
    for i, wp in enumerate(waypoints, start=1):
        print(f"   {i:02d}: posx({wp})")

    print("\n[완료] 전체 파이프라인 종료")
    print("============================================")

    return waypoints


# ----------------------------
# 로봇 초기화/실행
# ----------------------------
def initialize_robot(): # 로봇의 Tool과 TCP를 설정
    from DSR_ROBOT2 import (
        set_tool, set_tcp, get_tool, get_tcp,
        ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS,
        get_robot_mode, set_robot_mode,
    )

    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(2)

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


def perform_task(): # 로봇이 수행할 작업
    from DSR_ROBOT2 import posx, movej, movel, wait

    # 0) 배치/경로 생성 + print
    waypoints = run_tile_pipeline_prints()
    if not waypoints:
        print("[중단] waypoint가 0개라 이동을 수행하지 않습니다.")
        return

    # 1) 초기 자세
    JReady = [0, 0, 90, 0, 90, 0]
    print("[로봇] movej -> JReady")
    movej(JReady, vel=VELOCITY, acc=ACC)

    # 2) waypoint 순차 실행 (1회)
    print("[로봇] waypoint movel 시작")
    for i, wp in enumerate(waypoints, start=1):
        print(f"[로봇] {i:02d}/{len(waypoints)} movel -> {wp}")
        movel(posx(wp), vel=VELOCITY, acc=ACC)
        wait(2.0)  # 각 waypoint 사이에 잠시 대기 (필요시 조정)

    print("[로봇] waypoint movel 종료")


def main(args=None): # 메인 함수: ROS2 노드 초기화 및 동작 수행
    rclpy.init(args=args)
    node = rclpy.create_node("tile_place_auto", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        initialize_robot()
        perform_task()
    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()