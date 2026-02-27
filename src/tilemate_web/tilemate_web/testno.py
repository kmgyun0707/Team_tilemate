"""
testno_interactive.py
TILING MONITOR 인터랙티브 테스트 노드

index.html 공정 흐름에 맞춰 Firebase robot_status를 직접 조작합니다.
실제 firebase_bridge.py / ROS2 없이 웹 UI 동작을 검증할 수 있습니다.
"""

import time
import random
import firebase_admin
from firebase_admin import credentials, db
import os

SERVICE_ACCOUNT_KEY_PATH = os.path.expanduser(
    "~/Team_tilemate/src/tilemate_web/config/co1-tiling-firebase-adminsdk-fbsvc-f4f88c3832.json"
)
FIREBASE_DB_URL = "https://co1-tiling-default-rtdb.asia-southeast1.firebasedatabase.app"

TOTAL_TILES = 9

R     = "\033[0m"
BOLD  = "\033[1m"
CYAN  = "\033[96m"
YEL   = "\033[93m"
GRN   = "\033[92m"
RED   = "\033[91m"
GRAY  = "\033[90m"
BLUE  = "\033[94m"
MAG   = "\033[95m"

def c(color, text): print(f"{color}{text}{R}")

def init_firebase():
    if not os.path.exists(SERVICE_ACCOUNT_KEY_PATH):
        c(RED, f"오류: 키 파일 없음 → {SERVICE_ACCOUNT_KEY_PATH}")
        exit(1)
    if not firebase_admin._apps:
        cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
        firebase_admin.initialize_app(cred, {"databaseURL": FIREBASE_DB_URL})
    return db.reference("robot_status"), db.reference("robot_command")

def ask(prompt, default=None):
    suffix = f" [{default}]" if default is not None else ""
    val = input(f"{BOLD}  {prompt}{suffix}: {R}").strip()
    return val if val else str(default) if default is not None else ""

def ask_tile(prompt="타일 번호 (1~9, 엔터=전체)"):
    val = ask(prompt)
    if val == "":
        return list(range(1, TOTAL_TILES + 1))
    try:
        n = int(val)
        if 1 <= n <= TOTAL_TILES:
            return [n]
    except:
        pass
    c(RED, "  잘못된 입력 → 전체 실행")
    return list(range(1, TOTAL_TILES + 1))

def ask_int(prompt, lo, hi, default):
    val = ask(prompt, default)
    try:
        n = int(val)
        if lo <= n <= hi:
            return n
    except:
        pass
    return default

def delay(sec=0.3):
    time.sleep(sec)

def print_menu(design):
    print()
    c(BOLD + CYAN, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
    c(BOLD + CYAN, "   TILING MONITOR — 인터랙티브 테스트 노드")
    c(BOLD + CYAN, f"   현재 디자인: {design}")
    c(BOLD + CYAN, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
    c(YEL,  "  ─ 단계별 실행 ─────────────────────────────")
    c(YEL,  "  [1]  Step 1  접착제 파지")
    c(YEL,  "  [2]  Step 2  접착제 도포  → 웹: 9개 타일 회색")
    c(YEL,  "  [3]  Step 3  타일 파지    → 웹: 해당 타일 노란색")
    c(YEL,  "  [4]  Step 4  타일 배치    → 웹: 초록 마르는중 타이머 시작")
    c(YEL,  "  [5]  Step 5  압착 확인    → 웹: inspect_no/tile_level 전송")
    c(YEL,  "  [6]  Step 6  압착 중      → 웹: press_no 전송, 압착 완료")
    c(GRN,  "  ─ 전체/묶음 실행 ──────────────────────────")
    c(GRN,  "  [A]  전체 자동 실행 (Step1 → Step6)")
    c(GRN,  "  [P]  Step5+6 묶음  (압착 확인 → 필요 타일 압착)")
    c(BLUE, "  ─ 설정 / 기타 ───────────────────────────")
    c(BLUE, "  [D]  디자인 변경 (1/2/3)")
    c(BLUE, "  [S]  현재 Firebase 상태 조회")
    c(RED,  "  [E]  비상정지 시뮬레이션")
    c(GRAY, "  [R]  초기화")
    c(GRAY, "  [Q]  종료")
    c(BOLD + CYAN, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")

# ────────────────────────────────────────────────────────
# Step 함수
# ────────────────────────────────────────────────────────

def step1(sr, design):
    """
    접착제 파지
    웹: current_step=1 → 모든 타일 empty (running/finished 제외)
    """
    c(CYAN, "\n[STEP 1] 접착제 파지")
    sr.update({
        "current_step": 1,
        "state": "접착제 파지중",
        "joint_speed": round(random.uniform(1.2, 1.8), 3),
        "design": design,
        "completed_jobs": 0,
        "inspect_no": 0,
        "tile_level": 0.0,
        "press_no": 0,
    })
    c(GRN, "  → current_step=1  |  웹: 모든 타일 빈 상태")


def step2(sr):
    """
    접착제 도포
    웹: current_step=2 → 9개 타일 전부 회색(coated)
    """
    c(CYAN, "\n[STEP 2] 접착제 도포")
    sr.update({
        "current_step": 2,
        "state": "접착제 도포중",
        "joint_speed": round(random.uniform(3.5, 4.5), 3),
    })
    c(GRN, "  → current_step=2  |  웹: 9개 타일 회색(도포완료)")


def step3(sr, tiles):
    """
    타일 파지
    웹: current_step=3, completed_jobs=t-1 → workingTileIdx 번 타일 노란색(작업중)
        3D 색상도 이 시점에 패턴 색 적용
    """
    c(CYAN, f"\n[STEP 3] 타일 파지 — 대상: {tiles}")
    for t in tiles:
        sr.update({
            "current_step": 3,
            "state": f"타일 파지중 [{t}/{TOTAL_TILES}]",
            "joint_speed": round(random.uniform(1.2, 1.8), 3),
            "completed_jobs": t - 1,
        })
        c(YEL, f"  타일 {t} 파지중  (completed_jobs={t-1})")
        c(GRN, f"  → 웹: 타일 {t} 노란색 + 3D 패턴 색 적용")
        delay(0.3)


def step4(sr, tiles):
    """
    타일 배치
    웹: completed_jobs=t 증가 → startTileTimer(120s) 초록 마르는중 타이머
        3D 색상 패턴 적용
    """
    c(CYAN, f"\n[STEP 4] 타일 배치 — 대상: {tiles}")
    for t in tiles:
        sr.update({
            "current_step": 4,
            "state": f"타일 배치중 [{t}/{TOTAL_TILES}]",
            "joint_speed": round(random.uniform(2.0, 3.0), 3),
            "completed_jobs": t - 1,
        })
        c(YEL, f"  타일 {t} 배치중...")
        delay(0.2)
        sr.update({
            "completed_jobs": t,
            "state": f"타일 {t} 배치 완료",
            "joint_speed": 0,
        })
        c(GRN, f"  타일 {t} 배치 완료  (completed_jobs={t})")
        c(GRN, f"  → 웹: 초록 마르는중 타이머 시작 + 3D 패턴 색 적용")
        delay(0.1)


def step5_single(sr, tile_num, fixed_val=None):
    """
    타일 압착 확인 (1개)
    웹: inspect_no=타일번호, tile_level=0 → '압착 체크중'
        tile_level 값 전송 → 0<v<1.5 '배치완료', v>=1.5 '압착필요'
        해당 타일 마르는중 타이머 정지
    """
    # 체크중
    sr.update({
        "current_step": 5,
        "state": f"압착 확인중 [{tile_num}/{TOTAL_TILES}]",
        "inspect_no": tile_num,
        "tile_level": 0.0,
        "joint_speed": round(random.uniform(0.8, 1.2), 3),
    })
    c(YEL, f"  타일 {tile_num}  tile_level=0.0  →  웹: '압착 체크중' + 타이머 정지")
    delay(0.3)

    # 측정값 (5,7,8,9번 → 압착 필요 / 나머지 → 배치 완료)
    PRESS_NEEDED = {5, 7, 8, 9}
    if fixed_val is None:
        val = (round(random.uniform(1.6, 3.0), 3) if tile_num in PRESS_NEEDED
               else round(random.uniform(0.1, 1.4), 3))
    else:
        val = fixed_val

    sr.update({
        "tile_level": val,
        "state": f"압착 {'필요' if val >= 1.5 else '완료'} [{tile_num}/{TOTAL_TILES}]",
    })
    result = f"{RED}압착 필요{R}" if val >= 1.5 else f"{GRN}배치 완료{R}"
    print(f"  타일 {tile_num}  tile_level={val}  →  {result}")
    delay(0.3)
    return val


def step5(sr, tiles, fixed_val=None):
    c(CYAN, f"\n[STEP 5] 타일 압착 확인 — 대상: {tiles}")
    results = {}
    for t in tiles:
        results[t] = step5_single(sr, t, fixed_val)
    c(GRN, "  → Step 5 완료")
    return results


def step6_single(sr, tile_num):
    """
    타일 압착 중 (1개)
    웹: press_no=타일번호 → tileState='pressed', '압착 완료'
        9개 모두 pressed → '배치 끝!' 팝업
    press_no=0 이면 웹에서 스킵
    """
    sr.update({
        "current_step": 6,
        "state": f"압착 중 [{tile_num}/{TOTAL_TILES}]",
        "press_no": tile_num,
        "joint_speed": round(random.uniform(0.3, 0.7), 3),
    })
    c(YEL, f"  타일 {tile_num} 압착 중  (press_no={tile_num})")
    delay(0.3)
    c(GRN, f"  타일 {tile_num} 압착 완료  →  웹: '압착 완료' 표시")


def step6(sr, tiles):
    c(CYAN, f"\n[STEP 6] 타일 압착 중 — 대상: {tiles}")
    for t in tiles:
        step6_single(sr, t)
    if TOTAL_TILES in tiles:
        sr.update({
            "current_step": 0,
            "state": "모든 타일링 작업 완료",
            "joint_speed": 0,
            "press_no": 0,
        })
        c(BOLD + GRN, "\n🎉 9개 타일 모두 완료!  →  웹: '배치 끝!' 팝업")


def step5_6_together(sr, tiles, fixed_val=None):
    """Step5 압착 확인 후 필요한 타일만 Step6 압착"""
    c(MAG, f"\n[Step5+6] 압착 확인 → 압착 실행 — 대상: {tiles}")
    results = step5(sr, tiles, fixed_val)
    need = [t for t, v in results.items() if v >= 1.5]
    skip = [t for t, v in results.items() if v < 1.5]
    if skip:
        c(GRN, f"  배치 완료 (압착 불필요): {skip}")
    if need:
        c(YEL, f"  압착 필요 → Step 6 진행: {need}")
        step6(sr, need)
    c(GRN, "  → Step5+6 완료")


def run_all(sr, design):
    c(BOLD + GRN, f"\n[전체 실행] 디자인={design}  Step 1 → 6")
    step1(sr, design);  delay(0.3)
    step2(sr);          delay(0.3)
    for t in range(1, TOTAL_TILES + 1):
        step3(sr, [t])
        step4(sr, [t])
    results = step5(sr, list(range(1, TOTAL_TILES + 1)))
    need = [t for t, v in results.items() if v >= 1.5]
    if need:
        c(YEL, f"\n  압착 필요 타일 (5,7,8,9): {need}")
        step6(sr, need)
    sr.update({"current_step": 0, "state": "모든 타일링 작업 완료", "joint_speed": 0, "press_no": 0})
    c(BOLD + GRN, "\n🎉 전체 공정 완료!")


# ────────────────────────────────────────────────────────
# 비상정지 / 상태조회 / 초기화 / 디자인
# ────────────────────────────────────────────────────────

def estop(sr):
    c(RED, "\n[비상정지] 종류:")
    c(RED, "  [1] TCP 합력 초과  (collision_joint=0)")
    c(RED, "  [2] TCP Fz 초과   (collision_joint=-1)")
    c(RED, "  [3] 관절 토크 초과 (collision_joint=1~6)")
    val = ask("선택", 1)
    if val == "1":
        v = round(random.uniform(61, 90), 2)
        sr.update({"state": "충돌 감지 - 비상정지", "collision_joint": 0,  "collision_torque": v, "force_total": v})
        c(RED, f"  → TCP 합력 {v} N 초과")
    elif val == "2":
        v = round(random.uniform(31, 55), 2)
        sr.update({"state": "충돌 감지 - 비상정지", "collision_joint": -1, "collision_torque": v, "force_z": v})
        c(RED, f"  → TCP Fz {v} N 초과")
    elif val == "3":
        j = ask_int("관절 번호 (1~6)", 1, 6, 1)
        v = round(random.uniform(61, 100), 2)
        sr.update({"state": "충돌 감지 - 비상정지", "collision_joint": j,  "collision_torque": v})
        c(RED, f"  → J{j} 관절 토크 {v} Nm 초과")


def show_status(sr):
    data = sr.get()
    c(BOLD + BLUE, "\n[Firebase robot_status]")
    if data:
        for k in ["current_step","state","completed_jobs","design",
                  "inspect_no","tile_level","press_no",
                  "joint_speed","force_z","force_total"]:
            print(f"  {BLUE}{k:<20}{R} {data.get(k, '-')}")
    else:
        c(GRAY, "  데이터 없음")


def do_reset(sr, cr):
    c(GRAY, "\n[초기화]")
    sr.set({
        "current_step": 0, "state": "대기",
        "pos_x": 0.0, "pos_y": 0.0, "pos_z": 0.0,
        "completed_jobs": 0, "working_tile": 0,
        "joint_speed": 0, "design": 0, "design_ab": "",
        "inspect_no": 0, "tile_level": 0.0, "press_no": 0,
        "force_z": 0.0, "force_total": 0.0,
        "tool_force": {"0":0.0,"1":0.0,"2":0.0,"3":0.0,"4":0.0,"5":0.0},
        "ext_torque": {"0":0.0,"1":0.0,"2":0.0,"3":0.0,"4":0.0,"5":0.0},
    })
    cr.update({"action": "idle"})
    c(GRN, "  → 초기화 완료")


def select_design():
    c(BLUE, "\n  [1] 지그재그① WBWB 체커보드")
    c(BLUE, "  [2] 지그재그② 줄무늬")
    c(BLUE, "  [3] 데코타일  포인트 패턴")
    val = ask("디자인 선택 (1~3)", 1)
    try:
        n = int(val)
        if 1 <= n <= 3:
            return n
    except:
        pass
    return 1


# ────────────────────────────────────────────────────────
# 메인 루프
# ────────────────────────────────────────────────────────

def main():
    sr, cr = init_firebase()
    c(BOLD + GRN, "\nFirebase 연결 성공!")
    design = 1

    while True:
        print_menu(design)
        choice = ask("선택").upper()

        if choice == "1":
            step1(sr, design)

        elif choice == "2":
            step2(sr)

        elif choice == "3":
            tiles = ask_tile()
            step3(sr, tiles)

        elif choice == "4":
            tiles = ask_tile()
            step4(sr, tiles)

        elif choice == "5":
            tiles = ask_tile()
            c(YEL, "  tile_level 값:")
            c(YEL, "  [1] 고정 (5,7,8,9=압착필요 / 1,2,3,4,6=배치완료)")
            c(YEL, "  [2] 전부 압착 필요 (>=1.5)")
            c(YEL, "  [3] 전부 배치 완료 (<1.5)")
            c(YEL, "  [4] 직접 입력")
            v = ask("선택", 1)
            if v == "2":
                step5(sr, tiles, round(random.uniform(1.6, 3.0), 3))
            elif v == "3":
                step5(sr, tiles, round(random.uniform(0.1, 1.4), 3))
            elif v == "4":
                try:    manual = float(ask("tile_level 값", 0.5))
                except: manual = 0.5
                step5(sr, tiles, manual)
            else:
                step5(sr, tiles)

        elif choice == "6":
            c(YEL, "  press_no 전송:")
            c(YEL, "  [1] 타일 선택")
            c(YEL, "  [2] 0 전송 (스킵)")
            v = ask("선택", 1)
            if v == "2":
                sr.update({"current_step": 6, "press_no": 0, "state": "압착 스킵"})
                c(GRAY, "  → press_no=0 (스킵)")
            else:
                tiles = ask_tile()
                step6(sr, tiles)

        elif choice == "A":
            design = select_design()
            run_all(sr, design)

        elif choice == "P":
            tiles = ask_tile("Step5+6 대상 타일")
            c(YEL, "  tile_level 값:")
            c(YEL, "  [1] 랜덤  [2] 전부 압착필요  [3] 전부 배치완료")
            v = ask("선택", 1)
            if v == "2":
                step5_6_together(sr, tiles, round(random.uniform(1.6, 3.0), 3))
            elif v == "3":
                step5_6_together(sr, tiles, round(random.uniform(0.1, 1.4), 3))
            else:
                step5_6_together(sr, tiles)

        elif choice == "D":
            design = select_design()
            sr.update({"design": design})
            c(GRN, f"  → 디자인 {design} 설정")

        elif choice == "S":
            show_status(sr)

        elif choice == "E":
            estop(sr)

        elif choice == "R":
            do_reset(sr, cr)
            design = 1

        elif choice == "Q":
            c(GRAY, "\n종료합니다.")
            break
        else:
            c(RED, "  잘못된 입력")


if __name__ == "__main__":
    main()