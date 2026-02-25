import time
import firebase_admin
from firebase_admin import credentials, db
import os

# ── Firebase 설정 ──────────────────────────────────────────
SERVICE_ACCOUNT_KEY_PATH = os.path.expanduser(
    "~/Team_tilemate/src/tilemate_web/config/co1-tiling-firebase-adminsdk-fbsvc-f4f88c3832.json"
)
FIREBASE_DB_URL = "https://co1-tiling-default-rtdb.asia-southeast1.firebasedatabase.app"

TOTAL_TILES  = 9
STEP_INTERVAL = 0.2   # 각 스텝 사이 간격 (초)

# Step 2 접착제 도포 순서
GLUE_SEQUENCE = [5, 2, 8, 4, 1, 7, 6, 3, 9]


def wait_for_start(cmd_ref):
    print("=" * 50)
    print("Firebase 'action=start' 대기 중... (웹에서 시작 버튼을 누르세요)")
    while True:
        cmd = cmd_ref.get()
        if isinstance(cmd, dict) and cmd.get("action") == "start":
            design = int(cmd.get("design", 1))
            print(f"[START] 시작 명령 감지! (선택된 디자인: {design})")
            return design
        time.sleep(0.5)


def run_simulation(status_ref, design):
    print(f"[SIM] 시뮬레이션 시작 (총 {TOTAL_TILES}개 타일, 디자인: {design})")

    # 초기화
    status_ref.update({
        "completed_jobs": 0,
        "current_step":   0,
        "state":          "준비 중",
        "design":         design,
        "joint_speed":    0,
        "press_tile":     0,
    })

    # ── Step 1 → 2 → (3 → 4) × 9 ─────────────────────────

    for i in range(TOTAL_TILES):
        work_tile = i + 1           # 1~9
        glue_tile = GLUE_SEQUENCE[i]

        # Step 1: 접착제 파지
        status_ref.update({
            "current_step": 1,
            "state":        f"접착제 파지중 [{work_tile}/{TOTAL_TILES}]",
            "joint_speed":  1.5,
        })
        print(f"[STEP 1] 타일 {work_tile} - 접착제 파지")
        time.sleep(STEP_INTERVAL)

        # Step 2: 접착제 도포
        status_ref.update({
            "current_step": 2,
            "state":        f"접착제 도포중 (타일 {glue_tile} 위치)",
            "joint_speed":  4.0,
        })
        print(f"[STEP 2] 타일 {work_tile} - {glue_tile}번 위치 도포")
        time.sleep(STEP_INTERVAL)

        # Step 3: 타일 파지
        status_ref.update({
            "current_step": 3,
            "state":        f"타일 파지중 [{work_tile}/{TOTAL_TILES}]",
            "joint_speed":  1.5,
        })
        print(f"[STEP 3] 타일 {work_tile} - 타일 흡착")
        time.sleep(STEP_INTERVAL)

        # Step 4: 타일 배치
        status_ref.update({
            "current_step": 4,
            "state":        f"타일 배치중 [{work_tile}/{TOTAL_TILES}]",
            "joint_speed":  2.5,
        })
        print(f"[STEP 4] 타일 {work_tile} - 배치")
        time.sleep(STEP_INTERVAL)

        # 타일 1개 완료
        status_ref.update({
            "completed_jobs": work_tile,
            "state":          f"타일 {work_tile} 완료",
            "joint_speed":    0,
        })
        print(f"[DONE]  타일 {work_tile} 완료 ({work_tile}/{TOTAL_TILES})")
        time.sleep(0.1)

    # ── Step 5: 타일 압착 (9개 전체 순서대로) ────────────
    print("\n[STEP 5] 전체 타일 압착 시작")
    status_ref.update({
        "current_step": 5,
        "state":        "타일 압착 중",
        "joint_speed":  1.0,
    })

    for tile_num in range(1, TOTAL_TILES + 1):
        status_ref.update({
            "press_tile": tile_num,
            "state":      f"타일 {tile_num} 압착 중",
        })
        print(f"[STEP 5] 타일 {tile_num} 압착")
        time.sleep(STEP_INTERVAL)

    # 완료
    status_ref.update({
        "current_step": 0,
        "state":        "모든 타일링 작업 완료",
        "joint_speed":  0,
        "press_tile":   0,
    })
    print("[FINISH] 시뮬레이션 완료!")


def main():
    if not os.path.exists(SERVICE_ACCOUNT_KEY_PATH):
        print(f"오류: Firebase 키 파일 없음: {SERVICE_ACCOUNT_KEY_PATH}")
        return

    cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
    firebase_admin.initialize_app(cred, {"databaseURL": FIREBASE_DB_URL})

    status_ref = db.reference("robot_status")
    cmd_ref    = db.reference("robot_command")

    print("-" * 50)
    print("Fake Node (step 1→2→(3→4)×9→5)")
    print("-" * 50)

    try:
        while True:
            design = wait_for_start(cmd_ref)
            run_simulation(status_ref, design)

            print("\n[WAIT] 다시 하려면 웹에서 초기화 → 시작 누르세요.")
            cmd_ref.update({"action": "idle"})

    except KeyboardInterrupt:
        print("\n종료합니다.")


if __name__ == "__main__":
    main()