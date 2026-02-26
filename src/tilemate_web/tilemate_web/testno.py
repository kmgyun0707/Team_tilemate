import time
import firebase_admin
from firebase_admin import credentials, db
import os

# Firebase 설정 정보 (제공해주신 경로 및 URL 기반)
SERVICE_ACCOUNT_KEY_PATH = os.path.expanduser(
    "~/Team_tilemate/src/tilemate_web/config/co1-tiling-firebase-adminsdk-fbsvc-f4f88c3832.json"
)
FIREBASE_DB_URL = "https://co1-tiling-default-rtdb.asia-southeast1.firebasedatabase.app"

TOTAL_TILES = 9
STEP_INTERVAL = 0.2  # 각 스텝 사이의 시간 간격 (0.2초)

# 사용자가 요청한 Step 2(접착제 도포) 시의 타일 위치 순서
GLUE_SEQUENCE = [5, 2, 8, 4, 1, 7, 6, 3, 9]

def wait_for_start(cmd_ref):
    """Firebase에서 '시작(start)' 명령이 올 때까지 대기합니다."""
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
    """
    Step 1, 2를 수행한 뒤 Step 3, 4를 9번 반복하는 시뮬레이션 로직입니다.
    웹 UI의 시각화 로직에 맞춰 1-2-3-4 사이클을 9회 반복하며 데이터를 업데이트합니다.
    """
    print(f"[SIM] 시뮬레이션 시작 (총 {TOTAL_TILES}개 타일 작업)")
    
    # 로봇 상태 초기화
    status_ref.update({
        "completed_jobs": 0,
        "current_step": 0,
        "state": "준비 중",
        "design": design,
        "joint_speed": 0
    })

    # 9개의 타일에 대해 공정 반복
    for i in range(TOTAL_TILES):
        current_job_idx = i  # 0 ~ 8
        work_tile_num = i + 1  # 1 ~ 9 (Step 3, 4가 표시될 타일 번호)
        glue_tile_num = GLUE_SEQUENCE[i]  # Step 2에서 회색으로 표시될 타일 번호

        # --- Step 1 & 2 (접착제 단계) ---
        # Step 1: 접착제 파지 (UI에는 아무런 타일 변화 없음)
        status_ref.update({
            "current_step": 1,
            "state": f"접착제 파지중 [{work_tile_num}/{TOTAL_TILES}]",
            "joint_speed": 1.5
        })
        print(f"[STEP 1] 타일 {work_tile_num} 공정 - 접착제 도구 준비")
        time.sleep(STEP_INTERVAL)

        # Step 2: 접착제 도포 (UI에서 지정된 순서대로 회색 '도포중' 표시)
        status_ref.update({
            "current_step": 2,
            "state": f"접착제 도포중 (타일 {glue_tile_num} 위치)",
            "joint_speed": 4.0
        })
        print(f"[STEP 2] 타일 {work_tile_num} 공정 - {glue_tile_num}번 위치 도포")
        time.sleep(STEP_INTERVAL)

        # --- Step 3 & 4 (타일 단계) ---
        # Step 3: 타일 파지 (UI에서 해당 타일 번호가 노란색 '작업중' 표시)
        status_ref.update({
            "current_step": 3,
            "state": f"타일 파지중 [{work_tile_num}/{TOTAL_TILES}]",
            "joint_speed": 1.5
        })
        print(f"[STEP 3] 타일 {work_tile_num} 공정 - 타일 흡착")
        time.sleep(STEP_INTERVAL)

        # Step 4: 타일 배치 (UI에서 해당 타일 번호가 노란색 유지)
        status_ref.update({
            "current_step": 4,
            "state": f"타일 배치중 [{work_tile_num}/{TOTAL_TILES}]",
            "joint_speed": 2.5
        })
        print(f"[STEP 4] 타일 {work_tile_num} 공정 - 배치 위치 {work_tile_num}")
        time.sleep(STEP_INTERVAL)

        # 타일 1개 최종 완료 (Step 4 종료 후 completed_jobs를 올리면 UI에서 초록색 '마르는중'으로 변경됨)
        status_ref.update({
            "completed_jobs": work_tile_num,
            "state": f"타일 {work_tile_num} 완료",
            "joint_speed": 0
        })
        print(f"[DONE] 타일 {work_tile_num} 작업 완료 (전체 {work_tile_num}/{TOTAL_TILES})")
        time.sleep(0.1)

    # 모든 타일 공정 종료
    status_ref.update({
        "current_step": 0,
        "state": "모든 타일링 작업 완료",
        "joint_speed": 0
    })
    print("[FINISH] 모든 시뮬레이션 공정이 성공적으로 끝났습니다.")

def main():
    # Firebase 키 파일 존재 여부 확인
    if not os.path.exists(SERVICE_ACCOUNT_KEY_PATH):
        print(f"오류: Firebase 키 파일을 찾을 수 없습니다: {SERVICE_ACCOUNT_KEY_PATH}")
        return

    # Firebase 관리자 SDK 초기화 (제공된 경로 사용)
    cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
    firebase_admin.initialize_app(cred, {"databaseURL": FIREBASE_DB_URL})

    status_ref = db.reference("robot_status")
    cmd_ref = db.reference("robot_command")

    print("-" * 50)
    print("Tile Monitoring Test Node (v2.1)")
    print("-" * 50)

    try:
        while True:
            # 웹의 시작 신호 대기
            design = wait_for_start(cmd_ref)
            
            # 시뮬레이션 루프 실행
            run_simulation(status_ref, design)

            # 실행 완료 후 다음 시작을 위해 action 초기화
            print("\n[WAIT] 다시 시작하려면 웹에서 '초기화' 후 '시작' 버튼을 누르세요.")
            cmd_ref.update({"action": "idle"})
            
    except KeyboardInterrupt:
        print("\n시뮬레이션 노드를 종료합니다.")

if __name__ == "__main__":
    main()
