#!/usr/bin/env python3
"""
fake_simulation_node.py
-----------------------
Firebase robot_command/action 감지 → 시뮬레이션 실행

동작:
  - Firebase에서 action=start 감지 시 시뮬레이션 시작
  - 0.2초마다 current_step 1→2→3→4→1→2... 순환
  - step이 4(타일 배치중)를 지나면 completed_jobs +1
  - 총 9개 타일 완료 시 종료

설치:
    pip install firebase-admin
"""

import time
import firebase_admin
from firebase_admin import credentials, db

FIREBASE_CREDENTIAL_PATH = "/home/sa/cobot_ws/src/cobot1/config/co1-tiling-firebase-adminsdk-fbsvc-f4f88c3832.json"
FIREBASE_DB_URL = "https://co1-tiling-default-rtdb.asia-southeast1.firebasedatabase.app"

TOTAL_TILES = 9
STEP_INTERVAL = 0.2   # 스텝 변화 간격 (초)
STEPS_PER_TILE = 4    # 타일 1개당 step 사이클 수 (1~4 두 바퀴)


def wait_for_start(cmd_ref):
    print("Firebase action=start 대기 중... (웹에서 시작 버튼을 누르세요)")
    while True:
        action = cmd_ref.get()
        if action == "start":
            print(f"[START] action=start 감지!")
            return
        time.sleep(0.3)


def run_simulation(status_ref):
    step_sequence = [1, 2, 3, 4]  # 접착제파지 → 도포 → 타일파지 → 배치
    step_idx = 0
    completed_jobs = 0
    cycle_count = 0  # 현재 타일에서 step 사이클 수

    print(f"[SIM] 시뮬레이션 시작 (총 {TOTAL_TILES}개 타일)")
    status_ref.update({"completed_jobs": 0, "current_step": 1, "state": "접착제 파지중"})

    state_labels = {
        1: "접착제 파지중",
        2: "접착제 도포중",
        3: "타일 파지중",
        4: "타일 배치중",
    }

    while completed_jobs < TOTAL_TILES:
        # stop_soft 체크
        current_step = step_sequence[step_idx % len(step_sequence)]
        label = state_labels[current_step]

        status_ref.update({
            "current_step": current_step,
            "state": f"{label} [{completed_jobs + 1}/{TOTAL_TILES}]"
        })
        print(f"[STEP] step={current_step} ({label}) | 타일 {completed_jobs + 1}/{TOTAL_TILES}")

        time.sleep(STEP_INTERVAL)

        step_idx += 1
        cycle_count += 1

        # STEPS_PER_TILE 사이클마다 타일 1개 완료
        if cycle_count >= STEPS_PER_TILE:
            cycle_count = 0
            completed_jobs += 1
            status_ref.update({
                "completed_jobs": completed_jobs,
                "current_step": 4,
                "state": f"타일 배치 완료 [{completed_jobs}/{TOTAL_TILES}]"
            })
            print(f"[DONE] completed_jobs={completed_jobs}")
            time.sleep(0.3)

    # 전체 완료
    status_ref.update({
        "current_step": 0,
        "state": "전체 완료"
    })
    print(f"[FINISH] 전체 {TOTAL_TILES}개 타일 완료!")


def main():
    cred = credentials.Certificate(FIREBASE_CREDENTIAL_PATH)
    firebase_admin.initialize_app(cred, {"databaseURL": FIREBASE_DB_URL})

    status_ref = db.reference("robot_status")
    cmd_ref    = db.reference("robot_command/action")

    print("=" * 40)
    print("Fake Simulation Node")
    print("웹에서 패턴 선택 후 시작 버튼을 누르세요")
    print("=" * 40)

    while True:
        # start 대기
        wait_for_start(cmd_ref)

        # 시뮬레이션 실행
        run_simulation(status_ref)

        # 다음 라운드 대기
        print("\n[WAIT] 다시 시작하려면 웹에서 초기화 후 시작 버튼을 누르세요\n")
        # action을 idle로 바꿔서 재시작 감지 가능하게
        cmd_ref.set("idle")


if __name__ == "__main__":
    main()