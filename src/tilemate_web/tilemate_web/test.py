#!/usr/bin/env python3
"""
test.py
─────────────────────────────────────────────────────────────
FastAPI 서버 + Firebase 통합 테스트 스크립트

- Firebase robot_status 업데이트 → index.html 2D 그리드 / 공정 단계 반영
- FastAPI 엔드포인트 호출 → SSE inspect_updated → 보고서 모달 반영

사용법:
    # 터미널 1: 서버 실행
    cd /home/sa/Team_tilemate/src/tilemate_web/tilemate_web
    python server.py

    # 터미널 2: 이 스크립트 실행
    python test.py
"""

import json
import math
import os
import random
import sys
import time

try:
    import requests
except ImportError:
    print("[ERROR] pip install requests")
    sys.exit(1)

try:
    import firebase_admin
    from firebase_admin import credentials, db as fb_db
    _FB_AVAILABLE = True
except ImportError:
    _FB_AVAILABLE = False
    print("[WARNING] firebase_admin 없음. Firebase 기능 비활성화.")
    print("          pip install firebase-admin")

# ── 설정 ──────────────────────────────────────────────────
#BASE_URL = "http://127.0.0.1:8000"
BASE_URL = "http://192.168.10.48:8000"

SERVICE_ACCOUNT_KEY_PATH = os.path.expanduser(
    "~/Team_tilemate/src/tilemate_web/config/co1-tiling-firebase-adminsdk-fbsvc-f4f88c3832.json"
)
DATABASE_URL = "https://co1-tiling-default-rtdb.asia-southeast1.firebasedatabase.app"

INSPECTION_JSON_PATH = os.path.expanduser(
    "~/Team_tilemate/src/tilemate_web/config/dummy.json"
)

DESIGN_PATTERNS = {
    1: ("B,A,B,A,B,A,B,A,B", "체커보드"),
    2: ("B,B,B,A,A,A,B,B,B", "줄무늬"),
    3: ("B,A,B,A,A,A,B,A,B", "데코"),
}

TILE_STEP_LABELS = {
    0: "대기",
    1: "타일 파지 중",
    2: "시멘트 도포 중",
    3: "타일 부착 중",
    4: "단차 검수 중",
    5: "압착 보정 중",
    6: "타일 완료",
}

# ── Firebase 초기화 ────────────────────────────────────────
_fb_ref = None

def init_firebase():
    global _fb_ref
    if not _FB_AVAILABLE:
        return False
    if not os.path.exists(SERVICE_ACCOUNT_KEY_PATH):
        print(f"[WARNING] Firebase 키 없음: {SERVICE_ACCOUNT_KEY_PATH}")
        return False
    try:
        if not firebase_admin._apps:
            cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
            firebase_admin.initialize_app(cred, {"databaseURL": DATABASE_URL})
        _fb_ref = fb_db.reference("/robot_status")
        ok("Firebase 연결 완료")
        return True
    except Exception as e:
        print(f"[WARNING] Firebase 초기화 실패: {e}")
        return False

def fb_update(data: dict):
    """Firebase robot_status 업데이트. 실패해도 무시."""
    if _fb_ref is None:
        return
    try:
        _fb_ref.update(data)
    except Exception as e:
        print(f"  [FB] 업데이트 실패 (무시): {e}")

def fb_reset():
    """Firebase robot_status 초기화."""
    if _fb_ref is None:
        print("  Firebase 연결 없음")
        return
    try:
        _fb_ref.set({
            "current_step": 0,
            "state": "대기",
            "completed_jobs": 0,
            "working_tile": 0,
            "tile_type": 0,
            "tile_step": 0,
            "overall_progress": 0.0,
            "tile_progress": 0.0,
            "design": 0,
            "design_ab": "",
            "token": "",
            "is_resume": False,
            "joint_speed": 0.0,
            "tile_level": 0.0,
            "inspect_no": 0,
            "press_no": 0,
            "message": "",
            "inspection_result": None,
        })
        ok("Firebase 초기화 완료")
    except Exception as e:
        print(f"  [FB] 초기화 실패: {e}")

# ── 출력 헬퍼 ─────────────────────────────────────────────
def ok(msg):   print(f"  ✅ {msg}")
def err(msg):  print(f"  ❌ {msg}")
def info(msg): print(f"  ℹ️  {msg}")

# ── FastAPI 헬퍼 ───────────────────────────────────────────
def check_server():
    try:
        r = requests.get(BASE_URL, timeout=3)
        ok(f"서버 응답 확인 (status={r.status_code})")
        return True
    except Exception as e:
        err(f"서버 연결 실패: {e}")
        info("서버를 먼저 실행하세요: python server.py")
        return False

def get_latest():
    try:
        r = requests.get(f"{BASE_URL}/api/inspect/latest", timeout=5)
        data = r.json()
        tiles = data.get("tiles", [])
        ts = data.get("timestamp_sec", 0)
        ts_str = time.strftime('%H:%M:%S', time.localtime(ts)) if ts else "-"
        info(f"최신 결과: tiles={len(tiles)}개, timestamp={ts_str}")
        return data
    except Exception as e:
        err(f"latest 조회 실패: {e}")
        return None

def load_dummy():
    print("\n  ▶ dummy.json 로드 중...")
    try:
        r = requests.post(f"{BASE_URL}/api/inspect/dummy", timeout=5)
        data = r.json()
        if r.status_code == 200 and data.get("success"):
            ok("더미 로드 완료")
            return True
        else:
            err(f"더미 로드 실패: {data.get('message', r.status_code)}")
            return False
    except Exception as e:
        err(f"요청 실패: {e}")
        return False

def post_result(payload: dict):
    try:
        r = requests.post(
            f"{BASE_URL}/api/inspect/result",
            json=payload,
            headers={"Content-Type": "application/json"},
            timeout=5,
        )
        data = r.json()
        if r.status_code == 200 and data.get("success"):
            ok("inspection result 전송 완료")
            fb_update({"inspection_result": payload})
            return True
        else:
            err(f"전송 실패: {data}")
            return False
    except Exception as e:
        err(f"요청 실패: {e}")
        return False

def make_sample_payload(tile_count: int = 3) -> dict:
    tiles = []
    for i in range(tile_count):
        angle_r = random.uniform(-5.0, 5.0)
        angle_p = random.uniform(-5.0, 5.0)
        angle_y = random.uniform(-2.0, 2.0)
        dist_mm = random.uniform(-8.0, 8.0)
        u = 200 + i * 120 + random.uniform(-10, 10)
        v = 200 + random.uniform(-10, 10)
        tiles.append({
            "name": f"pattern_{i+1}",
            "conf_score": round(random.uniform(0.97, 0.9999), 6),
            "center_uv": [round(u, 2), round(v, 2)],
            "size_uv": [96.0, 96.0],
            "rpy_deg": [round(angle_r, 4), round(angle_p, 4), round(angle_y, 4)],
            # UI 판정에 필수적인 변수들 추가
            "roll_deg": round(angle_r, 4),    
            "pitch_deg": round(angle_p, 4),   
            "yaw_deg": round(angle_y, 4),     
            "rmse_mm": round(random.uniform(0.5, 4.0), 2), 
            "roi": [100, 100, 200, 200],      
            "normal": [                       
                round(math.sin(math.radians(angle_p)), 6),
                round(-math.sin(math.radians(angle_r)), 6),
                round(math.cos(math.radians(angle_r)) * math.cos(math.radians(angle_p)), 6),
            ],
            "plane_normal": [
                round(math.sin(math.radians(angle_p)), 6),
                round(-math.sin(math.radians(angle_r)), 6),
                round(math.cos(math.radians(angle_r)) * math.cos(math.radians(angle_p)), 6),
            ],
            "plane_d": round(-380.0 + dist_mm, 4),
            "plane_centroid_mm": [
                round(-60.0 + i * 80.0, 4),
                round(-50.0, 4),
                round(382.0 + dist_mm, 4),
            ],
        })

    # Wall 데이터 추가
    wall = {
        "roi": [0, 0, 640, 480],
        "rmse_mm": round(random.uniform(0.5, 1.5), 2),
        "roll_deg": 0.1,
        "pitch_deg": 0.1,
        "yaw_deg": 0.0,
        "normal": [0.0, 0.0, 1.0],
    }

    return {
        "success": True,
        "message": "inspect_complete",
        "frame_id": "camera_color_optical_frame",
        "timestamp_sec": time.time(),
        "wall": wall,     # 추가된 부분
        "tiles": tiles,
    }

# ── 시나리오 헬퍼 ─────────────────────────────────────────
def set_tile_step(step: int, tile_index: int = 0, design: int = 1):
    """Firebase tile_step + 공정 단계 업데이트 → index.html 반영."""
    pattern_str, _ = DESIGN_PATTERNS.get(design, DESIGN_PATTERNS[1])
    label = TILE_STEP_LABELS.get(step, "작업 중")
    fb_update({
        "tile_step":        step,
        "current_step":     1,
        "state":            label,
        "working_tile":     tile_index,
        "design":           design,
        "design_ab":        pattern_str,
        "overall_progress": round(step / 6.0, 3),
        "tile_progress":    0.0,
    })
    print(f"  → tile_step={step}  ({label})  tile={tile_index}  design={design}")
    time.sleep(0.4)

def select_design() -> int:
    print("\n  디자인 선택:")
    for k, (_, name) in DESIGN_PATTERNS.items():
        print(f"    {k}) {name}")
    try:
        return int(input("  번호: ").strip())
    except Exception:
        return 1

def run_full_scenario(design: int = 1, tile_count: int = 9):
    """
    실제 시나리오: PICK → COWORK → PLACE x tile_count 반복 후
    한번에 단차검수(INSPECT) → 압착보정(COMPACT).
    테스트용으로 cycle_count(기본 2)만큼만 돌고 검수.
    """
    pattern_str, pattern_name = DESIGN_PATTERNS.get(design, DESIGN_PATTERNS[1])
    print(f"\n[전체 시나리오] design={design} ({pattern_name}), 전체 타일={tile_count}개")
    print("웹 브라우저에서 http://192.168.10.48:8000 열어두세요!\n")

    try:
        cycle_count = int(input(f"  테스트 사이클 수 (기본 2, 최대 {tile_count}): ").strip() or "2")
        cycle_count = max(1, min(cycle_count, tile_count))
    except ValueError:
        cycle_count = 2
    print(f"  → {cycle_count}사이클 후 단차검수 진행\n")

    fb_reset()
    time.sleep(0.5)

    # ── 1. 타일 파지→시멘트→부착 반복 ──
    for tile_idx in range(cycle_count):
        print(f"  ═══ 타일 {tile_idx + 1}/{cycle_count} (파지→시멘트→부착) ═══")
        for step in [1, 2, 3]:
            set_tile_step(step, tile_index=tile_idx, design=design)
            time.sleep(0.8)
        # 부착 완료
        set_tile_step(6, tile_index=tile_idx, design=design)
        fb_update({"completed_jobs": tile_idx + 1, "tile_step": 0})
        time.sleep(0.3)

    # ── 2. 단차검수 (전체 한번에) ──
    print(f"\n  ▶ INSPECT (tile_step=4) → {cycle_count}개 타일 단차검수 시작!")
    fb_update({"inspection_result": None})
    set_tile_step(4, tile_index=cycle_count - 1, design=design)
    fb_update({"completed_jobs": cycle_count})
    time.sleep(0.8)

    dummy_path = os.path.expanduser(INSPECTION_JSON_PATH)
    if os.path.exists(dummy_path):
        with open(dummy_path, "r", encoding="utf-8") as f:
            payload = json.load(f)
        payload["timestamp_sec"] = time.time()
        # tiles name을 pattern_1 ~ pattern_N 으로 맞춤
        for i, t in enumerate(payload.get("tiles", [])):
            t["name"] = f"pattern_{i + 1}"
        info(f"dummy.json 로드 ({len(payload.get('tiles', []))}개 타일)")
    else:
        payload = make_sample_payload(tile_count=cycle_count)
        for i, t in enumerate(payload["tiles"]):
            t["name"] = f"pattern_{i + 1}"
        info("dummy.json 없음 → 랜덤 샘플 사용")
    post_result(payload)
    time.sleep(0.5)

    input(f"\n  단차검수 확인 후 엔터 (압착보정으로 진행)...")

    # ── 3. 압착보정 ──
    print(f"\n  ▶ COMPACT (tile_step=5) → 압착 보정")
    set_tile_step(5, tile_index=cycle_count - 1, design=design)
    time.sleep(1.0)

    print(f"\n  ✅ 시나리오 완료 ({cycle_count}개 타일)")
    fb_update({"state": "작업 완료", "overall_progress": 1.0})

# ── 메인 메뉴 ─────────────────────────────────────────────
MENU = """
─── 테스트 메뉴 ───────────────────────────────────────────
  [FastAPI - 단차 보고서 모달]
  1) dummy.json 로드              (POST /api/inspect/dummy)
  2) 랜덤 3타일 샘플 전송         (POST /api/inspect/result)
  3) config/dummy.json 직접 전송  (POST /api/inspect/result)
  4) 최신 결과 확인               (GET  /api/inspect/latest)

  [Firebase - index.html 2D 그리드 / 공정 단계]
  f1) tile_step 수동 설정  (파지/시멘트/부착/검수/압착)
  f2) 디자인 패턴 설정     (흰색/검정 2D 그리드)
  f3) Firebase 초기화

  [시나리오]
  a)  자동 전체 시나리오  (PICK→COWORK→PLACE→INSPECT→COMPACT)

  q)  종료
───────────────────────────────────────────────────────────"""


def main():
    # main() 시작부에 추가
    has_firebase = init_firebase()
    if not has_firebase:
        print("[ERROR] Firebase 연결 필수! 키 파일 경로 확인:")
        print(f"  {SERVICE_ACCOUNT_KEY_PATH}")
        sys.exit(1)  # Firebase 없으면 아예 종료
    print("\n" + "=" * 57)
    print("  단차 검수 통합 테스트 스크립트")
    print(f"  서버: {BASE_URL}")
    print("=" * 57)

    has_firebase = init_firebase()
    if not has_firebase:
        print("  ⚠️  Firebase 없이 FastAPI 기능만 사용 가능합니다.")

    if not check_server():
        sys.exit(1)

    while True:
        print(MENU)
        choice = input("선택: ").strip().lower()

        if choice == "1":
            load_dummy()
            time.sleep(0.3)
            get_latest()

        elif choice == "2":
            payload = make_sample_payload(tile_count=3)
            info("샘플 타일 3개 생성")
            post_result(payload)
            time.sleep(0.3)
            get_latest()

        elif choice == "3":
            path = os.path.expanduser(INSPECTION_JSON_PATH)
            if not os.path.exists(path):
                err(f"파일 없음: {path}")
            else:
                with open(path, "r", encoding="utf-8") as f:
                    payload = json.load(f)
                post_result(payload)
                time.sleep(0.3)
                get_latest()

        elif choice == "4":
            get_latest()

        elif choice == "f1":
            print("  0=대기 1=파지 2=시멘트 3=부착 4=검수 5=압착 6=완료")
            try:
                step   = int(input("  step 번호: ").strip())
                tile   = int(input("  tile_index (0~8, 기본 0): ").strip() or "0")
                design = select_design()
                set_tile_step(step, tile_index=tile, design=design)
                if step == 4:
                    ans = input("  inspection result도 전송할까요? (엔터=예/n=아니오): ").strip().lower()
                    if ans != "n":
                        payload = make_sample_payload(tile_count=2)
                        post_result(payload)
            except ValueError:
                err("숫자를 입력하세요.")

        elif choice == "f2":
            design = select_design()
            pattern_str, name = DESIGN_PATTERNS.get(design, DESIGN_PATTERNS[1])
            fb_update({"design": design, "design_ab": pattern_str})
            ok(f"디자인 설정: {design} ({name}) → {pattern_str}")

        elif choice == "f3":
            fb_reset()

        elif choice == "a":
            design = select_design()
            try:
                count = int(input("  타일 개수 (기본 3): ").strip() or "3")
            except ValueError:
                count = 3
            run_full_scenario(design=design, tile_count=count)

        elif choice == "q":
            print("\n[종료]")
            break

        else:
            print("  잘못된 입력입니다.")


if __name__ == "__main__":
    main()