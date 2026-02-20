#!/usr/bin/env python3
"""
fake_tile_color_node.py
-----------------------
테스트용 페이크 노드.
커맨드라인에서 직접 입력해서 Firebase에 red_tile / black_tile 발행.

사용법:
    python3 fake_tile_color_node.py

명령어 형식:
    red 3       -> 3번 타일 빨간색
    black 5     -> 5번 타일 검정색
    reset       -> red_tile, black_tile 초기화
    q / quit    -> 종료

설치:
    pip install firebase-admin
"""

import firebase_admin
from firebase_admin import credentials, db

FIREBASE_CREDENTIAL_PATH = "/home/sa/cobot_ws/src/cobot1/config/co1-tiling-firebase-adminsdk-fbsvc-f4f88c3832.json"
FIREBASE_DB_URL = "https://co1-tiling-default-rtdb.asia-southeast1.firebasedatabase.app"


def main():
    cred = credentials.Certificate(FIREBASE_CREDENTIAL_PATH)
    firebase_admin.initialize_app(cred, {"databaseURL": FIREBASE_DB_URL})
    ref = db.reference("robot_status")

    print("=" * 40)
    print("Fake Tile Color Node")
    print("명령어: red <번호> / black <번호> / reset / q")
    print("예시:   red 3   ->  3번 타일 빨간색")
    print("=" * 40)

    while True:
        try:
            raw = input(">> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print("\n종료")
            break

        if not raw:
            continue

        if raw in ("q", "quit", "exit"):
            print("종료")
            break

        if raw == "reset":
            ref.update({"red_tile": None, "black_tile": None})
            print("[RESET] red_tile, black_tile 초기화")
            continue

        parts = raw.split()

        if len(parts) != 2 or parts[0] not in ("red", "black"):
            print("형식 오류. 예: red 3 / black 5 / reset / q")
            continue

        color, idx_str = parts
        try:
            idx = int(idx_str)
        except ValueError:
            print(f"번호가 올바르지 않습니다: {idx_str}")
            continue

        if idx < 1 or idx > 9:
            print("타일 번호는 1~9 사이여야 합니다.")
            continue

        if color == "red":
            ref.update({"red_tile": idx})
            print(f"[PUBLISH] red_tile = {idx}")
        elif color == "black":
            ref.update({"black_tile": idx})
            print(f"[PUBLISH] black_tile = {idx}")


if __name__ == "__main__":
    main()