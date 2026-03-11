"""
firebase_bridge/constants.py
Firebase 초기 상태 및 공통 상수 정의
"""

import os

SERVICE_ACCOUNT_KEY_PATH = os.path.expanduser(
    "~/Team_tilemate/src/tilemate_web/config/co1-tiling-firebase-adminsdk-fbsvc-f4f88c3832.json"
)
DATABASE_URL = "https://co1-tiling-default-rtdb.asia-southeast1.firebasedatabase.app"

INITIAL_ROBOT_STATUS = {
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
    "tool_force": {"0": 0.0, "1": 0.0, "2": 0.0, "3": 0.0, "4": 0.0, "5": 0.0},
    "force_z": 0.0,
    "force_total": 0.0,
    "ext_torque": {"0": 0.0, "1": 0.0, "2": 0.0, "3": 0.0, "4": 0.0, "5": 0.0},
    "message": "",
}

DESIGN_PATTERNS = {
    1: "B,A,B,A,B,A,B,A,B",
    2: "B,B,B,A,A,A,B,B,B",
}

TILE_SYMBOL_MAP = {"A": 1, "B": 2, "C": 3}  # A=흰색, B=검정, C=데코
