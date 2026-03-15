"""
firebase_bridge/ros_node.py
FirebaseBridgeNode: ROS2 노드 본체.
ActionHandlerMixin, SensorHandlerMixin, CoworkFlowMixin을 조합.
"""

import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Float32, Int32, String, Float32MultiArray, Bool
from sensor_msgs.msg import JointState
from dsr_msgs2.srv import GetToolForce, GetExternalTorque
from tilemate_msgs.action import ExecuteJob

import firebase_admin
from firebase_admin import credentials, db

from .constants import SERVICE_ACCOUNT_KEY_PATH, DATABASE_URL, INITIAL_ROBOT_STATUS
from .action_handler import ActionHandlerMixin
from .sensor_handler import SensorHandlerMixin
from .cowork_flow import CoworkFlowMixin
from . import tts_helper

# STT / 키워드 추출
try:
    import sys, os as _os
    _this_dir = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
    if _this_dir not in sys.path:
        sys.path.insert(0, _this_dir)
    from STT import STT
    from keyword_extraction import ExtractKeyword
    _STT_AVAILABLE = True
except ImportError as _e:
    _STT_AVAILABLE = False
    print(f"[WARNING] STT/keyword_extraction import 실패: {_e}")


class FirebaseBridgeNode(ActionHandlerMixin, SensorHandlerMixin, CoworkFlowMixin, Node):
    def __init__(self):
        super().__init__("firebase_bridge")

        self._init_firebase()
        self._init_ros_interfaces()
        self._init_state()
        self._init_stt()

        tts_helper.init_pygame_mixer(self.get_logger())

        # Firebase 명령 감시 스레드 시작
        self._cmd_thread = threading.Thread(
            target=self._watch_firebase_command, daemon=True
        )
        self._cmd_thread.start()

    # ── Firebase 초기화 ──────────────────────────────────
    def _init_firebase(self):
        if not os.path.exists(SERVICE_ACCOUNT_KEY_PATH):
            raise FileNotFoundError(f"Firebase key not found: {SERVICE_ACCOUNT_KEY_PATH}")
        if not firebase_admin._apps:
            cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
            firebase_admin.initialize_app(cred, {"databaseURL": DATABASE_URL})

        self.ref = db.reference("/robot_status")
        self.cmd_ref = db.reference("/robot_command")
        self.ref.set(INITIAL_ROBOT_STATUS)
        self.get_logger().info("Firebase Connected!")

        tts_helper.upload_tts_preset(
            "작업을 시작합니다", "tts_start_b64",
            firebase_ref=self.ref, logger=self.get_logger()
        )

    # ── ROS 인터페이스 ───────────────────────────────────
    def _init_ros_interfaces(self):
        # Publishers
        self._pub_cmd         = self.create_publisher(String, "/robot/command", 10)
        self._pub_human_take  = self.create_publisher(Bool, "/dsr01/cowork/human_take_confirm", 10)
        self._pub_cement_done = self.create_publisher(Bool, "/dsr01/cowork/cement_done", 10)

        # Subscribers (로봇 상태 → Firebase)
        self.create_subscription(Int32,            "/robot/step",           self._cb_step,          10)
        self.create_subscription(String,           "/robot/state",          self._cb_state,         10)
        self.create_subscription(Float32MultiArray,"/robot/tcp",            self._cb_tcp,           10)
        self.create_subscription(Int32,            "/robot/completed_jobs", self._cb_completed_jobs,10)
        self.create_subscription(JointState,       "/dsr01/joint_states",   self._cb_joint_states,  10)
        self.create_subscription(Float32,          "/robot/tile_level",     self._cb_tile_level,    10)
        self.create_subscription(Int32,            "/robot/tile_inspect_no",self._cb_tile_inspect_no,10)
        self.create_subscription(Int32,            "/robot/pressing_no",    self._cb_pressing_no,   10)
        self.create_subscription(String,           "/robot/inspection_result", self._cb_inspection_result, 10)
        # Action client
        self.task_job_client = ActionClient(self, ExecuteJob, "/task/run_job")

        # Service clients
        self._tool_force_client = self.create_client(GetToolForce,      "/dsr01/aux_control/get_tool_force")
        self._ext_torque_client = self.create_client(GetExternalTorque, "/dsr01/aux_control/get_external_torque")
        self.create_timer(0.3, self._timer_get_tool_force)
        self.create_timer(0.3, self._timer_get_ext_torque)
        self.get_logger().info("Service clients ready: get_tool_force, get_external_torque")

    # ── 내부 상태 초기화 ─────────────────────────────────
    def _init_state(self):
        # 충돌 감지
        self.COLLISION_THRESHOLD = 60.0
        self.FORCE_THRESHOLD     = 60.0
        self.FORCE_Z_THRESHOLD   = 40.0
        self._collision_detected = False
        self._reset_time         = 0.0

        # 타임스탬프
        self._last_tcp_update   = 0.0
        self._last_joint_update = 0.0

        # 액션 상태
        self._active_goal_handle    = None
        self._active_result_future  = None
        self._active_token          = ""
        self._last_completed_jobs_fb= 0
        self._last_tile_step_fb     = 0
        self._last_cowork_stage     = ""

        # 시멘트 대기 플래그
        self._cement_waiting    = False
        self._cement_wait_thread= None

        # Firebase 명령 상태
        self._last_command = "idle"
        self.cmd_ref.update({"action": "idle"})
        self.get_logger().info(f"Firebase 현재 명령 상태: '{self._last_command}'")

    # ── STT 초기화 ───────────────────────────────────────
    def _init_stt(self):
        if _STT_AVAILABLE:
            try:
                self._stt = STT(os.getenv("OPENAI_API_KEY", ""))
                self._keyword_extractor = ExtractKeyword()
                self._stt.on_listening = lambda: self.ref.update({"stt_mic_state": "listening"})
                self.get_logger().info("STT / KeywordExtractor 초기화 완료")
            except Exception as e:
                self._stt = None
                self._keyword_extractor = None
                self.get_logger().warn(f"STT/KeywordExtractor 초기화 실패: {e}")
        else:
            self._stt = None
            self._keyword_extractor = None

    # ── Firebase 명령 감시 루프 ──────────────────────────
    def _watch_firebase_command(self):
        self.get_logger().info("Firebase command watcher started...")

        while rclpy.ok():
            try:
                raw = self.cmd_ref.get()
                if isinstance(raw, dict):
                    action = str(raw.get("action", "idle")).strip().lower()
                    cmd = raw
                else:
                    action = str(raw).strip().lower() if raw else "idle"
                    cmd = {}

                if action and action != self._last_command:
                    self._last_command = action
                    self._dispatch_command(action, cmd)

            except Exception as e:
                self.get_logger().error(f"Firebase watch error: {e}")

            time.sleep(0.3)

    def _dispatch_command(self, action: str, cmd: dict):
        """Firebase 명령을 적절한 핸들러로 분기."""
        if action == "stt_trigger":
            threading.Thread(target=self._handle_stt_trigger, daemon=True).start()

        elif action == "start":
            self._send_task_job_goal(cmd, is_resume=bool(cmd.get("is_resume", False)))

        elif action == "resume":
            # 웹/외부 클라이언트가 resume에 최소 필드만 보낼 수 있으므로
            # robot_status 값을 보강해 재개 goal 생성 실패를 방지.
            status = {}
            try:
                snap = self.ref.get()
                if isinstance(snap, dict):
                    status = snap
            except Exception as e:
                self.get_logger().warn(f"[RESUME] robot_status 조회 실패 (무시): {e}")

            resume_cmd = dict(cmd) if isinstance(cmd, dict) else {}

            if "design" not in resume_cmd:
                resume_cmd["design"] = int(status.get("design", 0))

            if int(resume_cmd.get("design", 0)) == 3 and not str(resume_cmd.get("custom_pattern", "")).strip():
                resume_cmd["custom_pattern"] = str(status.get("design_ab", "")).strip()

            if "current_step" not in resume_cmd:
                resume_cmd["current_step"] = int(status.get("current_step", 0))
            if "completed_jobs" not in resume_cmd:
                resume_cmd["completed_jobs"] = int(status.get("completed_jobs", 0))
            if "current_tile_index" not in resume_cmd:
                resume_cmd["current_tile_index"] = int(status.get("working_tile", status.get("completed_jobs", 0)))

            # 하위 호환: /robot/command 토픽에도 resume 전파
            msg = String(); msg.data = "resume"
            self._publish_reliable(self._pub_cmd, msg, retries=2)

            self._send_task_job_goal(resume_cmd, is_resume=True)

        elif action == "stop":
            self.get_logger().warn("[CMD] Firebase stop")
            self._cancel_active_goal()
            msg = String(); msg.data = "stop"
            self._publish_reliable(self._pub_cmd, msg, retries=3)

        elif action == "reset":
            self.get_logger().warn("[CMD] Firebase reset")
            self._cancel_active_goal()
            msg = String(); msg.data = "reset"
            self._publish_reliable(self._pub_cmd, msg, retries=2)
            self._collision_detected = False
            self._reset_time = time.time()
            self.ref.set(INITIAL_ROBOT_STATUS)
            self.get_logger().info("[RESET] Firebase robot_status 초기화 완료")

    # ── 유틸 ─────────────────────────────────────────────
    def _publish_reliable(self, publisher, msg, retries=1, interval=0.05):
        for i in range(retries):
            publisher.publish(msg)
            if i < retries - 1:
                time.sleep(interval)

    # ── ROS 간단 콜백 ────────────────────────────────────
    def _cb_step(self, msg: Int32):
        self.ref.update({"current_step": int(msg.data)})

    def _cb_state(self, msg: String):
        self.ref.update({"state": msg.data})

    def _cb_completed_jobs(self, msg: Int32):
        self.ref.update({"completed_jobs": int(msg.data)})

    def _cb_tile_level(self, msg: Float32):
        self.ref.update({"tile_level": float(msg.data)})

    def _cb_tile_inspect_no(self, msg: Int32):
        self.ref.update({"inspect_no": int(msg.data)})

    def _cb_pressing_no(self, msg: Int32):
        self.ref.update({"press_no": int(msg.data)})
    def _cb_inspection_result(self, msg: String):
        import json
        try:
            data = json.loads(msg.data)
            self.ref.update({"inspection_result": data})
            self.get_logger().info("[INSPECT] inspection_result Firebase 업로드 완료")
        except Exception as e:
            self.get_logger().error(f"[INSPECT] JSON 파싱 오류: {e}")
