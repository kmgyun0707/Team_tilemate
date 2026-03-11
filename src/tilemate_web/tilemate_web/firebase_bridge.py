#!/usr/bin/env python3
# firebase_bridge.py
#
# Firebase <-> ROS2 bridge
#
# Firebase robot_command:
#   action: "start" | "resume" | "stop" | "reset" | "idle"
#   design: 1 | 2 | 3
#   custom_pattern: "B,A,B,A,B,A,B,A,B"   # design=3일 때
#   is_resume: true/false
#   completed_jobs: int
#   current_step: int
#   current_tile_index: int
#   token: "job_xxx"
#
# ROS2:
#   - 상위 작업 실행: ExecuteJob ActionClient -> /task/run_job
#   - stop/reset 호환용: /robot/command publish 유지
#   - 상태 구독: /robot/step, /robot/state, /robot/tcp, /robot/completed_jobs 등
#   - 서비스: get_tool_force, get_external_torque
#
# 주의:
#   현재 TaskManagerNode는 PickTile client가 선언돼 있지만 실제로는
#   Place -> Inspect -> Compact 흐름만 수행하고 있음.
#   따라서 feedback의 tile_step도 현재 TaskManager 구현 기준으로 반영됨.

import os
import time
import math
import threading
import tempfile
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Float32, Int32, String, Float32MultiArray, Bool
from sensor_msgs.msg import JointState
from dsr_msgs2.srv import GetToolForce, GetExternalTorque

from tilemate_msgs.action import ExecuteJob

import firebase_admin
from firebase_admin import credentials, db

# STT / 키워드 추출
try:
    import sys
    import os as _os
    _this_dir = _os.path.dirname(_os.path.abspath(__file__))
    if _this_dir not in sys.path:
        sys.path.insert(0, _this_dir)
    from STT import STT
    from keyword_extraction import ExtractKeyword
    _STT_AVAILABLE = True
except ImportError as _stt_err:
    _STT_AVAILABLE = False
    print(f"[WARNING] STT/keyword_extraction import 실패: {_stt_err}")

# TTS (gtts + pygame 또는 playsound)
try:
    from gtts import gTTS
    import pygame
    _TTS_AVAILABLE = True
except ImportError:
    _TTS_AVAILABLE = False
    print("[WARNING] gTTS 또는 pygame 없음. TTS 비활성화. pip install gtts pygame")


# --------------------------------------------------
# Firebase 설정
# --------------------------------------------------
SERVICE_ACCOUNT_KEY_PATH = os.path.expanduser(
    "~/Team_tilemate/src/tilemate_web/config/co1-tiling-firebase-adminsdk-fbsvc-f4f88c3832.json"
)
DATABASE_URL = "https://co1-tiling-default-rtdb.asia-southeast1.firebasedatabase.app"


class FirebaseBridgeNode(Node):
    def __init__(self):
        super().__init__("firebase_bridge")

        # --------------------------------------------------
        # Firebase init
        # --------------------------------------------------
        if not os.path.exists(SERVICE_ACCOUNT_KEY_PATH):
            raise FileNotFoundError(f"Firebase key not found: {SERVICE_ACCOUNT_KEY_PATH}")

        if not firebase_admin._apps:
            cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
            firebase_admin.initialize_app(cred, {"databaseURL": DATABASE_URL})

        self.ref = db.reference("/robot_status")
        self.cmd_ref = db.reference("/robot_command")

        self.get_logger().info("Firebase Connected!")

        # --------------------------------------------------
        # Firebase 초기 상태
        # --------------------------------------------------
        self.ref.set({
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
            "pos_x": 0.0,
            "pos_y": 0.0,
            "pos_z": 0.0,
            "joint_speed": 0.0,
            "tile_level": 0.0,
            "inspect_no": 0,
            "press_no": 0,
            "tool_force": {"0": 0.0, "1": 0.0, "2": 0.0, "3": 0.0, "4": 0.0, "5": 0.0},
            "force_z": 0.0,
            "force_total": 0.0,
            "ext_torque": {"0": 0.0, "1": 0.0, "2": 0.0, "3": 0.0, "4": 0.0, "5": 0.0},
            "message": "",
        })

        # --------------------------------------------------
        # publisher (기존 하위 정지/리셋 체계 호환용)
        # --------------------------------------------------
        self._pub_cmd = self.create_publisher(String, "/robot/command", 10)

        # --------------------------------------------------
        # CoworkActionServer 연동 publisher
        # /dsr01/cowork/human_take_confirm  → 사람이 타일 잡았음
        # /dsr01/cowork/cement_done         → 시멘트 도포 완료
        # --------------------------------------------------
        self._pub_human_take = self.create_publisher(
            Bool, "/dsr01/cowork/human_take_confirm", 10
        )
        self._pub_cement_done = self.create_publisher(
            Bool, "/dsr01/cowork/cement_done", 10
        )

        # --------------------------------------------------
        # subscriber (로봇 상태 -> Firebase)
        # --------------------------------------------------
        self.create_subscription(Int32, "/robot/step", self._cb_step, 10)
        self.create_subscription(String, "/robot/state", self._cb_state, 10)
        self.create_subscription(Float32MultiArray, "/robot/tcp", self._cb_tcp, 10)
        self.create_subscription(Int32, "/robot/completed_jobs", self._cb_completed_jobs, 10)
        self.create_subscription(JointState, "/dsr01/joint_states", self._cb_joint_states, 10)
        self.create_subscription(Float32, "/robot/tile_level", self._cb_tile_level, 10)
        self.create_subscription(Int32, "/robot/tile_inspect_no", self._cb_tile_inspect_no, 10)
        self.create_subscription(Int32, "/robot/pressing_no", self._cb_pressing_no, 10)

        # --------------------------------------------------
        # ExecuteJob ActionClient
        # TaskManagerNode:
        #   ActionServer(ExecuteJob, "task/run_job", ...)
        # --------------------------------------------------
        self.task_job_client = ActionClient(self, ExecuteJob, "/task/run_job")

        self._active_goal_handle = None
        self._active_result_future = None
        self._active_token = ""
        self._last_completed_jobs_fb = 0
        self._last_tile_step_fb = 0
        self._last_cowork_stage = ""

        # --------------------------------------------------
        # 서비스 클라이언트
        # --------------------------------------------------
        self._tool_force_client = self.create_client(
            GetToolForce, "/dsr01/aux_control/get_tool_force"
        )
        self._ext_torque_client = self.create_client(
            GetExternalTorque, "/dsr01/aux_control/get_external_torque"
        )

        self.create_timer(0.3, self._timer_get_tool_force)
        self.create_timer(0.3, self._timer_get_ext_torque)

        self.get_logger().info("Service clients ready: get_tool_force, get_external_torque")

        # --------------------------------------------------
        # 충돌 감지 설정
        # --------------------------------------------------
        self.COLLISION_THRESHOLD = 60.0
        self.FORCE_THRESHOLD = 60.0
        self.FORCE_Z_THRESHOLD = 40.0
        self._collision_detected = False
        self._reset_time = 0.0

        self._last_tcp_update = 0.0
        self._last_joint_update = 0.0

        # --------------------------------------------------
        # Firebase command watcher
        # --------------------------------------------------
        self._last_command = "idle"
        self.get_logger().info(f"Firebase 현재 명령 상태: '{self._last_command}'")
        self.cmd_ref.update({"action": "idle"})

        # STT / 키워드 추출기
        if _STT_AVAILABLE:
            try:
                self._stt = STT(os.getenv("OPENAI_API_KEY", ""))
                self._keyword_extractor = ExtractKeyword()
                # 캘리브레이션 완료 → Firebase stt_mic_state: listening 알림
                self._stt.on_listening = lambda: self.ref.update({"stt_mic_state": "listening"})
                self.get_logger().info("STT / KeywordExtractor 초기화 완료")
            except Exception as e:
                self._stt = None
                self._keyword_extractor = None
                self.get_logger().warn(f"STT/KeywordExtractor 초기화 실패: {e}")
        else:
            self._stt = None
            self._keyword_extractor = None

        # --------------------------------------------------
        # 시멘트 대기 상태 플래그
        # --------------------------------------------------
        self._cement_waiting = False        # True면 "끝났어" 대기 중
        self._cement_wait_thread = None

        # pygame 초기화 (TTS 재생용)
        if _TTS_AVAILABLE:
            try:
                pygame.mixer.init()
            except Exception as e:
                self.get_logger().warn(f"[TTS] pygame.mixer 초기화 실패: {e}")

        self._cmd_thread = threading.Thread(
            target=self._watch_firebase_command,
            daemon=True,
        )
        self._cmd_thread.start()

    # ==================================================
    # Action helpers
    # ==================================================
    def _pattern_to_layout(self, pattern_str: str):
        """
        "B,A,B,A,B,A,B,A,B" -> [2,1,2,1,2,1,2,1,2]
        A=흰색=1, B=검정=2, C=데코=3
        """
        if not pattern_str:
            return []

        tokens = [x.strip().upper() for x in pattern_str.split(",") if x.strip()]
        mapping = {"A": 1, "B": 2, "C": 3}  # A=흰색=1, B=검정=2, C=데코=3

        layout = []
        for t in tokens:
            if t not in mapping:
                raise ValueError(f"Unknown tile symbol in pattern: {t}")
            layout.append(mapping[t])

        return layout

    def _resolve_design_pattern(self, cmd_dict):
        """
        Firebase design -> pattern string
        """
        design = int(cmd_dict.get("design", 0))

        if design == 1:
            # zigzag: 흰색 시작 체크무늬 B=흰색, A=검정
            return "B,A,B,A,B,A,B,A,B", design
        elif design == 2:
            # straight: B=흰색줄, A=검정줄
            return "B,B,B,A,A,A,B,B,B", design
        elif design == 3:
            custom_pattern = str(cmd_dict.get("custom_pattern", "")).strip()
            if not custom_pattern:
                raise ValueError("design=3 but no custom_pattern")
            return custom_pattern, design
        else:
            raise ValueError(f"Unsupported design: {design}")

    def _send_task_job_goal(self, cmd_dict, is_resume: bool):
        if not self.task_job_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("ExecuteJob action server not available: /task/run_job")
            self.ref.update({"state": "작업 서버 연결 실패"})
            return

        pattern_str, design_id = self._resolve_design_pattern(cmd_dict)
        layout = self._pattern_to_layout(pattern_str)

        token = str(cmd_dict.get("token", f"job_{int(time.time() * 1000)}"))
        completed_jobs = int(cmd_dict.get("completed_jobs", 0))
        current_step = int(cmd_dict.get("current_step", 0))
        current_tile_index = int(cmd_dict.get("current_tile_index", completed_jobs))

        goal = ExecuteJob.Goal()
        goal.token = token
        goal.design_layout = layout
        goal.is_resume = bool(is_resume)
        goal.completed_jobs = completed_jobs
        goal.current_step = current_step
        goal.current_tile_index = current_tile_index

        self._active_token = token
        self._last_completed_jobs_fb = completed_jobs if is_resume else 0
        self._last_tile_step_fb = current_step if is_resume else 0

        self.ref.update({
            "state": "작업 시작 요청",
            "design": design_id,
            "design_ab": pattern_str,
            "token": token,
            "is_resume": bool(is_resume),
            "completed_jobs": completed_jobs if is_resume else 0,
            "current_step": current_step if is_resume else 0,
            "working_tile": current_tile_index if is_resume else 0,
            "overall_progress": 0.0,
            "tile_progress": 0.0,
            "tile_step": current_step if is_resume else 0,
            "message": "",
        })

        self.get_logger().info(
            f"[TASK_JOB] send goal: token={token}, resume={is_resume}, "
            f"layout={layout}, completed_jobs={completed_jobs}, "
            f"current_step={current_step}, current_tile_index={current_tile_index}"
        )

        send_future = self.task_job_client.send_goal_async(
            goal,
            feedback_callback=self._fb_task_job,
        )
        send_future.add_done_callback(self._on_task_job_goal_response)

    def _on_task_job_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"[TASK_JOB] goal response failed: {e}")
            self.ref.update({"state": f"goal response error: {e}"})
            return

        if not goal_handle.accepted:
            self.get_logger().error("[TASK_JOB] goal rejected")
            self.ref.update({"state": "작업 요청 거절"})
            self._active_goal_handle = None
            return

        self.get_logger().info("[TASK_JOB] goal accepted")
        self.ref.update({"state": "작업 수락됨"})
        self._active_goal_handle = goal_handle
        self._active_result_future = goal_handle.get_result_async()
        self._active_result_future.add_done_callback(self._on_task_job_result)

    def _on_task_job_result(self, future):
        try:
            result_wrap = future.result()
            result = result_wrap.result
        except Exception as e:
            self.get_logger().error(f"[TASK_JOB] result failed: {e}")
            self.ref.update({"state": f"result error: {e}"})
            self._active_goal_handle = None
            self._active_result_future = None
            return

        self._active_goal_handle = None
        self._active_result_future = None

        if result.success:
            self.get_logger().info(f"[TASK_JOB] success: {result.message}")
            self.ref.update({
                "state": "작업 완료",
                "message": result.message,
                "overall_progress": 1.0,
                "tile_progress": 1.0,
            })
        else:
            self.get_logger().warn(f"[TASK_JOB] failed: {result.message}")
            self.ref.update({
                "state": "작업 실패",
                "message": result.message,
            })

    def _fb_task_job(self, fb_msg):
        fb = fb_msg.feedback

        tile_index = int(fb.tile_index)
        tile_type = int(fb.tile_type)
        
        # 액션 파일의 detail_step을 파이어베이스의 tile_step으로 매핑
        tile_step = int(fb.detail_step) 
        overall_step = int(fb.overall_step)

        # ── 피드백 전체 터미널 출력 ──
        print(
            f"\n[FB] ─────────────────────────────────────\n"
            f"  overall_step   : {overall_step}\n"
            f"  tile_step      : {tile_step}  (prev={self._last_tile_step_fb})\n"
            f"  tile_index     : {tile_index}\n"
            f"  tile_type      : {tile_type}\n"
            f"  overall_prog   : {float(fb.overall_progress):.3f}\n"
            f"  detail_prog    : {float(fb.detail_progress):.3f}\n"
            f"  state          : {fb.state}\n"
            f"─────────────────────────────────────────",
            flush=True,
        )

        # 완료 개수 추정
        # TaskManager에서는 tile_step == TILE_STEP_DONE(5)일 때 타일 완료로 볼 수 있음
        completed_jobs = self._last_completed_jobs_fb
        if tile_step == 5 and self._last_tile_step_fb != 5:
            completed_jobs = max(completed_jobs, tile_index + 1)
            self._last_completed_jobs_fb = completed_jobs

        # --------------------------------------------------
        # state 문자열 기반 트리거
        # "WAIT_HUMAN_TAKE" 포함 → waiting_pick
        # "WAIT_CEMENT_DONE" 포함 → waiting_cement
        # --------------------------------------------------
        current_state = str(fb.state)

        if "WAIT_HUMAN_TAKE" in current_state and "WAIT_HUMAN_TAKE" not in self._last_cowork_stage and not self._cement_waiting:
            self.get_logger().info(f"[CEMENT] WAIT_HUMAN_TAKE 감지 → waiting_pick 시작 (state={current_state})")
            self._cement_waiting = True
            self._cement_wait_thread = threading.Thread(
                target=self._cement_pick_flow,
                daemon=True,
            )
            self._cement_wait_thread.start()

        elif "WAIT_CEMENT_DONE" in current_state and "WAIT_CEMENT_DONE" not in self._last_cowork_stage and not self._cement_waiting:
            self.get_logger().info(f"[CEMENT] WAIT_CEMENT_DONE 감지 → waiting_cement 시작 (state={current_state})")
            self._cement_waiting = True
            self._cement_wait_thread = threading.Thread(
                target=self._cement_done_flow,
                daemon=True,
            )
            self._cement_wait_thread.start()

        self._last_cowork_stage = current_state
        self._last_tile_step_fb = tile_step

        self.ref.update({
            "current_step": overall_step,
            "overall_progress": float(fb.overall_progress),
            "working_tile": tile_index,
            "tile_type": tile_type,
            "tile_step": tile_step,
            # 액션 파일의 detail_progress를 파이어베이스의 tile_progress로 매핑
            "tile_progress": float(fb.detail_progress), 
            "completed_jobs": completed_jobs,
            "state": str(fb.state),
        })

    def _cancel_active_goal(self):
        if self._active_goal_handle is None:
            self.get_logger().warn("[TASK_JOB] no active goal to cancel")
            return

        try:
            self.get_logger().warn("[TASK_JOB] cancel active goal")
            cancel_future = self._active_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._on_cancel_done)
        except Exception as e:
            self.get_logger().error(f"[TASK_JOB] cancel failed: {e}")

    def _on_cancel_done(self, future):
        try:
            _ = future.result()
            self.get_logger().warn("[TASK_JOB] cancel response received")
            self.ref.update({"state": "작업 취소 요청됨"})
        except Exception as e:
            self.get_logger().error(f"[TASK_JOB] cancel result error: {e}")

    # ==================================================
    # Tool force service
    # ==================================================
    def _timer_get_tool_force(self):
        if not self._tool_force_client.service_is_ready():
            return

        req = GetToolForce.Request()
        req.ref = 0
        future = self._tool_force_client.call_async(req)
        future.add_done_callback(self._cb_tool_force_response)

    def _cb_tool_force_response(self, future):
        try:
            res = future.result()
            if res.success:
                forces_dict = {
                    str(i): 0.0 if math.isnan(v) else float(round(v, 2))
                    for i, v in enumerate(res.tool_force)
                }

                fx = forces_dict.get("0", 0.0)
                fy = forces_dict.get("1", 0.0)
                fz = forces_dict.get("2", 0.0)

                force_total = float(round(math.sqrt(fx**2 + fy**2 + fz**2), 2))

                self.ref.update({
                    "tool_force": forces_dict,
                    "force_z": fz,
                    "force_total": force_total,
                })

                # TCP 합력 임계 초과
                if (
                    not self._collision_detected
                    and (time.time() - self._reset_time > 3.0)
                    and force_total > self.FORCE_THRESHOLD
                ):
                    self._collision_detected = True
                    self.get_logger().warn(
                        f"[FORCE DETECTED] force_total={force_total} > {self.FORCE_THRESHOLD}"
                    )

                    self.cmd_ref.update({"action": "stop", "timestamp": int(time.time() * 1000)})
                    self.ref.update({
                        "state": "충돌 감지 - 비상정지",
                        "collision_joint": 0,
                        "collision_torque": force_total,
                    })

                    self._cancel_active_goal()
                    msg = String()
                    msg.data = "stop"
                    self._publish_reliable(self._pub_cmd, msg, retries=3)

                # Fz 임계 초과
                elif (
                    not self._collision_detected
                    and (time.time() - self._reset_time > 3.0)
                    and abs(fz) > self.FORCE_Z_THRESHOLD
                ):
                    self._collision_detected = True
                    self.get_logger().warn(
                        f"[FORCE_Z DETECTED] fz={fz} > {self.FORCE_Z_THRESHOLD}"
                    )

                    self.cmd_ref.update({"action": "stop", "timestamp": int(time.time() * 1000)})
                    self.ref.update({
                        "state": "충돌 감지 - 비상정지",
                        "collision_joint": -1,
                        "collision_torque": fz,
                    })

                    self._cancel_active_goal()
                    msg = String()
                    msg.data = "stop"
                    self._publish_reliable(self._pub_cmd, msg, retries=3)

        except Exception as e:
            self.get_logger().error(f"[FORCE] error: {e}")

    # ==================================================
    # External torque service
    # ==================================================
    def _timer_get_ext_torque(self):
        if not self._ext_torque_client.service_is_ready():
            return

        req = GetExternalTorque.Request()
        future = self._ext_torque_client.call_async(req)
        future.add_done_callback(self._cb_ext_torque_response)

    def _cb_ext_torque_response(self, future):
        try:
            res = future.result()
            if res.success:
                torque_dict = {
                    str(i): 0.0 if math.isnan(v) else float(round(v, 2))
                    for i, v in enumerate(res.ext_torque)
                }

                self.ref.update({"ext_torque": torque_dict})

                if not self._collision_detected and (time.time() - self._reset_time > 3.0):
                    for i, v in torque_dict.items():
                        if abs(v) > self.COLLISION_THRESHOLD:
                            self._collision_detected = True
                            self.get_logger().warn(
                                f"[COLLISION DETECTED] J{int(i)+1} torque={v} > {self.COLLISION_THRESHOLD}"
                            )

                            self.cmd_ref.update({"action": "stop", "timestamp": int(time.time() * 1000)})
                            self.ref.update({
                                "state": "충돌 감지 - 비상정지",
                                "collision_joint": int(i) + 1,
                                "collision_torque": v,
                            })

                            self._cancel_active_goal()
                            msg = String()
                            msg.data = "stop"
                            self._publish_reliable(self._pub_cmd, msg, retries=3)
                            break

        except Exception as e:
            self.get_logger().error(f"[EXT_TORQUE] error: {e}")

    # ==================================================
    # Reliable publish
    # ==================================================
    def _publish_reliable(self, publisher, msg, retries=1, interval=0.05):
        for i in range(retries):
            publisher.publish(msg)
            if i < retries - 1:
                time.sleep(interval)

    # ==================================================
    # TTS 헬퍼
    # ==================================================
    def _speak(self, text: str):
        """
        텍스트를 한국어 TTS로 재생합니다.
        gTTS로 mp3 생성 후 pygame으로 재생.
        """
        if not _TTS_AVAILABLE:
            self.get_logger().warn(f"[TTS] 비활성화 상태. 출력 생략: '{text}'")
            return
        try:
            with tempfile.NamedTemporaryFile(suffix=".mp3", delete=False) as f:
                tmp_path = f.name
            tts = gTTS(text=text, lang="ko")
            tts.save(tmp_path)

            pygame.mixer.music.load(tmp_path)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                time.sleep(0.1)

            os.remove(tmp_path)
            self.get_logger().info(f"[TTS] 재생 완료: '{text}'")
        except Exception as e:
            self.get_logger().error(f"[TTS] 오류: {e}")

    # ==================================================
    # 시멘트 대기 흐름 (STEP 1 완료 → 대기 → STEP 2 재개)
    # ==================================================
    def _cement_pick_flow(self):
        """
        WAIT_HUMAN_TAKE 진입 시 실행
        - 웹 팝업: waiting_pick
        - TTS: "타일을 잡아주세요"
        - STT: "타일 잡았어" 계열 감지
        - → ROS publish: /dsr01/cowork/human_take_confirm (Bool true)
        """
        try:
            PICK_KEYWORDS = ["잡았어", "잡았다", "집었어", "잡았음", "타일 잡았어", "집었다"]

            self.ref.update({
                "state": "타일 파지 대기 중 - 타일을 잡으면 '타일 잡았어' 라고 말해주세요",
                "cement_state": "waiting_pick",
            })
            self.get_logger().info("[CEMENT] 1단계: 타일 파지 대기")
            self.ref.update({"stt_mic_state": "tts_speaking"})
            self._speak("타일을 잡아주세요")
            time.sleep(0.5)

            if self._stt is None:
                self.get_logger().warn("[CEMENT] STT 없음. 5초 후 자동 진행합니다.")
                time.sleep(5.0)
            else:
                recognized = False
                first_attempt = True
                while not recognized:
                    try:
                        self.get_logger().info("[CEMENT] 🎙 타일 파지 음성 대기 중...")
                        if first_attempt:
                            self.ref.update({"stt_mic_state": "calibrating"})
                        text = self._stt.speech2text()
                        first_attempt = False
                        self.get_logger().info(f"[CEMENT] STT 결과: '{text}'")
                        if any(kw in text for kw in PICK_KEYWORDS):
                            recognized = True
                            self.ref.update({"stt_mic_state": ""})
                        else:
                            self.get_logger().info("[CEMENT] 키워드 미감지. 재청취...")
                            self.ref.update({"stt_mic_state": "retry"})
                    except Exception as e:
                        self.get_logger().error(f"[CEMENT] STT 오류: {e}")
                        self.ref.update({"stt_mic_state": "retry"})
                        first_attempt = False
                        time.sleep(1.0)

            msg_take = Bool()
            msg_take.data = True
            self._publish_reliable(self._pub_human_take, msg_take, retries=3)
            self.get_logger().info("[CEMENT] human_take_confirm 토픽 전송 완료")
            self.ref.update({"state": "타일 내려놓는 중", "cement_state": "tile_release", "stt_mic_state": ""})

        except Exception as e:
            self.get_logger().error(f"[CEMENT] _cement_pick_flow 오류: {e}")
        finally:
            self._cement_waiting = False

    def _cement_done_flow(self):
        """
        WAIT_CEMENT_DONE 진입 시 실행
        - 웹 팝업: waiting_cement
        - TTS: "시멘트를 발라주세요"
        - STT: "시멘트 다 발랐어" 계열 감지
        - TTS: "타일을 두세요"
        - → ROS publish: /dsr01/cowork/cement_done (Bool true)
        """
        try:
            CEMENT_KEYWORDS = ["발랐어", "발랐다", "다 발랐어", "시멘트 다 발랐어", "완료", "됐어", "다됐어", "끝났어", "끝났다"]

            self.ref.update({
                "state": "시멘트 도포 대기 중 - 다 바르면 '시멘트 다 발랐어' 라고 말해주세요",
                "cement_state": "waiting_cement",
                "stt_mic_state": "",
            })
            self.get_logger().info("[CEMENT] 2단계: 시멘트 도포 대기")
            self.ref.update({"stt_mic_state": "tts_speaking"})
            self._speak("시멘트를 발라주세요")
            time.sleep(0.5)

            if self._stt is None:
                self.get_logger().warn("[CEMENT] STT 없음. 5초 후 자동 재개합니다.")
                time.sleep(5.0)
            else:
                recognized = False
                first_attempt = True
                while not recognized:
                    try:
                        self.get_logger().info("[CEMENT] 🎙 시멘트 완료 음성 대기 중...")
                        if first_attempt:
                            self.ref.update({"stt_mic_state": "calibrating"})
                        text = self._stt.speech2text()
                        first_attempt = False
                        self.get_logger().info(f"[CEMENT] STT 결과: '{text}'")
                        if any(kw in text for kw in CEMENT_KEYWORDS):
                            recognized = True
                            self.ref.update({"stt_mic_state": ""})
                        else:
                            self.get_logger().info("[CEMENT] 키워드 미감지. 재청취...")
                            self.ref.update({"stt_mic_state": "retry"})
                    except Exception as e:
                        self.get_logger().error(f"[CEMENT] STT 오류: {e}")
                        self.ref.update({"stt_mic_state": "retry"})
                        first_attempt = False
                        time.sleep(1.0)

            self.ref.update({"stt_mic_state": "tts_speaking"})
            self._speak("타일을 두세요")
            time.sleep(0.5)

            msg_cement = Bool()
            msg_cement.data = True
            self._publish_reliable(self._pub_cement_done, msg_cement, retries=3)
            self.get_logger().info("[CEMENT] cement_done 토픽 전송 완료")

            self._speak("작업을 재개할게요")
            self.ref.update({
                "state": "시멘트 완료 - 작업 재개",
                "cement_state": "done",
                "stt_mic_state": "",
            })
            self.get_logger().info("[CEMENT] 재개 완료")

        except Exception as e:
            self.get_logger().error(f"[CEMENT] _cement_done_flow 오류: {e}")
        finally:
            self._cement_waiting = False

    # ==================================================
    # STT + Keyword Extraction
    # ==================================================
    def _handle_stt_trigger(self):
        """
        음성 녹음(STT) → 키워드 추출 → Firebase robot_status/stt_layout 저장
        index.html이 stt_layout을 폴링하여 결과를 표시합니다.
        """
        self.get_logger().info("[STT] 음성 녹음 시작")
        self.ref.update({"stt_layout": None, "stt_state": "recording"})

        try:
            if self._stt is None or self._keyword_extractor is None:
                raise RuntimeError("STT 또는 KeywordExtractor가 초기화되지 않았습니다.")

            # 1) 음성 녹음 및 텍스트 변환 (VAD 자동 감지)
            text = self._stt.speech2text()
            self.get_logger().info(f"[STT] 인식 결과: {text}")
            self.ref.update({"stt_state": "extracting", "stt_text": text})

            # 2) 키워드 추출 → layout [1~2 × 9]
            layout = self._keyword_extractor.extract_keyword(text)
            if layout is None:
                raise ValueError(f"키워드 추출 실패: '{text}'")

            self.get_logger().info(f"[STT] layout: {layout}")

            # 3) Firebase에 저장 (index.html이 폴링으로 감지)
            self.ref.update({
                "stt_layout": layout,
                "stt_state": "done",
            })

        except Exception as e:
            self.get_logger().error(f"[STT] 오류: {e}")
            self.ref.update({"stt_state": "error", "stt_error": str(e)})

    # ==================================================
    # Firebase command watcher
    # ==================================================
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

                    # stt_trigger -> STT 녹음 + 키워드 추출 후 Firebase에 layout 저장
                    if action == "stt_trigger":
                        stt_thread = threading.Thread(
                            target=self._handle_stt_trigger,
                            daemon=True,
                        )
                        stt_thread.start()

                    # start -> ExecuteJob goal
                    elif action == "start":
                        is_resume = bool(cmd.get("is_resume", False))
                        self._send_task_job_goal(cmd, is_resume=is_resume)

                    # resume -> ExecuteJob goal (강제 true)
                    elif action == "resume":
                        self._send_task_job_goal(cmd, is_resume=True)

                    # stop -> active goal cancel + 기존 stop topic
                    elif action == "stop":
                        self.get_logger().warn("[CMD] Firebase stop")
                        self._cancel_active_goal()

                        msg = String()
                        msg.data = "stop"
                        self._publish_reliable(self._pub_cmd, msg, retries=3)

                    # reset -> active goal cancel + 기존 reset topic + firebase reset
                    elif action == "reset":
                        self.get_logger().warn("[CMD] Firebase reset")
                        self._cancel_active_goal()

                        msg = String()
                        msg.data = "reset"
                        self._publish_reliable(self._pub_cmd, msg, retries=2)

                        self._collision_detected = False
                        self._reset_time = time.time()

                        self.ref.set({
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
                            "pos_x": 0.0,
                            "pos_y": 0.0,
                            "pos_z": 0.0,
                            "joint_speed": 0.0,
                            "tile_level": 0.0,
                            "inspect_no": 0,
                            "press_no": 0,
                            "tool_force": {"0": 0.0, "1": 0.0, "2": 0.0, "3": 0.0, "4": 0.0, "5": 0.0},
                            "force_z": 0.0,
                            "force_total": 0.0,
                            "ext_torque": {"0": 0.0, "1": 0.0, "2": 0.0, "3": 0.0, "4": 0.0, "5": 0.0},
                            "message": "",
                        })

                        self.get_logger().info("[RESET] Firebase robot_status 초기화 완료")

            except Exception as e:
                self.get_logger().error(f"Firebase watch error: {e}")

            time.sleep(0.3)

    # ==================================================
    # ROS state callbacks -> Firebase
    # ==================================================
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

    # ==================================================
    # TCP velocity
    # ==================================================
    M0609_DH = [
        (0.0,   0.1555,  math.pi / 2),
        (0.409, 0.0,     0.0),
        (0.367, 0.0,     0.0),
        (0.0,   0.1335,  math.pi / 2),
        (0.0,   0.0995, -math.pi / 2),
        (0.0,   0.0996,  0.0),
    ]

    def _compute_tcp_velocity(self, q, qdot):
        dh = self.M0609_DH

        T = np.eye(4)
        transforms = [T.copy()]

        for i, (a, d, alpha) in enumerate(dh):
            ct = math.cos(q[i])
            st = math.sin(q[i])
            ca = math.cos(alpha)
            sa = math.sin(alpha)

            A = np.array([
                [ct, -st * ca,  st * sa, a * ct],
                [st,  ct * ca, -ct * sa, a * st],
                [0,   sa,       ca,      d],
                [0,   0,        0,       1],
            ])

            T = T @ A
            transforms.append(T.copy())

        p_n = transforms[6][:3, 3]
        J_v = np.zeros((3, 6))

        for i in range(6):
            z_i = transforms[i][:3, 2]
            p_i = transforms[i][:3, 3]
            J_v[:, i] = np.cross(z_i, p_n - p_i)

        v_tcp = J_v @ np.array(qdot)
        return float(round(np.linalg.norm(v_tcp), 4))

    def _cb_joint_states(self, msg: JointState):
        now = time.time()
        if now - self._last_joint_update < 0.5:
            return
        self._last_joint_update = now

        pos = list(msg.position)
        vel = list(msg.velocity)

        if len(pos) >= 6 and len(vel) >= 6:
            tcp_speed = self._compute_tcp_velocity(pos[:6], vel[:6])
            self.ref.update({"joint_speed": tcp_speed})

    def _cb_tcp(self, msg: Float32MultiArray):
        now = time.time()
        if now - self._last_tcp_update < 0.2:
            return
        self._last_tcp_update = now

        data = msg.data
        if len(data) >= 3:
            self.ref.update({
                "pos_x": round(float(data[0]), 2),
                "pos_y": round(float(data[1]), 2),
                "pos_z": round(float(data[2]), 2),
            })


def main(args=None):
    rclpy.init(args=args)
    node = FirebaseBridgeNode()
    print("Firebase Bridge running... (Ctrl+C to stop)")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        try:
            db.reference("/robot_status").update({
                "state": "대기",
                "current_step": 0,
            })
        except Exception:
            pass

        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()