#!/usr/bin/env python3
# tilemate_main/tile_motion_node.py
#
# ✅ Option B: Stop(진짜 취소/정지) -> Resume(앱 레벨 재시작)
# - stop_soft 들어오면: 외부 interrupt_node에서 MoveStop으로 로봇 정지
#   tile_motion_node는 error로 끝내지 않고 "stopped:<tok>:<checkpoint>"로 종료
# - resume 들어오면: MoveResume(로봇레벨) ❌ 신뢰하지 않음
#   /tile/resume 토픽을 받아 checkpoint부터 남은 시퀀스를 재실행(restart from checkpoint)
#
# ✅ resume 시 안전 재진입:
#   1) compliance 해제
#   2) WORLD z lift
#   3) JReady 정렬
#   4) phase 롤백(보수적으로 PICK부터 재시작)파일

# 수정 - 타일별 기울기 측정후 불량타일 압착 진행 


import time
import traceback
import threading
from typing import Any, Dict, Optional, List, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, Float64, String

import DR_init
from tilemate_main.robot_config import RobotConfig


# ----------------------------
# params
# ----------------------------
VELOCITY = 30
ACC = 30

OPEN_W  = 0.040
CLOSE_W = 0.019
#압착판 그리퍼 간격 (예시값, 실제 환경에 맞게 조정 필요)
COMPACT_CLOSE_W = 0.005  # 압착판을 잡은 상태의 간격 (예: 30mm)


# ----------------------------
# positions
# ----------------------------
PICK_ABOVE_A = [344.54+5.0-2.0+8.0-8.0, -100.1-5.0-2.0+5.0, 260.95, 74.35, 178.8, 73.81]
PICK_ABOVE_B = [436.16+1.0-3.0+5.0-1.0, -98.84+2.0-1.0-5.0, 260.78, 69.2, 178.73, 68.63]

PLACE_TILT_BASE01 = [402.08, 158.83, 229.77, 75.0, 178.60, 77.00]
PLACE_TILT_BASE02 = [469.39, 157.97, 228.80, 75.0, 178.60, 77.00]
PLACE_TILT_BASE03 = [539.79, 156.41, 229.13, 75.0, 178.60, 77.00]
PLACE_TILT_BASE04 = [401.85,  90.05, 229.55, 75.0, 178.60, 77.00]
PLACE_TILT_BASE05 = [469.94,  89.92, 228.78, 75.0, 178.60, 77.00]
PLACE_TILT_BASE06 = [539.63,  88.61, 228.16, 75.0, 178.60, 77.00]
PLACE_TILT_BASE07 = [401.19,  22.88, 228.09, 75.0, 178.60, 77.00]
PLACE_TILT_BASE08 = [468.62,  21.50, 227.98, 75.0, 178.60, 77.00]
PLACE_TILT_BASE09 = [539.05,  20.33, 227.54, 75.0, 178.60, 77.00]

TOOL_GRIP_ABOVE = [531.2, -101.3, 210, 169.29, 177.87, 169.98]
TOOL_GRIP_DOWN  = [531.2, -101.3, 165, 169.29, 177.87, 169.98]
TOOL_WAYPOINT   = [470, 24, 230, 6, -179, 97]

# [✅ 추가] 압착판(Compaction Tool) 거치대 좌표 (실제 환경에 맞게 수정 필요)
COMPACT_TOOL_ABOVE = [531.2, -101.3 -87, 220.0, 169.29, 177.87, 169.98]
COMPACT_TOOL_DOWN  = [531.2, -101.3 -87, 146.0, 169.29, 177.87, 169.98]
COMPACT_TOOL_WAYPOINT =[470.0, 24.0, 230.0, 6.0, -179.0, 97.0]
    
class _GripperClient:
    def __init__(self, node: Node):
        self._node = node
        self._pub = node.create_publisher(Float64, "/gripper/width_m", 10)

    def set_width(self, width_m: float):
        msg = Float64()
        msg.data = float(width_m)
        self._pub.publish(msg)
        self._node.get_logger().info(f"[GRIPPER->CMD] width_m={msg.data:.4f}")

    def grab(self):
        self._node.get_logger().info("[GRIPPER] grab")
        self.set_width(CLOSE_W)
        time.sleep(1.0)

    def release(self):
        self._node.get_logger().info("[GRIPPER] release")
        self.set_width(OPEN_W)
        time.sleep(1.0)

    def close_fully(self):
        # [✅ 추가] 기울기 측정을 위해 그리퍼를 완전히 닫아 뾰족하게 만듦
        self._node.get_logger().info("[GRIPPER] 기울기 측정을 위해 그리퍼 완전 닫기(0.0)")
        self.set_width(0.0)
        time.sleep(1.0)       


class TileMotionNode(Node):
    STEP_IDLE   = 0
    STEP_PICK   = 3 # 수정금지 ! 타일에서 3 4 -> 태스크 매니저에 3,4로 매핑되어있음 
    STEP_PLACE  = 4
    STEP_DETACH = 4
    STEP_DONE   = 5

    def __init__(self, cfg: RobotConfig, boot_node: Node):
        super().__init__("tile_motion_node", namespace=cfg.robot_id)
        self.cfg = cfg
        self._boot_node = boot_node

        # flags
        self._pause = False
        self._stop_soft = False

        # run token
        self._pending_token: Optional[int] = None
        self._last_token: Optional[int] = None

        # resume
        self._resume_requested = False

        # checkpoint
        # 예: {"phase":"PICK","tile_i":4} = 4번째 타일의 PICK 단계부터 재시작
        self._checkpoint: Optional[Dict[str, Any]] = None
        self._stopped = False
        # [✅ 추가] 압착이 필요한 타일 번호를 저장하는 딕셔너리 {타일번호(1~9): True/False}
        self._needs_compaction: Dict[int, bool] = {}

        # worker
        self._running = False
        self._worker = None
        self._worker_done = True
        self._worker_ok = False
        self._worker_tok: Optional[int] = None
        self._worker_err = ""  # "stopped" | error

        # pubs
        self.pub_status = self.create_publisher(String, "/tile/status", 10)
        self.pub_step   = self.create_publisher(Int32,  "/tile/step", 10)
        self.pub_state  = self.create_publisher(String, "/robot/state", 10)
        self.pub_completed_jobs = self.create_publisher(Int32, "/robot/completed_jobs", 10) # 타일 각각 작업 완료시 퍼블리셔

        # subs
        self.create_subscription(Int32, "/tile/run_once", self._cb_run_once, 10)
        self.create_subscription(Bool,  "/tile/resume",   self._cb_resume, 10)
        self.create_subscription(Bool,  "/task/pause",    self._cb_pause, 10)
        self.create_subscription(Bool,  "/task/stop_soft",self._cb_stop_soft, 10)
        self.create_subscription(String,"/robot/design_ab", self._cb_design_ab, 10)

        self._design_pattern = ["A"] * 9
        self.gripper = _GripperClient(self)

        self._initialize_robot()
        self._set_tile_status(self.STEP_IDLE, "작업명령 대기중")
        self.get_logger().info("TileMotionNode ready!!!")

    # -----------------
    # init / helpers
    # -----------------
    def _initialize_robot(self):
        from DSR_ROBOT2 import set_tool, set_tcp, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS, set_robot_mode
        self.get_logger().info("[TILE] initialize_robot()")
        set_robot_mode(ROBOT_MODE_MANUAL)
        set_tool(self.cfg.tool)
        set_tcp(self.cfg.tcp)
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        time.sleep(0.5)

    def _set_tile_status(self, step: int, state: str):
        m_step = Int32()
        m_step.data = int(step)
        m_state = String()
        m_state.data = str(state)
        self.pub_step.publish(m_step)
        self.pub_state.publish(m_state)
        self.get_logger().info(f"[TILE] step={m_step.data} state='{m_state.data}'")

    def _publish_status(self, s: str):
        m = String()
        m.data = s
        self.pub_status.publish(m)
        self.get_logger().info(f"[TILE->STATUS] {m.data}")

    def _wait_if_paused(self):
        if self._pause:
            self._set_tile_status(self.STEP_IDLE, "일시정지(pause)")
        while rclpy.ok() and self._pause and not self._stop_soft:
            time.sleep(0.05)

    def _sleep_interruptible(self, sec: float, dt: float = 0.05) -> bool:
        t0 = time.time()
        while (time.time() - t0) < float(sec):
            if self._stop_soft:
                return False
            self._wait_if_paused()
            time.sleep(float(dt))
        return True

    def _check_abort(self) -> bool:
        if self._stop_soft:
            self.get_logger().warn("[TILE] stop requested (stop_soft=True)")
            return True
        self._wait_if_paused()
        return bool(self._stop_soft)

    # ---- checkpoint helpers ----
    def _set_ckpt(self, phase: str, tile_i: int):
        new_ckpt = {"phase": str(phase), "tile_i": int(tile_i)}
        if self._checkpoint == new_ckpt:
            return  # ✅ 같은 값이면 중복 로그/중복 publish 방지
        self._checkpoint = new_ckpt
        self.get_logger().info(f"[TILE][CKPT] set {self._checkpoint_to_string()}")

    def _checkpoint_to_string(self) -> str:
        if not self._checkpoint:
            return "none"
        phase = self._checkpoint.get("phase", "none")
        tile_i = int(self._checkpoint.get("tile_i", 1))
        return f"{phase}:{tile_i}"

    # -----------------
    # callbacks
    # -----------------
    def _cb_design_ab(self, msg: String):
        raw = msg.data
        self._design_pattern = [x.strip().upper() for x in raw.split(",")]
        self.get_logger().info(f"[TILE] design pattern updated: {self._design_pattern}")

    def _cb_run_once(self, msg: Int32):
        if self._running:
            self.get_logger().warn("[TILE] run_once ignored (already running)")
            return
        self._pending_token = int(msg.data)
        self.get_logger().info(f"[TILE] received token={self._pending_token}")

    def _cb_pause(self, msg: Bool):
        self._pause = bool(msg.data)
        self.get_logger().warn(f"[TILE] pause={self._pause}")

    def _cb_stop_soft(self, msg: Bool):
        self._stop_soft = bool(msg.data)
        self.get_logger().warn(f"[TILE] stop_soft={self._stop_soft}")

    def _cb_resume(self, msg: Bool):
        if not bool(msg.data):
            return
        self.get_logger().warn("[TILE] resume requested (/tile/resume)")
        self._resume_requested = True


    def return_tool(self) -> bool:
        """
        ✅ 작업 완료 시 흡착 툴을 거치대에 반납.
        - stop_soft / pause 고려: 각 구간마다 _check_abort(), _wait_if_paused()로 중단/일시정지 대응
        - 실제 즉시정지는 interrupt_node(MoveStop)가 담당 (여긴 "다음 모션 발행 차단"만 보장)
        - 성공 True / 중단 또는 실패 False
        """
        from DSR_ROBOT2 import movel, wait, posx, DR_BASE

        self.get_logger().info("[TILE] 작업 완료. 흡착 툴을 거치대에 반납합니다...")

        # 0) 중단/일시정지 즉시 반영
        if self._check_abort():
            self.get_logger().warn("[TILE][RETURN_TOOL] aborted before start")
            return False

        try:
            # 1) 안전 구역(Waypoint)으로 먼저 이동
            self._wait_if_paused()
            if self._check_abort():
                return False
            movel(posx(list(TOOL_WAYPOINT)), vel=VELOCITY, acc=ACC, ref=DR_BASE)
            if not self._sleep_interruptible(0.2):
                return False

            # 2) 툴 거치대 상부로 이동
            self._wait_if_paused()
            if self._check_abort():
                return False
            movel(posx(list(TOOL_GRIP_ABOVE)), vel=VELOCITY, acc=ACC, ref=DR_BASE)
            if not self._sleep_interruptible(0.2):
                return False

            # 3) 툴 거치대 위치로 하강
            self._wait_if_paused()
            if self._check_abort():
                return False
            movel(posx(list(TOOL_GRIP_DOWN)), vel=VELOCITY, acc=ACC, ref=DR_BASE)
            wait(0.5)
            if not self._sleep_interruptible(0.1):
                return False

            # 4) 그리퍼를 열어서 툴 놓기
            self._wait_if_paused()
            if self._check_abort():
                return False
            self.gripper.release()
            if self._check_abort():
                return False

            # 5) 툴과 부딪히지 않게 다시 상단으로 상승
            self._wait_if_paused()
            if self._check_abort():
                return False
            movel(posx(list(TOOL_GRIP_ABOVE)), vel=VELOCITY, acc=ACC, ref=DR_BASE)
            if not self._sleep_interruptible(0.2):
                return False

            self.get_logger().info("✅ [TILE] 흡착 툴 반납 완료!")
            return True

        except Exception as e:
            # stop_soft 중이면 "stopped"로 취급 (상위 worker가 stopped로 마무리하게)
            if self._stop_soft:
                self.get_logger().warn(f"[TILE][RETURN_TOOL] exception during stop -> treat as stopped: {e}")
                return False
            self.get_logger().error(f"[TILE][RETURN_TOOL] failed: {e}")
            return False
        
    def handle_compact_tool(self, action="GRAB") -> bool:
        """[✅ 추가] 압착 툴 파지 및 반납 함수"""
        from DSR_ROBOT2 import movel, wait, posx, DR_BASE
        self.get_logger().info(f"[TILE] 압착 툴 {action} 시작...")
        if self._check_abort(): return False
        try:
            self._wait_if_paused()
            movel(posx(list(TOOL_WAYPOINT)), vel=VELOCITY, acc=ACC, ref=DR_BASE)
            if not self._sleep_interruptible(0.2): return False
            
            movel(posx(list(COMPACT_TOOL_ABOVE)), vel=VELOCITY, acc=ACC, ref=DR_BASE)
            if not self._sleep_interruptible(0.2): return False
            
            if action == "RETURN": 
                movel(posx(list(COMPACT_TOOL_DOWN)), vel=VELOCITY, acc=ACC, ref=DR_BASE)
                self.gripper.release()

            else: self.gripper.release() # 잡기 전에도 열어두기
            
            movel(posx(list(COMPACT_TOOL_DOWN)), vel=VELOCITY, acc=ACC, ref=DR_BASE)
            wait(0.5)

            # [✅ 수정] grab() 대신, 상단에 선언해둔 COMPACT_CLOSE_W(0.005) 넓이로 직접 꽉 닫게 만듭니다.
            if action == "GRAB": 
                self.gripper.set_width(COMPACT_CLOSE_W)
                time.sleep(1.0)
            elif action == "RETURN": self.gripper.release()
          
            if self._check_abort(): return False
            movel(posx(list(COMPACT_TOOL_ABOVE)), vel=VELOCITY, acc=ACC, ref=DR_BASE)
            if not self._sleep_interruptible(0.2): return False
            
            movel(posx(list(TOOL_WAYPOINT)), vel=VELOCITY, acc=ACC, ref=DR_BASE)
            self.get_logger().info(f"✅ [TILE] 압착 툴 {action} 완료!")
            return True
        except Exception as e:
            if self._stop_soft: return False
            self.get_logger().error(f"[TILE][COMPACT_TOOL] failed: {e}")
            return False



    # -----------------
    # tick (worker orchestration)
    # -----------------
    def tick(self):
        # 0) worker 종료 처리
        if self._running and self._worker_done:
            tok = self._worker_tok
            ok = self._worker_ok
            err = self._worker_err

            if ok and not self._stop_soft:
                self._publish_status(f"done:{tok}")
                self._stopped = False
                self._checkpoint = None
                self._set_tile_status(self.STEP_DONE, "타일 작업 완료")

            elif err == "stopped":
                ck = self._checkpoint_to_string()
                self._publish_status(f"stopped:{tok}:{ck}")
                self._stopped = True
                self._last_token = tok

            else:
                if self._stop_soft and not err:
                    err = "aborted(stop_soft)"
                self._publish_status(f"error:{tok}:{err or 'aborted/failed'}")
                self._stopped = False
                self._checkpoint = None

            # reset
            self._running = False
            self._worker = None
            self._worker_tok = None
            self._worker_done = True
            self._worker_ok = False
            self._worker_err = ""
            return

        # 1) worker 돌고 있으면 return
        if self._running:
            return

        # 2) resume 처리
        if self._resume_requested:
            self._resume_requested = False

            if self._stop_soft:
                self.get_logger().warn("[TILE] resume ignored: stop_soft=True (set stop_soft False first)")
                return

            if not self._stopped or self._last_token is None or self._checkpoint is None:
                self.get_logger().warn(
                    f"[TILE] resume ignored: not stopped (stopped={self._stopped}, "
                    f"last_token={self._last_token}, ckpt={self._checkpoint_to_string()})"
                )
                return

            tok = int(self._last_token)
            ckpt = dict(self._checkpoint)

            self.get_logger().warn(f"[TILE] resume start tok={tok} from ckpt={self._checkpoint_to_string()}")
            self._start_worker(tok=tok, start_ckpt=ckpt, resume_mode=True)
            return

        # 3) new run_once
        if self._pending_token is None:
            return

        tok = int(self._pending_token)
        self._pending_token = None

        if self._stop_soft:
            self.get_logger().warn("[TILE] stop_soft=True -> skip token")
            return

        self._checkpoint = None
        self._stopped = False
        self._last_token = tok

        self._needs_compaction.clear() # [✅ 추가] 새 작업 시작 시 압착 데이터 초기화

        self._start_worker(tok=tok, start_ckpt=None, resume_mode=False)

    def _start_worker(self, tok: int, start_ckpt: Optional[Dict[str, Any]], resume_mode: bool):
        self._running = True
        self._worker_done = False
        self._worker_ok = False
        self._worker_tok = tok
        self._worker_err = ""

        def _run_worker():
            try:
                self._wait_if_paused()
                if self._stop_soft:
                    self._worker_ok = False
                    self._worker_err = "stopped"
                    return

                self.get_logger().info(f"[TILE] worker start token={tok} resume_mode={resume_mode}")
                ok = self._perform_cycle(start_ckpt=start_ckpt, resume_mode=resume_mode)
                self._worker_ok = bool(ok)

                if not ok and not self._worker_err:
                    self._worker_err = "stopped" if self._stop_soft else "aborted/failed"

            except Exception as e:
                if self._stop_soft:
                    self.get_logger().warn(f"[TILE] exception during stop -> treat as stopped: {e}")
                    self._worker_ok = False
                    self._worker_err = "stopped"
                else:
                    self.get_logger().error(f"[TILE] exception in worker: {e}")
                    self.get_logger().error(traceback.format_exc())
                    self._worker_ok = False
                    self._worker_err = str(e)

            finally:
                self._worker_done = True

        self._worker = threading.Thread(target=_run_worker, daemon=True)
        self._worker.start()

    # -----------------
    # main tile cycle (worker thread)
    # -----------------
    def _perform_cycle(self, start_ckpt: Optional[Dict[str, Any]], resume_mode: bool) -> bool:
        from DSR_ROBOT2 import (
            posx, movej, movel, wait, get_current_posx,
            release_compliance_ctrl, release_force,
            set_robot_mode, set_tool, set_tcp,
            DR_BASE, DR_TOOL, DR_WORLD,
            ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS,
        )

        # ---- safe wrappers (blocking 유지 버전) ----
        # stop_soft는 "다음 모션 발행 차단"만 보장. 실제 즉시정지는 interrupt_node(MoveStop)가 담당.
        def safe_movej(j, **kwargs) -> bool:
            if self._check_abort():
                self._worker_err = "stopped"
                return False
            try:
                movej(j, **kwargs)
                time.sleep(0.2)
                return True
            except Exception as e:
                if self._stop_soft:
                    self._worker_err = "stopped"
                    return False
                self._worker_err = f"movej failed: {e}"
                return False

        def _to_posx(p):
            if isinstance(p, (list, tuple)):
                return posx(list(p))
            return p

        def safe_movel(p, **kwargs) -> bool:
            if self._check_abort():
                self._worker_err = "stopped"
                return False
            try:
                if "ref" not in kwargs:
                    kwargs["ref"] = DR_BASE  # ✅ 절대좌표는 base 고정
                movel(_to_posx(p), **kwargs)
                time.sleep(0.5)
                return True
            except Exception as e:
                if self._stop_soft:
                    self._worker_err = "stopped"
                    return False
                self._worker_err = f"movel failed: {e}"
                return False

        def disable_compliance():
            try:
                release_force()
            except Exception:
                pass
            try:
                release_compliance_ctrl()
            except Exception:
                pass

        def rearm_tool_tcp(tool: str, tcp: str) -> bool:
            try:
                set_robot_mode(ROBOT_MODE_MANUAL)
                set_tool(tool)
                set_tcp(tcp)
                set_robot_mode(ROBOT_MODE_AUTONOMOUS)
                return self._sleep_interruptible(0.3)
            except Exception as e:
                self._worker_err = "stopped" if self._stop_soft else f"rearm failed: {e}"
                return False

        def home_align_with_lift(jready, lift_mm: float = 30.0) -> bool:
            disable_compliance()
            if self._check_abort():
                self._worker_err = "stopped"
                return False

            # WORLD lift (현재 자세에서 z만 올림)
            try:
                cur, _ = get_current_posx(DR_WORLD)
                lift = [cur[0], cur[1], cur[2] + float(lift_mm), cur[3], cur[4], cur[5]]
                safe_movel(posx(lift), ref=DR_WORLD, vel=10, acc=10)
                if not self._sleep_interruptible(0.2):
                    self._worker_err = "stopped"
                    return False
            except Exception:
                if self._stop_soft:
                    self._worker_err = "stopped"
                    return False

            if not safe_movej(jready, vel=20, acc=20):
                return False
            return self._sleep_interruptible(0.2)

        # ---- your helpers ----
        def move_relative(dx: float, dy: float, dz: float) -> bool:
            if self._check_abort():
                self._worker_err = "stopped"
                return False
            cur, _ = get_current_posx(DR_BASE)
            target = posx([cur[0] + dx, cur[1] + dy, cur[2] + dz, cur[3], cur[4], cur[5]])
            if not safe_movel(target, ref=DR_BASE, vel=VELOCITY, acc=ACC):
                return False
            return self._sleep_interruptible(1.0)

        def compliant_approach(threshold_n=11.0, timeout_s=10.0) -> bool:
            # ✅ 반드시 timeout/stop 체크가 있는 버전으로 교체 추천
            from DSR_ROBOT2 import (
                set_ref_coord, task_compliance_ctrl, set_desired_force,
                check_force_condition, release_force, release_compliance_ctrl,
                DR_FC_MOD_REL, DR_AXIS_Z
            )
            self._wait_if_paused()
            if self._stop_soft:
                return False

            set_ref_coord(DR_TOOL)
            task_compliance_ctrl(stx=[3000, 3000, 80, 200, 200, 200], time=0.0)
            wait(0.2)

            set_desired_force(
                fd=[0, 0, float(threshold_n + 15), 0, 0, 0],
                dir=[0, 0, 1, 0, 0, 0],
                mod=DR_FC_MOD_REL
            )

            t0 = time.time()
            try:
                while True:
                    self._wait_if_paused()
                    if self._stop_soft:
                        return False
                    if timeout_s is not None and (time.time() - t0) > float(timeout_s):
                        self.get_logger().warn("[COMPLIANT] timeout -> fail")
                        return False
                    ret = check_force_condition(DR_AXIS_Z, min=0, max=float(threshold_n))
                    if ret == -1:
                        return True
                    wait(0.05)
            finally:
                try: release_force()
                except Exception: pass
                try: release_compliance_ctrl()
                except Exception: pass
                wait(0.1)

        def smart_twist_compaction(timeout_s=15.0) -> bool:
            """[✅ 추가] 손목 관절을 비비면서 타일을 압착하는 스마트 다짐 함수"""
            from DSR_ROBOT2 import (
                set_ref_coord, task_compliance_ctrl, set_desired_force,
                check_force_condition, release_force, release_compliance_ctrl,
                DR_FC_MOD_REL, DR_AXIS_Z, DR_TOOL, DR_BASE,
                get_current_posx, get_current_posj, amovej, get_tool_force
            )
            self._wait_if_paused()
            if self._stop_soft: return False

            # 1. 압착 하강 준비
            set_ref_coord(DR_TOOL)
            task_compliance_ctrl(stx=[3000, 3000, 20, 200, 200, 200], time=0.0)
            wait(0.2)

            self.get_logger().info("   ⬇️ 30N의 힘으로 바닥을 향해 하강합니다.")
            set_desired_force(fd=[0, 0, 30.0, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

            t0 = time.time()
            touched = False
            contact_z = 0.0
            contact_joint = None

            try:
                # 2. 바닥 감지 (6N)
                while (time.time() - t0) < float(timeout_s):
                    self._wait_if_paused()
                    if self._stop_soft: return False
                    
                    if check_force_condition(DR_AXIS_Z, min=0, max=6.0) == -1:
                        touched = True
                        contact_pos, _ = get_current_posx(DR_BASE)
                        contact_z = contact_pos[2]
                        contact_joint = get_current_posj()
                        self.get_logger().info(f"   ✅ 바닥 접촉 감지 완료 (Z: {contact_z:.2f}mm)")
                        break
                    wait(0.05)

                if not touched:
                    self.get_logger().error("   ❌ 바닥 감지 실패 (타임아웃).")
                    return False

                # 3. 조인트 비비기 시퀀스
                self.get_logger().info("   ↔️ 손목 관절(Joint 6) 비비기 시작: 좌우 20도")
                set_desired_force(fd=[0, 0, 10.0, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL) # 10n 시 타일 이동 방지 위해 힘 낮춤
                
                press_t0 = time.time()
                direction_flag = 1
                last_switch_time = time.time()

                while (time.time() - press_t0) < 3.0:  # 3초간 비비기
                    if self._check_abort(): return False
                    
                    current_time = time.time()
                    if current_time - last_switch_time > 0.5:
                        direction_flag *= -1
                        last_switch_time = current_time
                    
                    target_joint = list(contact_joint)
                    target_joint[5] = contact_joint[5] + (10.0 * direction_flag)
                    
                    amovej(target_joint, vel=80, acc=80)
                    
                    # 로깅 (과부하 방지를 위해 주석 처리하거나 살려두셔도 됨)
                    # cur_force = get_tool_force(DR_BASE)
                    # self.get_logger().info(f"   ↔️ 목표 각도: {target_joint[5]:.1f}도 | Fz: {abs(cur_force[2]):.1f}N")
                    wait(0.1)

                # 4. 마무리 다짐(Tamping)
                self.get_logger().info("   ✅ 비비기 완료. 원래 각도로 복귀하여 다짐(Tamping) 중...")
                amovej(contact_joint, vel=40, acc=40)
                
                # 3초 대기 (일시정지/정지 체크 포함)
                if not self._sleep_interruptible(3.0): return False

                final_pos, _ = get_current_posx(DR_BASE)
                self.get_logger().info(f"   📊 최종 안착: {final_pos[2]:.2f} mm (압착 깊이: {contact_z - final_pos[2]:.2f} mm)")
                return True

            finally:
                disable_compliance()
                wait(0.1)



        def probe_single_point(p_safe) -> Optional[float]:
            """[✅ 추가] 지정된 좌표에서 바닥을 터치하고 Z값을 반환"""
            from DSR_ROBOT2 import (
                set_ref_coord, task_compliance_ctrl, set_desired_force, 
                check_force_condition, DR_FC_MOD_REL, DR_AXIS_Z, DR_TOOL
            )
            if not safe_movel(p_safe, vel=VELOCITY, acc=ACC, ref=DR_BASE): return None
            
            set_ref_coord(DR_TOOL)
            task_compliance_ctrl(stx=[3000, 3000, 20, 200, 200, 200], time=0.0)
            wait(0.2)
            set_desired_force(fd=[0, 0, 30.0, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            
            t0 = time.time()
            z_val = None
            touched = False
            
            try:
                while (time.time() - t0) < 15.0:
                    if self._check_abort(): return None
                    if check_force_condition(DR_AXIS_Z, min=0, max=5.0) == -1:
                        disable_compliance()
                        wait(0.2)
                        cur_pos, _ = get_current_posx(DR_BASE)
                        z_val = cur_pos[2]
                        touched = True
                        break
                    wait(0.05)
            finally:
                disable_compliance()
                
            safe_movel(p_safe, vel=VELOCITY, acc=ACC, ref=DR_BASE) # 원복
            
            if not touched:
                self.get_logger().error("❌ [INSPECT] 바닥 감지 시간 초과! (15초 넘게 닿지 않음. Z 시작 높이를 확인하세요)")
            return z_val if touched else None

        def detach_tile(tile_idx: int) -> bool:
            if self._check_abort():
                self._worker_err = "stopped"
                return False
            from DSR_ROBOT2 import posx, movel, set_robot_mode, add_tcp, set_tcp
            from DSR_ROBOT2 import ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS, DR_TOOL

            try:
                set_robot_mode(ROBOT_MODE_MANUAL)
                tcp_name = "MySuction_v1"
                tcp_offset = [0, 0, 275, 0, 0, 0]
                try:
                    add_tcp(tcp_name, tcp_offset)
                except Exception:
                    pass
                set_tcp(tcp_name)
                set_robot_mode(ROBOT_MODE_AUTONOMOUS)
                wait(0.2)

                tilt_angle = -26 if (tile_idx % 3 == 0) else 24
                tilt_forward = posx([0, 0, 0, 0, tilt_angle, 0])
                movel(tilt_forward, vel=30, acc=30, ref=DR_TOOL, time=0.5)
                
                wait(0.2)
                return True
            finally:
                try:
                    set_robot_mode(ROBOT_MODE_MANUAL)
                    set_tcp(self.cfg.tcp)
                    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
                except Exception:
                    pass

        # ---- sequence data ----
        JReady = [0, 0, 90, 0, 90, 90]

        place_targets: List[Tuple[int, List[float]]] = [
            (1, PLACE_TILT_BASE01),
            (2, PLACE_TILT_BASE02),
            (3, PLACE_TILT_BASE03),
            (4, PLACE_TILT_BASE04),
            (5, PLACE_TILT_BASE05),
            (6, PLACE_TILT_BASE06),
            (7, PLACE_TILT_BASE07),
            (8, PLACE_TILT_BASE08),
            (9, PLACE_TILT_BASE09),
        ]

        # ---- checkpoint init ----
        ck = start_ckpt or self._checkpoint or {"phase": "PREPARE", "tile_i": 1}

        # ✅ resume_mode 안전 재진입
        if resume_mode:
            self._set_tile_status(self.STEP_IDLE, f"재개(resume) 홈정렬중 ckpt={ck.get('phase')}:{ck.get('tile_i')}")
            if not rearm_tool_tcp(self.cfg.tool, self.cfg.tcp):
                return False
            if not home_align_with_lift(JReady, lift_mm=30.0):
                return False

            ph = str(ck.get("phase", "PREPARE"))

            # ✅ 도구 파지 중/직전에서 멈췄으면: 반드시 PREPARE부터 (도구 파지 다시)
            if ph in ("PREPARE", "JREADY") or ph.startswith("TOOL_"):
                self.get_logger().warn(f"[TILE][RESUME] tool-phase({ph}) -> rollback to PREPARE (re-grasp tool)")
                ck["phase"] = "PREPARE"
                ck["tile_i"] = 1  # 도구 파지는 타일 인덱스 의미 없음

            # ✅ 타일 작업 중(PLACE/DETACH)에서 멈췄으면: 보수적으로 PICK부터
            elif ph in ("PLACE", "DETACH"):
                self.get_logger().warn("[TILE][RESUME] rollback phase to PICK for stable re-entry")
                ck["phase"] = "PICK"
                # tile_i는 유지
            # [✅ 추가] 검사 및 압착 중 멈췄을 때 롤백 로직
            elif ph.startswith("INSPECT"):
                ck["phase"] = "INSPECT_START" # 검사는 무조건 처음부터 다시
            elif ph.startswith("COMPACT"):
                ck["phase"] = "COMPACT" # 압착은 해당 타일부터 다시

        # =========================
        # PHASE: PREPARE (tool grasp + waypoint)
        # =========================
        if ck["phase"] == "PREPARE":
            # ✅ PREPARE 진입 자체를 ckpt로 남김 (worker 진입 직후 stop에도 대비)
            self._set_ckpt("PREPARE", int(ck.get("tile_i", 1)))

            self._set_tile_status(self.STEP_IDLE, "JReady 이동 및 도구 파지")
            self._set_ckpt("JREADY", int(ck.get("tile_i", 1)))
            if not safe_movej(JReady, vel=VELOCITY, acc=ACC): return False

            # ----- tool grasp sequence -----
            self._set_ckpt("TOOL_RELEASE", 1)
            self.gripper.release()

            self._set_ckpt("TOOL_APPROACH_ABOVE", 1)
            if not safe_movel(posx(TOOL_GRIP_ABOVE), vel=VELOCITY, acc=ACC): return False

            self._set_ckpt("TOOL_APPROACH_DOWN", 1)
            if not safe_movel(posx(TOOL_GRIP_DOWN),  vel=VELOCITY, acc=ACC): return False

            self._set_ckpt("TOOL_GRAB", 1)
            self.gripper.grab()

            self._set_ckpt("TOOL_LIFT", 1)
            if not safe_movel(posx(TOOL_GRIP_ABOVE), vel=VELOCITY, acc=ACC): return False
            
            if not move_relative(0, 100, 0): return False

            self._set_tile_status(self.STEP_IDLE, "안전구역(Waypoint) 이동")
            self._set_ckpt("TOOL_WAYPOINT", 1)
            if not safe_movel(posx(TOOL_WAYPOINT), vel=VELOCITY, acc=ACC): return False

            # 다음 단계 진입 ckpt 
            # [타일 배치 진입점]
            next_tile_i = int(ck.get("tile_i", 1))

            # [✅ 테스트용: 배치 건너뛰고 바로 툴 반납 -> 검사로 점프!]
            # self._set_ckpt("RETURN_SUCTION_TOOL", 0)
            # ck = {"phase": "RETURN_SUCTION_TOOL", "tile_i": 0}

            #원본 
            self._set_ckpt("PICK", next_tile_i)
            ck = {"phase": "PICK", "tile_i": next_tile_i}

        # =========================
        # PHASE: TILE LOOP from tile_i
        # =========================

        # ✅ 타일 루프는 PICK/PLACE/DETACH/DONE에서만 진입 허용
        ph = str(ck.get("phase", "PREPARE"))
        if ph not in ("PICK", "PLACE", "DETACH", "DONE"):
            self.get_logger().warn(f"[TILE] phase={ph} -> block tile loop (must run PREPARE first)")
            self._worker_err = f"invalid_phase_before_tile_loop:{ph}"
            return False


        start_tile_i = int(ck.get("tile_i", 1))
        if start_tile_i < 1: start_tile_i = 1
        if start_tile_i > 9: start_tile_i = 9

        # tile_i-1 인덱스부터 진행
        for idx in range(start_tile_i - 1, len(place_targets)):
            tile_i, place_pos = place_targets[idx]

            # design pick
            list_index = tile_i - 1
            tile_type = self._design_pattern[list_index] if list_index < len(self._design_pattern) else "A"
            if tile_type == "B":
                pick_pos = PICK_ABOVE_B
                color_name = "흰색"
            else:
                pick_pos = PICK_ABOVE_A
                color_name = "검정"

            # ---------------- PICK ----------------
            self._set_ckpt("PICK", tile_i)
            if self._check_abort():
                self._worker_err = "stopped"
                return False

            self._set_tile_status(self.STEP_PICK, f"타일 파지 준비({color_name}) - {tile_i}번")
            if not safe_movel(posx(pick_pos), vel=VELOCITY, acc=ACC): return False

            self._set_tile_status(self.STEP_PICK, f"타일 파지 하강 - {tile_i}번")
            if not compliant_approach(threshold_n=13.0, timeout_s=10.0):
                self._worker_err = "stopped" if self._stop_soft else "pick_compliant_failed"
                self._sleep_interruptible(0.5)
                return False

            self._set_tile_status(self.STEP_PICK, f"타일 파지 상승 - {tile_i}번")
            if not safe_movel(posx(pick_pos), vel=VELOCITY, acc=ACC): return False
            if not move_relative(0, 100, 0): return False

            # ---------------- PLACE ----------------
            self._set_ckpt("PLACE", tile_i)
            if self._check_abort():
                self._worker_err = "stopped"
                return False

            self._set_tile_status(self.STEP_PLACE, f"타일 배치 상부 - {tile_i}번")
            if not safe_movel(posx(place_pos), vel=VELOCITY, acc=ACC): return False

            self._set_tile_status(self.STEP_PLACE, f"타일 배치 하강 - {tile_i}번")
            if not compliant_approach(threshold_n=11.0, timeout_s=10.0):
                self._worker_err = "stopped" if self._stop_soft else "place_compliant_failed"
                return False

            # ---------------- DETACH ----------------
            self._set_ckpt("DETACH", tile_i)
            if self._check_abort():
                self._worker_err = "stopped"
                return False

            self._set_tile_status(self.STEP_DETACH, f"타일 박리(detach) - {tile_i}번")
            if not detach_tile(tile_i): return False

            self._set_tile_status(self.STEP_PLACE, f"타일 배치 상부 복귀 - {tile_i}번")
            if not safe_movel(posx(place_pos), vel=VELOCITY, acc=ACC): return False

            self.get_logger().info(f"🎉 {tile_i}번 타일 완료")
            m = Int32()
            m.data = int(tile_i)  # 또는 누적 완료 개수면 idx+1
            self.pub_completed_jobs.publish(m)
            self.get_logger().info(f"[TILE] publish /robot/completed_jobs={m.data}")
            

            # 다음 타일로 넘어가기 전에 checkpoint advance
            # (다음 resume는 tile_i+1의 PICK부터 시작)
            if tile_i < 9:
                self._set_ckpt("PICK", tile_i + 1)

        # =========================
        # [✅ 추가된 PHASE] 툴 반납 -> 전수 검사 -> 불량 타일 압착
        # =========================
        
        # 1. 흡착 툴 반납 및 검사 준비
        # PICK/PLACE/DETACH 페이즈가 다 끝나면 이 페이즈로 자연스럽게 넘어오게 함
        if ph in ("PICK", "PLACE", "DETACH"):
            ck = {"phase": "RETURN_SUCTION_TOOL", "tile_i": 0}

        if ck["phase"] == "RETURN_SUCTION_TOOL":
            self._set_ckpt("RETURN_SUCTION_TOOL", 0)
            if not self.return_tool(): 
                self._worker_err = "stopped" if self._stop_soft else "return_tool_failed"
                return False
            
            # [✅ 추가] 툴과 부딪히지 않도록 탁 트인 안전 구역(웨이포인트, Z=230)으로 먼저 도피!
            self.get_logger().info("[TILE] 그리퍼를 닫기 위해 안전 높이로 이동합니다.")
            if not safe_movel(posx(TOOL_WAYPOINT), vel=VELOCITY, acc=ACC): return False
            
            # 만약 웨이포인트 말고 그냥 그 자리에서 위로만 50mm 쑥! 올리고 싶으시다면 아래 코드를 대신 쓰셔도 됩니다.
            # if not move_relative(0, 0, 50.0): return False 
            
            # 안전한 공간에서 검사를 위해 그리퍼를 뾰족하게 완전히 닫음
            self.gripper.close_fully()
            ck = {"phase": "INSPECT_START", "tile_i": 1}

        # 2. 모든 타일 3점 터치 검사
        if ck["phase"] in ("INSPECT_START", "INSPECT"):
            self._set_tile_status(self.STEP_IDLE, "타일 기울기(단차) 전수 검사 중")
            start_idx = int(ck.get("tile_i", 1))
            
            for idx in range(start_idx - 1, len(place_targets)):
                tile_i, center_pos = place_targets[idx]
                self._set_ckpt("INSPECT", tile_i)
                
                z_safe = center_pos[2] -40
                rx, ry, rz = center_pos[3], center_pos[4], center_pos[5]
                offset = 30.0
                pts = [
                    posx([center_pos[0] + offset, center_pos[1] + offset, z_safe, rx, ry, rz]),
                    posx([center_pos[0] - offset, center_pos[1] + offset, z_safe, rx, ry, rz]),
                    posx([center_pos[0] - offset, center_pos[1] - offset, z_safe, rx, ry, rz])
                ]
                
                z_results = []
                for p in pts:
                    z = probe_single_point(p)
                    if z is None: return False # 중단 또는 오류
                    z_results.append(z)
                
                # 높이 단차 계산
                z_diff = max(z_results) - min(z_results)
                is_bad = z_diff >= 1.5  # 1.5mm 이상 차이나면 불량 판정
                self._needs_compaction[tile_i] = is_bad
                
                self.get_logger().info(f"[INSPECT] {tile_i}번 타일 단차: {z_diff:.2f}mm -> 압착 필요: {is_bad}")
                
            # 검사 종료 후 판별
            if any(self._needs_compaction.values()):
                self.get_logger().info("⚠️ 불량 타일 발견! 압착 시퀀스로 진입합니다.")
                ck = {"phase": "COMPACT_TOOL_GRAB", "tile_i": 1}
            else:
                self.get_logger().info("✅ 모든 타일 양호! 작업을 종료합니다.")
                ck = {"phase": "DONE", "tile_i": 0}

        # 3. 압착판 파지
        if ck["phase"] == "COMPACT_TOOL_GRAB":
            self._set_ckpt("COMPACT_TOOL_GRAB", 0)
            self._set_tile_status(self.STEP_IDLE, "압착판 파지 중")
            if not self.handle_compact_tool("GRAB"): 
                self._worker_err = "stopped" if self._stop_soft else "compact_tool_grab_failed"
                return False
            ck = {"phase": "COMPACT", "tile_i": 1}

        # 4. 불량 타일 스마트 압착
        if ck["phase"] == "COMPACT":
            self._set_tile_status(self.STEP_IDLE, "불량 타일 스마트 압착 진행")
            start_idx = int(ck.get("tile_i", 1))
            
            for idx in range(start_idx - 1, len(place_targets)):
                tile_i, place_pos = place_targets[idx]
                
                if self._needs_compaction.get(tile_i, False):
                    self._set_ckpt("COMPACT", tile_i)
                    self.get_logger().info(f"[COMPACT] {tile_i}번 타일 누르기 실행")
                    
                    safe_place = list(place_pos)
                    safe_place[2] -= 35.0
                    if not safe_movel(posx(safe_place), vel=VELOCITY, acc=ACC): return False
                    
                    # 압착 (15N의 힘으로 지긋이 누름 - 필요시 파라미터 수정)
                    # if not compliant_approach(threshold_n=15.0, timeout_s=20.0): return False
                    # [✅ 수정] 그냥 누르는 대신, 새로 만든 "비비기+다짐" 모션 호출!
                    if not smart_twist_compaction(timeout_s=20.0): return False
                    
                    if not safe_movel(posx(safe_place), vel=VELOCITY, acc=ACC): return False
            
            ck = {"phase": "COMPACT_TOOL_RETURN", "tile_i": 0}

        # 5. 압착판 반납
        if ck["phase"] == "COMPACT_TOOL_RETURN":
            self._set_ckpt("COMPACT_TOOL_RETURN", 0)
            self._set_tile_status(self.STEP_IDLE, "압착판 반납 중")
            if not self.handle_compact_tool("RETURN"): 
                self._worker_err = "stopped" if self._stop_soft else "compact_tool_return_failed"
                return False
            ck = {"phase": "DONE", "tile_i": 0}

        # =========================
        # PHASE: DONE
        # =========================

        if ck["phase"] == "DONE":
            self._set_ckpt("DONE", 0)
        return True


def main(args=None):
    rclpy.init(args=args)
    cfg = RobotConfig()

    boot = rclpy.create_node("dsr_boot_tile", namespace=cfg.robot_id)
    DR_init.__dsr__id = cfg.robot_id
    DR_init.__dsr__model = cfg.robot_model
    DR_init.__dsr__node = boot

    import DSR_ROBOT2  # noqa: F401

    node = TileMotionNode(cfg, boot)

    from rclpy.executors import SingleThreadedExecutor
    ex = SingleThreadedExecutor()
    ex.add_node(node)

    try:
        while rclpy.ok():
            ex.spin_once(timeout_sec=0.1)
            node.tick()
    finally:
        try:
            ex.remove_node(node)
        except Exception:
            pass
        try:
            node.destroy_node()
            boot.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()