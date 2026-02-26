#!/usr/bin/env python3
# tilemate_main/scraper_motion_node.py
#
# ✅ Option B: "Stop(진짜 취소/정지) -> Resume(앱 레벨 재시작)"
# - stop_soft 들어오면: MoveStop(외부 interrupt_node에서)로 로봇을 멈추고,
#   scraper_motion_node는 error로 끝내지 않고 "stopped:<tok>:<checkpoint>"로 종료
# - resume 들어오면: 컨트롤러 move_resume 호출 ❌ (실패 가능성 큼)
#   대신 /scraper/resume 토픽으로 재개 요청 -> 저장된 checkpoint부터 남은 시퀀스를 "재실행(restart from checkpoint)"
#
# ✅ 이 버전의 핵심:
# - resume 시: 현재 자세/회전 상태를 신뢰하지 않고
#   1) compliance 해제
#   2) z lift
#   3) JReady(Home) 정렬
#   4) COAT/RETURN 상태면 MOVE로 강제 롤백 후 다시 정석 루트로 진입
#
# ⚠️ 중요:
# - stop_soft=True가 계속 유지되면 재개가 불가능하므로, resume 전에 stop_soft는 False로 풀려야 함.
# - interrupt_node에서는 stop 때 MoveStop, resume 때 /scraper/resume=True publish 하는 구조 권장.

import time
import traceback
import threading
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, Float64, String

import DR_init
from tilemate_main.robot_config import RobotConfig


class _GripperClient:
    def __init__(self, node: Node):
        self._node = node
        self._pub = node.create_publisher(Float64, "/gripper/width_m", 10)

    def set_width(self, width_m: float):
        msg = Float64()
        msg.data = float(width_m)
        self._pub.publish(msg)
        self._node.get_logger().info(f"[GRIPPER->CMD] width_m={msg.data:.4f}")


class ScraperMotionNode(Node):
    STEP_PREPARE  = 0
    STEP_GRIPPING = 1
    STEP_COATING  = 2
    STEP_FINISH   = 3

    # 그립 폭:
    W_OPEN    = 0.060
    W_CLOSE   = 0.003
    W_RELEASE = 0.040

    def __init__(self, cfg: RobotConfig, boot_node: Node):
        super().__init__("scraper_motion_node", namespace=cfg.robot_id)
        self.cfg = cfg
        self._boot_node = boot_node

        # flags
        self._pause = False
        self._stop_soft = False

        # run token (job)
        self._pending_token: Optional[int] = None
        self._last_token: Optional[int] = None  # stopped 후 resume용

        # resume request (Option B)
        self._resume_requested = False

        # checkpoint (Option B)
        # 예: {"phase":"COAT","coat_i":2} = 도포 루프에서 2번째 세트까지 완료(다음은 2부터 시작)
        self._checkpoint: Optional[Dict[str, Any]] = None
        self._stopped = False  # stopped 상태인지

        # worker state
        self._running = False
        self._worker = None
        self._worker_done = True
        self._worker_ok = False
        self._worker_tok: Optional[int] = None
        self._worker_err = ""   # "stopped" | error string

        # pubs
        self.pub_status = self.create_publisher(String, "/scraper/status", 10)
        self.pub_state  = self.create_publisher(String, "/robot/state", 10)
        self.pub_step   = self.create_publisher(Int32,  "/scraper/step", 10)

        # subs
        self.create_subscription(Int32, "/scraper/run_once", self._cb_run_once, 10)
        self.create_subscription(Bool,  "/scraper/resume", self._cb_resume, 10)

        self.create_subscription(Bool,  "/task/pause", self._cb_pause, 10)
        self.create_subscription(Bool,  "/task/stop_soft", self._cb_stop_soft, 10)

        self.gripper = _GripperClient(self)

        self._initialize_robot()
        self._set_scraper_status(self.STEP_PREPARE, "작업명령 대기중")
        self.get_logger().info("ScraperMotionNode ready!!!")

    # -----------------
    # init / helpers
    # -----------------
    def _initialize_robot(self):
        from DSR_ROBOT2 import set_tool, set_tcp, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS, set_robot_mode
        self.get_logger().info("[SCRAPER] initialize_robot()")
        set_robot_mode(ROBOT_MODE_MANUAL)
        set_tool(self.cfg.tool)
        set_tcp(self.cfg.tcp)
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        time.sleep(0.5)

    def _set_scraper_status(self, step: int, state: str):
        m_step = Int32()
        m_step.data = int(step)
        m_state = String()
        m_state.data = str(state)
        self.pub_step.publish(m_step)
        self.pub_state.publish(m_state)
        self.get_logger().info(f"[SCRAPER] step={m_step.data} state='{m_state.data}'")

    def _publish_status(self, s: str):
        m = String()
        m.data = s
        self.pub_status.publish(m)
        self.get_logger().info(f"[SCRAPER->STATUS] {m.data}")

    def _wait_if_paused(self):
        # Pause(일시정지, pause) = 앱 레벨 sleep gate
        if self._pause:
            self._set_scraper_status(self.STEP_PREPARE, "일시정지(pause)")
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
        # stop_soft(취소/정지) 체크
        if self._stop_soft:
            self.get_logger().warn("[SCRAPER] stop requested (stop_soft=True)")
            return True
        self._wait_if_paused()
        return bool(self._stop_soft)

    # ---- checkpoint helpers ----
    def _set_ckpt(self, phase: str, coat_i: int = 0):
        self._checkpoint = {"phase": str(phase), "coat_i": int(coat_i)}
        self.get_logger().info(f"[SCRAPER][CKPT] set {self._checkpoint_to_string()}")

    def _checkpoint_to_string(self) -> str:
        if not self._checkpoint:
            return "none"
        phase = self._checkpoint.get("phase", "none")
        coat_i = int(self._checkpoint.get("coat_i", 0))
        return f"{phase}:{coat_i}"

    # -----------------
    # callbacks
    # -----------------
    def _cb_run_once(self, msg: Int32):
        if self._running:
            self.get_logger().warn("[SCRAPER] run_once ignored (already running)")
            return
        self._pending_token = int(msg.data)
        self.get_logger().info(f"[SCRAPER] received token={self._pending_token}")

    def _cb_pause(self, msg: Bool):
        self._pause = bool(msg.data)
        self.get_logger().warn(f"[SCRAPER] pause={self._pause}")

    def _cb_stop_soft(self, msg: Bool):
        self._stop_soft = bool(msg.data)
        self.get_logger().warn(f"[SCRAPER] stop_soft={self._stop_soft}")

    def _cb_resume(self, msg: Bool):
        if not bool(msg.data):  
            return
        self.get_logger().warn("[SCRAPER] resume requested (/scraper/resume)")

        # ✅ 재개 요청이면 pause도 같이 해제 (resume implies unpause)
        if self._pause:
            self._pause = False
            self.get_logger().warn("[SCRAPER] auto-unpause on resume")

        self._resume_requested = True

    # -----------------
    # tick (worker orchestration)
    # -----------------
    def tick(self):
        # 0) 워커 종료 처리
        if self._running and self._worker_done:
            tok = self._worker_tok
            ok = self._worker_ok
            err = self._worker_err

            # 정상 완료
            if ok and not self._stop_soft:
                self._publish_status(f"done:{tok}")
                self._stopped = False
                self._checkpoint = None

            # ✅ stopped (Option B)
            elif err == "stopped":
                ck = self._checkpoint_to_string()
                self._publish_status(f"stopped:{tok}:{ck}")
                self._stopped = True
                self._last_token = tok  # resume용 토큰 보관

            # 진짜 에러
            else:
                if self._stop_soft and not err:
                    err = "aborted(stop_soft)"
                self._publish_status(f"error:{tok}:{err or 'aborted/failed'}")
                self._stopped = False
                self._checkpoint = None

            # running state reset
            self._running = False
            self._worker = None
            self._worker_tok = None
            self._worker_done = True
            self._worker_ok = False
            self._worker_err = ""
            return

        # 1) 워커가 돌고 있으면 아무 것도 안 함
        if self._running:
            return

        # 2) ✅ resume 요청 처리
        if self._resume_requested:
            if self._stop_soft:
                self.get_logger().warn("[SCRAPER] resume pending: stop_soft=True (waiting stop_soft False)")
                return  # ✅ 플래그 유지

            self._resume_requested = False

            if self._stop_soft:
                self.get_logger().warn("[SCRAPER] resume ignored: stop_soft=True (set stop_soft False first)")
                return

            if not self._stopped or self._last_token is None or self._checkpoint is None:
                self.get_logger().warn(
                    f"[SCRAPER] resume ignored: not in stopped state (stopped={self._stopped}, "
                    f"last_token={self._last_token}, ckpt={self._checkpoint_to_string()})"
                )
                return

            tok = int(self._last_token)
            ckpt = dict(self._checkpoint)

            self.get_logger().warn(f"[SCRAPER] resume start tok={tok} from ckpt={self._checkpoint_to_string()}")
            self._start_worker(tok=tok, start_ckpt=ckpt, resume_mode=True)
            return

        # 3) 새 작업 시작 (run_once)
        if self._pending_token is None:
            return

        tok = int(self._pending_token)
        self._pending_token = None

        if self._stop_soft:
            self.get_logger().warn("[SCRAPER] stop_soft=True -> skip token")
            return

        # 새 작업이면 checkpoint 초기화
        self._checkpoint = None
        self._stopped = False
        self._last_token = tok

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

                self.get_logger().info(f"[SCRAPER] worker start token={tok} resume_mode={resume_mode}")
                ok = self._perform_cycle(start_ckpt=start_ckpt, resume_mode=resume_mode)
                self._worker_ok = bool(ok)

                if not ok and not self._worker_err:
                    if self._stop_soft:
                        self._worker_err = "stopped"
                    else:
                        self._worker_err = "aborted/failed"

            except Exception as e:
                if self._stop_soft:
                    self.get_logger().warn(f"[SCRAPER] exception during stop -> treat as stopped: {e}")
                    self._worker_ok = False
                    self._worker_err = "stopped"
                else:
                    self.get_logger().error(f"[SCRAPER] exception in worker: {e}")
                    self.get_logger().error(traceback.format_exc())
                    self._worker_ok = False
                    self._worker_err = str(e)

            finally:
                self._worker_done = True

        self._worker = threading.Thread(target=_run_worker, daemon=True)
        self._worker.start()
        
    # -----------------
    # DSR motions (runs in worker thread)
    # -----------------
    def _perform_cycle(self, start_ckpt: Optional[Dict[str, Any]], resume_mode: bool) -> bool:
        from DSR_ROBOT2 import (
            posx, posj, movej, movel, get_current_posx,
            add_tcp, set_tcp, set_robot_mode, set_tool,
            release_compliance_ctrl, task_compliance_ctrl, set_desired_force,
            DR_TOOL, DR_BASE, DR_WORLD, DR_FC_MOD_ABS,
            ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS,
        )

        # ---- wrappers ----
        def safe_movej(*args, **kwargs) -> bool:
            if self._check_abort():
                self._worker_err = "stopped"
                return False
            try:
                movej(*args, **kwargs)

                # ✅ MoveStop으로 인해 조기 리턴된 뒤, 다음 모션 보내기 전에 재확인
                if self._stop_soft:
                    self.get_logger().warn("[SAFE_MOVEJ] stopped after movej return")
                    self._worker_err = "stopped"
                    return False

                return True
            except Exception as e:
                if self._stop_soft:
                    self._worker_err = "stopped"
                    return False
                self._worker_err = f"movej failed: {e}"
                return False


        def safe_movel(*args, **kwargs) -> bool:
            if self._check_abort():
                self._worker_err = "stopped"
                return False
            try:
                movel(*args, **kwargs)

                # ✅ MoveStop으로 인해 조기 리턴된 뒤, 다음 모션 보내기 전에 재확인
                if self._stop_soft:
                    self.get_logger().warn("[SAFE_MOVEL] stopped after movel return")
                    self._worker_err = "stopped"
                    return False

                return True
            except Exception as e:
                if self._stop_soft:
                    self._worker_err = "stopped"
                    return False
                self._worker_err = f"movel failed: {e}"
                return False

        def set_gripper(w: float) -> bool:
            if self._check_abort():
                self._worker_err = "stopped"
                return False
            self.gripper.set_width(w)
            return self._sleep_interruptible(0.05)

        def move_relative(dx: float, dy: float, dz: float) -> bool:
            if self._check_abort():
                self._worker_err = "stopped"
                return False
            cur, _ = get_current_posx(DR_BASE)
            target = [cur[0] + dx, cur[1] + dy, cur[2] + dz, cur[3], cur[4], cur[5]]
            if not safe_movel(posx(target), ref=DR_BASE, vel=30, acc=30):
                return False
            return self._sleep_interruptible(0.2)

        def enable_soft_touch_compliance(stx=(4000, 4000, 80, 200, 200, 200)):
            task_compliance_ctrl(stx=list(stx), time=0.0)

        def enable_press_force(fz=-20.0):
            fd = [0.0, 0.0, float(fz), 0.0, 0.0, 0.0]
            direction = [0, 0, 1, 0, 0, 0]
            set_desired_force(fd, direction, 0, DR_FC_MOD_ABS)

        def disable_compliance():
            try:
                release_compliance_ctrl()
            except Exception:
                pass

        def rearm_tool_tcp(tool: str, tcp: str) -> bool:
            # re-arm (재무장): stop 이후 controller 상태를 신뢰하지 말고 tool/tcp/mode를 확정
            try:
                set_robot_mode(ROBOT_MODE_MANUAL)
                set_tool(tool)
                set_tcp(tcp)
                set_robot_mode(ROBOT_MODE_AUTONOMOUS)
                return self._sleep_interruptible(0.3)
            except Exception as e:
                if self._stop_soft:
                    self._worker_err = "stopped"
                else:
                    self._worker_err = f"rearm failed: {e}"
                return False

        def home_align_with_lift(jready, lift_mm: float = 30.0) -> bool:
            # home align (홈 정렬): compliance 해제 + z lift + JReady
            disable_compliance()

            if self._check_abort():
                self._worker_err = "stopped"
                return False

            # world lift
            try:
                cur, _ = get_current_posx(DR_WORLD)
                lift = [cur[0], cur[1], cur[2] + float(lift_mm), cur[3], cur[4], cur[5]]
                safe_movel(posx(lift), ref=DR_WORLD, vel=10, acc=10)
                if not self._sleep_interruptible(0.2):
                    self._worker_err = "stopped"
                    return False
            except Exception:
                # lift 실패해도 home 시도는 하되, stop이면 종료
                if self._stop_soft:
                    self._worker_err = "stopped"
                    return False

            if not safe_movej(jready, vel=20, acc=20):
                return False
            return self._sleep_interruptible(0.2)

        # ---- contact helper (필요 시 유지) ----
        
        def compliant_approach_down(threshold_n: float = 6.0, timeout_s: float = 8.0) -> bool:
            from DSR_ROBOT2 import (
                set_ref_coord, task_compliance_ctrl, set_desired_force,
                check_force_condition, release_force, release_compliance_ctrl,
                DR_BASE, DR_FC_MOD_REL, DR_AXIS_Z, wait
            )

            self._wait_if_paused()
            if self._stop_soft:
                return False

            # 시작 전에 잔류 제거 (여긴 OK)
            try:
                release_force()
            except Exception:
                pass
            try:
                release_compliance_ctrl()
            except Exception:
                pass
            wait(0.2)

            self.get_logger().info(f"[COMPLIANT] start (thr={threshold_n}N, timeout={timeout_s}s)")

            set_ref_coord(DR_BASE)
            task_compliance_ctrl(stx=[3000, 3000, 50, 200, 200, 200], time=0.0)
            wait(0.3)

            set_desired_force(
                fd=[0, 0, float(-(threshold_n+15)), 0, 0, 0],
                dir=[0, 0, 1, 0, 0, 0],
                mod=DR_FC_MOD_REL
            )

            t0 = time.time()
            while True:
                self._wait_if_paused()
                if self._stop_soft:
                    # stop이면 정리하고 나감
                    try: release_force()
                    except Exception: pass
                    try: release_compliance_ctrl()
                    except Exception: pass
                    wait(0.2)
                    return False

                if timeout_s is not None and (time.time() - t0) > float(timeout_s):
                    self.get_logger().warn("[COMPLIANT] timeout -> fail")
                    # 실패면 정리하고 나감
                    try: release_force()
                    except Exception: pass
                    try: release_compliance_ctrl()
                    except Exception: pass
                    wait(0.2)
                    return False

                ret = check_force_condition(DR_AXIS_Z, min=0, max=float(threshold_n))
                if ret == -1:
                    self.get_logger().info(f"[COMPLIANT] reached threshold={threshold_n}N")
                    # ✅ 성공 시에는 release하지 않고 그대로 반환 (COAT 동안 유지)
                    return True

                wait(0.1)
        def do_stroke() -> bool:
            # 한 번 stroke 시퀀스 (툴 기준 상대 회전 포함)
            if self._check_abort():
                self._worker_err = "stopped"
                return False

            if not safe_movel(posx([0, 0, 0, 0, -20, 0]), ref=DR_TOOL, time=5.0):
                return False
            if not self._sleep_interruptible(0.2): return False

            if not move_relative(0.0, 80.0, 0.0): return False
            if not self._sleep_interruptible(0.2): return False

            if not safe_movel(posx([0, 0, 0, 0, 40, 0]), ref=DR_TOOL, time=5.0):
                return False
            if not self._sleep_interruptible(0.2): return False

            if not move_relative(0.0, -150.0, 0.0): return False
            if not self._sleep_interruptible(0.2): return False

            if not safe_movel(posx([0, 0, 0, 0, -20, 0]), ref=DR_TOOL, time=5.0):
                return False
            if not self._sleep_interruptible(0.2): return False

            return True

        # ---- positions ----
        JReady = [0, 0, 90, 0, 90, 0]
        pre_grasp = posx([608.63, 69.05, 210.0,  52.0011, 179.0943,  52.3476])
        grasp     = posx([608.63, 69.05, 173.76, 52.0011, 179.0943,  52.3476])

        pre_mid = posx([480.8698, 91.12, 210.0, 59.9196, 179.1564, 60.5511])
        mid     = posx([480.8698, 91.12, 190.0, 59.9196, 179.1564, 60.5511])
        rotate  = posj([6.671, 16.319, 72.358, 0.566, 90.818, 95.63])

        # ---- checkpoint init ----
        ck = start_ckpt or self._checkpoint or {"phase": "PREPARE", "coat_i": 0}

        # ✅ resume_mode: 홈 정렬(home align) 후 안정 재진입
        if resume_mode:
            self._set_scraper_status(
                self.STEP_PREPARE,
                f"재개(resume) 홈정렬중 ckpt={ck.get('phase')}:{ck.get('coat_i', 0)}"
            )

            # 1) 기본 tool/tcp 확정 (re-arm)
            if not rearm_tool_tcp(self.cfg.tool, self.cfg.tcp):
                return False

            # 2) lift + home align
            if not home_align_with_lift(JReady, lift_mm=30.0):
                return False

            # 3) COAT/RETURN에서 멈춘 경우: MOVE로 롤백해서 회전/접근을 다시 정석 루트로
            #    (회전 두 번/미회전/자세 꼬임 방지)
            if ck.get("phase") in ("COAT", "RETURN"):
                self.get_logger().warn("[SCRAPER][RESUME] rollback phase to MOVE for stable re-entry")
                ck["phase"] = "MOVE"
                # coat_i는 유지: 이미 완료한 세트 다음부터 진행
                if "coat_i" not in ck:
                    ck["coat_i"] = 0

        # =========================
        # PHASE: PREPARE
        # =========================
        if ck["phase"] == "PREPARE":
            self._set_scraper_status(self.STEP_PREPARE, "스크래퍼 파지 준비")
            if not safe_movej(JReady, vel=self.cfg.vel, acc=self.cfg.acc):
                return False
            if not set_gripper(self.W_OPEN): return False
            if not self._sleep_interruptible(2.0): return False

            self._set_ckpt("GRIP", 0)
            ck = {"phase": "GRIP", "coat_i": 0}

        # =========================
        # PHASE: GRIP
        # =========================
        if ck["phase"] == "GRIP":
            self._set_scraper_status(self.STEP_GRIPPING, "스크래퍼 파지중")
            if not safe_movel(pre_grasp, vel=60, acc=60): return False
            if not self._sleep_interruptible(1.0): return False
            if not safe_movel(grasp, vel=60, acc=60): return False
            if not self._sleep_interruptible(1.0): return False
            if not set_gripper(self.W_CLOSE): return False
            if not self._sleep_interruptible(4.0): return False
            if not safe_movel(pre_grasp, vel=60, acc=60): return False
            if not self._sleep_interruptible(1.0): return False

            self._set_ckpt("MOVE", 0)
            ck = {"phase": "MOVE", "coat_i": 0}

        # =========================
        # PHASE: MOVE (to coating area)
        # =========================
        if ck["phase"] == "MOVE":
            # NOTE: resume로 MOVE에 들어오면 rotate를 다시 수행 -> 자세/회전 상태가 항상 동일해짐
            self._set_scraper_status(self.STEP_COATING, "도포 위치로 이동중")  # 또는 별도 STEP_MOVE를 추가

            if not safe_movel(pre_mid, vel=60, acc=60): return False
            if not self._sleep_interruptible(1.0): return False
            if not safe_movel(mid, vel=60, acc=60): return False
            if not self._sleep_interruptible(1.0): return False
            if not safe_movej(rotate, vel=40, acc=40): return False
            if not self._sleep_interruptible(1.0): return False

            # coat_i는 resume에서 유지될 수 있으니 그대로 전달
            coat_i = int(ck.get("coat_i", 0))
            self._set_ckpt("COAT", coat_i)
            ck = {"phase": "COAT", "coat_i": coat_i}

        # =========================
        # PHASE: COAT (checkpoint-able)
        # =========================
        if ck["phase"] == "COAT":
            try:
                # tcp 바꾸고, compliance/force 설정 후 도포
                set_robot_mode(ROBOT_MODE_MANUAL)
                tcp_name = "scraper"
                tcp_offset = [0, 0, 224.911 + 52.0, 0, 0, 0]
                try:
                    add_tcp(tcp_name, tcp_offset)
                except Exception as e:
                    self.get_logger().warn(f"[SCRAPER] add_tcp warn (maybe exists): {e}")

                # ✅ 중요: set_tcp 실패하면 COAT 진행 금지
                try:
                    set_tcp(tcp_name)
                except Exception as e:
                    self._worker_err = f"set_tcp({tcp_name}) failed: {e}"
                    return False

                set_robot_mode(ROBOT_MODE_AUTONOMOUS)
                if not self._sleep_interruptible(0.2): return False

                self._set_scraper_status(self.STEP_COATING, "접착제 도포중")

                enable_soft_touch_compliance(stx=(4000, 4000, 80, 200, 200, 200))
                ok = compliant_approach_down(threshold_n=4.0, timeout_s=8.0)
                if not ok:
                    self._worker_err = "no_contact"
                    return False
                
                if self._check_abort():
                    self._worker_err = "stopped"
                    return False

                enable_press_force(fz=-5.0)

                # 도포 3세트
                start_i = int(ck.get("coat_i", 0))
                if start_i < 0: start_i = 0
                if start_i > 3: start_i = 3

                for i in range(start_i, 3):
                    self.get_logger().info(f"[SCRAPER][COAT] set {i+1}/3 start")

                    if not do_stroke():
                        return False

                    # i번째 세트 완료 -> checkpoint advance
                    self._set_ckpt("COAT", i + 1)

                    # 세트 사이 이동
                    if i == 0:
                        if not move_relative(-50.0, 80.0, 0.0): return False
                    elif i == 1:
                        if not move_relative(100.0, 80.0, 0.0): return False

                # 도포 완료
                self._set_ckpt("RETURN", 3)
                ck = {"phase": "RETURN", "coat_i": 3}

            finally:
                disable_compliance()
                try:
                    set_robot_mode(ROBOT_MODE_MANUAL)
                    set_tcp(self.cfg.tcp)
                    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
                except Exception:
                    pass

        # =========================
        # PHASE: RETURN (put back)
        # =========================
        if ck["phase"] == "RETURN":
            self._set_scraper_status(self.STEP_FINISH, "접착제 도포 끝")
            if not safe_movel(pre_mid, vel=60, acc=60): return False
            if not self._sleep_interruptible(1.0): return False
            if not safe_movel(pre_grasp, vel=60, acc=60): return False
            if not self._sleep_interruptible(1.0): return False
            if not safe_movel(grasp, vel=60, acc=60): return False
            if not self._sleep_interruptible(1.0): return False
            if not set_gripper(self.W_RELEASE): return False
            if not self._sleep_interruptible(4.0): return False
            if not safe_movel(pre_grasp, vel=60, acc=60): return False
            if not self._sleep_interruptible(1.0): return False

            self._set_ckpt("DONE", 0)
            return True

        # DONE
        if ck["phase"] == "DONE":
            return True

        self._worker_err = f"unknown checkpoint phase: {ck.get('phase')}"
        return False


def main(args=None):
    rclpy.init(args=args)
    cfg = RobotConfig()

    # boot node (DSR 내부에서 spin 돌린다는 전제 유지)
    boot = rclpy.create_node("dsr_boot_scraper", namespace=cfg.robot_id)
    DR_init.__dsr__id = cfg.robot_id
    DR_init.__dsr__model = cfg.robot_model
    DR_init.__dsr__node = boot

    import DSR_ROBOT2  # noqa: F401

    node = ScraperMotionNode(cfg, boot)

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