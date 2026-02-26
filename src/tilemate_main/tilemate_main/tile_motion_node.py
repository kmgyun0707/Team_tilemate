#!/usr/bin/env python3
# tilemate_main/tile_motion_node.py
#
# ✅ Option B: Stop -> Resume(앱 레벨 재시작)
# ✅ 이 노드는 "타일 배치(픽/플레이스 + 흡착툴 반납)"까지만 담당
#    - 단차측정(INSPECT) / 압착(COMPACT)은 분리된 노드가 담당하고,
#      TaskManager가 순서를 오케스트레이션함.
#
# /tile/step (TaskManager 매핑용)
#   0: IDLE
#   3: TILE_PICK
#   4: TILE_PLACE
#
# /tile/status
#   done:<token> / stopped:<token>:<ckpt> / error:<token>:<msg>

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


class TileMotionNode(Node):
    # ✅ /tile/step 정의 (TaskManager가 그대로 매핑)
    STEP_IDLE  = 0
    STEP_PICK  = 3
    STEP_PLACE = 4

    def __init__(self, cfg: RobotConfig, boot_node: Node):
        super().__init__("tile_motion_node", namespace=cfg.robot_id)
        self.cfg = cfg
        self._boot_node = boot_node

        self._pause = False
        self._stop_soft = False

        self._pending_token: Optional[int] = None
        self._last_token: Optional[int] = None
        self._resume_requested = False

        self._checkpoint: Optional[Dict[str, Any]] = None
        self._stopped = False

        self._running = False
        self._worker = None
        self._worker_done = True
        self._worker_ok = False
        self._worker_tok: Optional[int] = None
        self._worker_err = ""

        # pubs
        self.pub_status = self.create_publisher(String, "/tile/status", 10)
        self.pub_step   = self.create_publisher(Int32,  "/tile/step", 10)
        self.pub_state  = self.create_publisher(String, "/robot/state", 10)
        self.pub_completed_jobs = self.create_publisher(Int32, "/robot/completed_jobs", 10)

        # subs
        self.create_subscription(Int32, "/tile/run_once", self._cb_run_once, 10)
        self.create_subscription(Bool,  "/tile/resume",   self._cb_resume, 10)
        self.create_subscription(Bool,  "/task/pause",    self._cb_pause, 10)
        self.create_subscription(Bool,  "/task/stop_soft", self._cb_stop_soft, 10)
        self.create_subscription(String, "/robot/design_ab", self._cb_design_ab, 10)

        self._design_pattern = ["A"] * 9
        self.gripper = _GripperClient(self)

        self._initialize_robot()
        self._set_tile_status(self.STEP_IDLE, "작업명령 대기중")
        self.get_logger().info("TileMotionNode(PLACE only) ready!!!")

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
        m_step = Int32(); m_step.data = int(step)
        m_state = String(); m_state.data = str(state)
        self.pub_step.publish(m_step)
        self.pub_state.publish(m_state)
        self.get_logger().info(f"[TILE] step={m_step.data} state='{m_state.data}'")

    def _publish_status(self, s: str):
        m = String(); m.data = s
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

    def _set_ckpt(self, phase: str, tile_i: int):
        new_ckpt = {"phase": str(phase), "tile_i": int(tile_i)}
        if self._checkpoint == new_ckpt:
            return
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

    # -----------------
    # suction tool return
    # -----------------
    def return_tool(self) -> bool:
        from DSR_ROBOT2 import movel, wait, posx, DR_BASE, movej
        self.get_logger().info("[TILE] 배치 완료. 흡착 툴을 거치대에 반납합니다...")

        if self._check_abort():
            self.get_logger().warn("[TILE][RETURN_TOOL] aborted before start")
            return False

        try:
            movel(posx(list(TOOL_WAYPOINT)), vel=VELOCITY, acc=ACC, ref=DR_BASE)
            if not self._sleep_interruptible(0.2): return False

            movel(posx(list(TOOL_GRIP_ABOVE)), vel=VELOCITY, acc=ACC, ref=DR_BASE)
            if not self._sleep_interruptible(0.2): return False

            movel(posx(list(TOOL_GRIP_DOWN)), vel=VELOCITY, acc=ACC, ref=DR_BASE)
            wait(0.5)

            self.gripper.release()

            movel(posx(list(TOOL_GRIP_ABOVE)), vel=VELOCITY, acc=ACC, ref=DR_BASE)
            if not self._sleep_interruptible(0.2): return False

            JReady = [0, 0, 90, 0, 90, 90]
            self.get_logger().info("✅ [TILE] 툴 반납 완료 -> 홈(JReady) 복귀")
            movej(JReady, vel=20, acc=20)
            if not self._sleep_interruptible(0.2): return False

            self.get_logger().info("✅ [TILE] 흡착 툴 반납 + 홈 복귀 완료!")
            return True

        except Exception as e:
            if self._stop_soft:
                self.get_logger().warn(f"[TILE][RETURN_TOOL] exception during stop -> treat as stopped: {e}")
                return False
            self.get_logger().error(f"[TILE][RETURN_TOOL] failed: {e}")
            return False

    # -----------------
    # tick / worker
    # -----------------
    def tick(self):
        if self._running and self._worker_done:
            tok = self._worker_tok
            ok = self._worker_ok
            err = self._worker_err

            if ok and not self._stop_soft:
                self._publish_status(f"done:{tok}")
                self._stopped = False
                self._checkpoint = None
                # ⚠️ 여기서 /tile/step=7 같은 "전체 완료"를 찍지 않음.
                # (inspect/compact가 별도 노드이므로 TaskManager가 이어서 수행)
                self._set_tile_status(self.STEP_IDLE, "배치 완료(done) - 다음 단계 대기")

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
                self._set_tile_status(self.STEP_IDLE, "에러/중단")

            self._running = False
            self._worker = None
            self._worker_tok = None
            self._worker_done = True
            self._worker_ok = False
            self._worker_err = ""
            return

        if self._running:
            return

        if self._resume_requested:
            if self._stop_soft:
                self.get_logger().warn("[TILE] resume pending: stop_soft=True (waiting stop_soft False)")
                return
            self._resume_requested = False

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
    # main cycle (PLACE ONLY)
    # -----------------
    def _perform_cycle(self, start_ckpt: Optional[Dict[str, Any]], resume_mode: bool) -> bool:
        from DSR_ROBOT2 import (
            posx, movej, movel, wait, get_current_posx,
            release_compliance_ctrl, release_force,
            set_robot_mode, set_tool, set_tcp,
            DR_BASE, DR_TOOL, DR_WORLD,
            ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS,
        )

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
                    kwargs["ref"] = DR_BASE
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
                try:
                    release_force()
                except Exception:
                    pass
                try:
                    release_compliance_ctrl()
                except Exception:
                    pass
                wait(0.1)

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
                if tile_idx == 3:
                    tilt_angle = -30
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

        ck = start_ckpt or self._checkpoint or {"phase": "PREPARE", "tile_i": 1}

        # resume safe entry
        if resume_mode:
            self._set_tile_status(self.STEP_IDLE, f"재개(resume) 홈정렬중 ckpt={ck.get('phase')}:{ck.get('tile_i')}")
            if not rearm_tool_tcp(self.cfg.tool, self.cfg.tcp):
                return False
            if not home_align_with_lift(JReady, lift_mm=30.0):
                return False

            ph0 = str(ck.get("phase", "PREPARE"))
            if ph0 in ("PREPARE", "JREADY") or ph0.startswith("TOOL_"):
                ck["phase"] = "PREPARE"
                ck["tile_i"] = 1
            elif ph0 in ("PLACE", "DETACH"):
                ck["phase"] = "PICK"

        # --------------------------
        # PREPARE (흡착툴 파지)
        # --------------------------
        if ck["phase"] == "PREPARE":
            self._set_ckpt("PREPARE", int(ck.get("tile_i", 1)))
            self._set_tile_status(self.STEP_IDLE, "JReady 이동 및 도구 파지")

            self._set_ckpt("JREADY", int(ck.get("tile_i", 1)))
            if not safe_movej(JReady, vel=VELOCITY, acc=ACC):
                return False

            self._set_ckpt("TOOL_RELEASE", 1)
            self.gripper.release()

            self._set_ckpt("TOOL_APPROACH_ABOVE", 1)
            if not safe_movel(posx(TOOL_GRIP_ABOVE), vel=VELOCITY, acc=ACC):
                return False

            self._set_ckpt("TOOL_APPROACH_DOWN", 1)
            if not safe_movel(posx(TOOL_GRIP_DOWN), vel=VELOCITY, acc=ACC):
                return False

            self._set_ckpt("TOOL_GRAB", 1)
            self.gripper.grab()

            self._set_ckpt("TOOL_LIFT", 1)
            if not safe_movel(posx(TOOL_GRIP_ABOVE), vel=VELOCITY, acc=ACC):
                return False

            if not move_relative(0, 100, 0):
                return False

            self._set_tile_status(self.STEP_IDLE, "안전구역(Waypoint) 이동")
            self._set_ckpt("TOOL_WAYPOINT", 1)
            if not safe_movel(posx(TOOL_WAYPOINT), vel=VELOCITY, acc=ACC):
                return False

            next_tile_i = int(ck.get("tile_i", 1))
            self._set_ckpt("PICK", next_tile_i)
            ck = {"phase": "PICK", "tile_i": next_tile_i}

        # --------------------------
        # TILE LOOP (PICK/PLACE)
        # --------------------------
        ph = str(ck.get("phase", "PREPARE"))
        if ph not in ("PICK", "PLACE", "DETACH", "DONE"):
            self._worker_err = f"invalid_phase_before_tile_loop:{ph}"
            return False

        start_tile_i = int(ck.get("tile_i", 1))
        start_tile_i = max(1, min(9, start_tile_i))

        for idx in range(start_tile_i - 1, len(place_targets)):
            tile_i, place_pos = place_targets[idx]

            list_index = tile_i - 1
            tile_type = self._design_pattern[list_index] if list_index < len(self._design_pattern) else "A"
            if tile_type == "B":
                pick_pos = PICK_ABOVE_B
                color_name = "흰색"
            else:
                pick_pos = PICK_ABOVE_A
                color_name = "검정"

            self._set_ckpt("PICK", tile_i)
            if self._check_abort():
                self._worker_err = "stopped"
                return False

            self._set_tile_status(self.STEP_PICK, f"타일 파지 준비({color_name}) - {tile_i}번")
            if not safe_movel(posx(pick_pos), vel=VELOCITY, acc=ACC):
                return False

            self._set_tile_status(self.STEP_PICK, f"타일 파지 하강 - {tile_i}번")
            if not compliant_approach(threshold_n=13.0, timeout_s=10.0):
                self._worker_err = "stopped" if self._stop_soft else "pick_compliant_failed"
                self._sleep_interruptible(0.5)
                return False

            self._set_tile_status(self.STEP_PICK, f"타일 파지 상승 - {tile_i}번")
            if not safe_movel(posx(pick_pos), vel=VELOCITY, acc=ACC):
                return False
            if not move_relative(0, 100, 0):
                return False

            self._set_ckpt("PLACE", tile_i)
            if self._check_abort():
                self._worker_err = "stopped"
                return False

            self._set_tile_status(self.STEP_PLACE, f"타일 배치 상부 - {tile_i}번")
            if not safe_movel(posx(place_pos), vel=VELOCITY, acc=ACC):
                return False

            self._set_tile_status(self.STEP_PLACE, f"타일 배치 하강 - {tile_i}번")
            if not compliant_approach(threshold_n=11.0, timeout_s=10.0):
                self._worker_err = "stopped" if self._stop_soft else "place_compliant_failed"
                return False

            self._set_ckpt("DETACH", tile_i)
            if self._check_abort():
                self._worker_err = "stopped"
                return False

            self._set_tile_status(self.STEP_PLACE, f"타일 박리(detach) - {tile_i}번")
            if not detach_tile(tile_i):
                return False

            self._set_tile_status(self.STEP_PLACE, f"타일 배치 상부 복귀 - {tile_i}번")
            if not safe_movel(posx(place_pos), vel=VELOCITY, acc=ACC):
                return False

            self.get_logger().info(f"🎉 {tile_i}번 타일 완료")
            m = Int32(); m.data = int(tile_i)
            self.pub_completed_jobs.publish(m)

            if tile_i < 9:
                self._set_ckpt("PICK", tile_i + 1)

        # --------------------------
        # RETURN SUCTION TOOL (끝)
        # --------------------------
        self._set_ckpt("RETURN_SUCTION_TOOL", 0)
        self._set_tile_status(self.STEP_PLACE, "타일 배치 완료 -> 흡착툴 반납")
        if not self.return_tool():
            self._worker_err = "stopped" if self._stop_soft else "return_tool_failed"
            return False

        self._set_ckpt("DONE", 0)
        self._set_tile_status(self.STEP_IDLE, "배치 완료(DONE) - inspect/compact는 외부 노드가 수행")
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