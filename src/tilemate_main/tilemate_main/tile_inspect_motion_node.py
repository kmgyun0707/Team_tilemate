#!/usr/bin/env python3
# tilemate_main/tile_inspect_node.py
#
# ✅ Option B: Stop -> Resume(앱 레벨 재시작)
# ✅ /tile/step
#   5: INSPECT(압착할 곳 확인)
#
# I/O
# - In : /tile/inspect/run_once (Int32 token)
# - In : /tile/inspect/resume   (Bool)
# - In : /task/pause            (Bool)
# - In : /task/stop_soft        (Bool)
# - Out: /tile/inspect/status   (String)  done:<tok> | stopped:<tok>:<ckpt> | error:<tok>:<err>
# - Out: /tile/step             (Int32)   5 during inspect
# - Out: /robot/state           (String)
#
# - Out: /robot/press           (Float32MultiArray) [tile_i, z_diff_mm, depth_mm(=0)]
# - Out: /robot/press_tile      (Int32) tile_i currently inspected

import time
import traceback
import threading
from typing import Any, Dict, Optional, List, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, Float32MultiArray, String

import DR_init
from tilemate_main.robot_config import RobotConfig

# ----------------------------
# params / thresholds
# ----------------------------
VELOCITY = 30
ACC = 30
BAD_Z_DIFF_MM = 1.5   # 불량 판정 기준
PROBE_TIMEOUT_S = 15.0

# ----------------------------
# positions (place centers)
# ----------------------------
PLACE_TILT_BASE01 = [402.08, 158.83, 229.77, 75.0, 178.60, 77.00]
PLACE_TILT_BASE02 = [469.39, 157.97, 228.80, 75.0, 178.60, 77.00]
PLACE_TILT_BASE03 = [539.79, 156.41, 229.13, 75.0, 178.60, 77.00]
PLACE_TILT_BASE04 = [401.85,  90.05, 229.55, 75.0, 178.60, 77.00]
PLACE_TILT_BASE05 = [469.94,  89.92, 228.78, 75.0, 178.60, 77.00]
PLACE_TILT_BASE06 = [539.63,  88.61, 228.16, 75.0, 178.60, 77.00]
PLACE_TILT_BASE07 = [401.19,  22.88, 228.09, 75.0, 178.60, 77.00]
PLACE_TILT_BASE08 = [468.62,  21.50, 227.98, 75.0, 178.60, 77.00]
PLACE_TILT_BASE09 = [539.05,  20.33, 227.54, 75.0, 178.60, 77.00]


class TileInspectMotionNode(Node):
    STEP_IDLE = 0
    STEP_INSPECT = 5

    def __init__(self, cfg: RobotConfig, boot_node: Node):
        super().__init__("tile_inspect_node", namespace=cfg.robot_id)
        self.cfg = cfg
        self._boot_node = boot_node

        # flags
        self._pause = False
        self._stop_soft = False

        # run token
        self._pending_token: Optional[int] = None
        self._last_token: Optional[int] = None
        self._resume_requested = False

        # checkpoint
        self._checkpoint: Optional[Dict[str, Any]] = None  # {"phase":"INSPECT","tile_i":k}
        self._stopped = False

        # results (export)
        self._needs_compaction: Dict[int, bool] = {}
        self._press_error_mm: Dict[int, float] = {}

        # worker
        self._running = False
        self._worker_done = True
        self._worker_ok = False
        self._worker_tok: Optional[int] = None
        self._worker_err = ""
        self._worker = None

 
        # pubs
        self.pub_status = self.create_publisher(String, "/tile/inspect/status", 10)
        self.pub_step   = self.create_publisher(Int32,  "/tile/step", 10)

        self.pub_state  = self.create_publisher(String, "/robot/state", 10)
        self.pub_press      = self.create_publisher(Float32MultiArray, "/robot/press", 10) # 단차 측정값 
        self.pub_press_tile = self.create_publisher(Int32, "/robot/press_tile", 10) # 단차를 측정 중인 타일번호

        # subs
        self.create_subscription(Int32, "/tile/inspect/run_once", self._cb_run_once, 10)
        self.create_subscription(Bool,  "/tile/inspect/resume",   self._cb_resume, 10)
        self.create_subscription(Bool,  "/task/pause",            self._cb_pause, 10)
        self.create_subscription(Bool,  "/task/stop_soft",        self._cb_stop_soft, 10)

        self._initialize_robot()
        self._set_status(self.STEP_IDLE, "inspect 대기중")
        self.get_logger().info("TileInspectNode ready!!!")

    # -----------------
    # init / helpers
    # -----------------
    def _initialize_robot(self):
        from DSR_ROBOT2 import set_tool, set_tcp, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS, set_robot_mode
        self.get_logger().info("[INSPECT] initialize_robot()")
        set_robot_mode(ROBOT_MODE_MANUAL)
        set_tool(self.cfg.tool)
        set_tcp(self.cfg.tcp)
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        time.sleep(0.2)

    def _set_status(self, step: int, state: str):
        m_step = Int32(); m_step.data = int(step)
        m_state = String(); m_state.data = str(state)
        self.pub_step.publish(m_step)
        self.pub_state.publish(m_state)
        self.get_logger().info(f"[INSPECT] step={m_step.data} state='{m_state.data}'")

    def _publish_status(self, s: str):
        m = String(); m.data = str(s)
        self.pub_status.publish(m)
        self.get_logger().info(f"[INSPECT->STATUS] {m.data}")

    def _pub_press_tile(self, tile_i: int):
        m = Int32(); m.data = int(tile_i)
        self.pub_press_tile.publish(m)

    def _pub_press_error(self, tile_i: int, z_diff_mm: float):
        m = Float32MultiArray()
        m.data = [float(tile_i), float(z_diff_mm), 0.0]
        self.pub_press.publish(m)

    def _wait_if_paused(self):
        if self._pause:
            self._set_status(self.STEP_IDLE, "일시정지(pause)")
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
            self.get_logger().warn("[INSPECT] stop requested (stop_soft=True)")
            return True
        self._wait_if_paused()
        return bool(self._stop_soft)

    def _set_ckpt(self, phase: str, tile_i: int):
        new_ckpt = {"phase": str(phase), "tile_i": int(tile_i)}
        if self._checkpoint == new_ckpt:
            return
        self._checkpoint = new_ckpt
        self.get_logger().info(f"[INSPECT][CKPT] set {self._checkpoint_to_string()}")

    def _checkpoint_to_string(self) -> str:
        if not self._checkpoint:
            return "none"
        return f"{self._checkpoint.get('phase','none')}:{int(self._checkpoint.get('tile_i',1))}"

    # -----------------
    # callbacks
    # -----------------
    def _cb_run_once(self, msg: Int32):
        if self._running:
            self.get_logger().warn("[INSPECT] run_once ignored (already running)")
            return
        self._pending_token = int(msg.data)
        self.get_logger().info(f"[INSPECT] received token={self._pending_token}")

    def _cb_pause(self, msg: Bool):
        self._pause = bool(msg.data)
        self.get_logger().warn(f"[INSPECT] pause={self._pause}")

    def _cb_stop_soft(self, msg: Bool):
        self._stop_soft = bool(msg.data)
        self.get_logger().warn(f"[INSPECT] stop_soft={self._stop_soft}")

    def _cb_resume(self, msg: Bool):
        if not bool(msg.data):
            return
        self.get_logger().warn("[INSPECT] resume requested (/tile/inspect/resume)")
        self._resume_requested = True

    # -----------------
    # tick / worker orchestration
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
                self._set_status(self.STEP_IDLE, "inspect 완료(done)")

            elif err == "stopped":
                ck = self._checkpoint_to_string()
                self._publish_status(f"stopped:{tok}:{ck}")
                self._stopped = True
                self._last_token = tok
                self._set_status(self.STEP_IDLE, f"inspect 중단(stopped) ckpt={ck}")

            else:
                if self._stop_soft and not err:
                    err = "aborted(stop_soft)"
                self._publish_status(f"error:{tok}:{err or 'aborted/failed'}")
                self._stopped = False
                self._checkpoint = None
                self._set_status(self.STEP_IDLE, f"inspect 에러(error) {err}")

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
                self.get_logger().warn("[INSPECT] resume pending: stop_soft=True")
                return
            self._resume_requested = False

            if not self._stopped or self._last_token is None or self._checkpoint is None:
                self.get_logger().warn(
                    f"[INSPECT] resume ignored: not stopped (stopped={self._stopped}, "
                    f"last_token={self._last_token}, ckpt={self._checkpoint_to_string()})"
                )
                return

            tok = int(self._last_token)
            ckpt = dict(self._checkpoint)
            self.get_logger().warn(f"[INSPECT] resume start tok={tok} from ckpt={self._checkpoint_to_string()}")
            self._start_worker(tok, ckpt, True)
            return

        if self._pending_token is None:
            return

        tok = int(self._pending_token)
        self._pending_token = None

        if self._stop_soft:
            self.get_logger().warn("[INSPECT] stop_soft=True -> skip token")
            return

        # reset per job
        self._checkpoint = None
        self._stopped = False
        self._last_token = tok
        self._needs_compaction.clear()
        self._press_error_mm.clear()

        self._start_worker(tok, None, False)

    def _start_worker(self, tok: int, start_ckpt: Optional[Dict[str, Any]], resume_mode: bool):
        self._running = True
        self._worker_done = False
        self._worker_ok = False
        self._worker_tok = tok
        self._worker_err = ""

        def _run():
            try:
                self._wait_if_paused()
                if self._stop_soft:
                    self._worker_err = "stopped"
                    self._worker_ok = False
                    return
                self.get_logger().info(f"[INSPECT] worker start tok={tok} resume={resume_mode}")
                ok = self._perform_cycle(start_ckpt=start_ckpt, resume_mode=resume_mode)
                self._worker_ok = bool(ok)
                if not ok and not self._worker_err:
                    self._worker_err = "stopped" if self._stop_soft else "aborted/failed"
            except Exception as e:
                if self._stop_soft:
                    self._worker_err = "stopped"
                else:
                    self.get_logger().error(f"[INSPECT] exception: {e}")
                    self.get_logger().error(traceback.format_exc())
                    self._worker_err = str(e)
                self._worker_ok = False
            finally:
                self._worker_done = True

        self._worker = threading.Thread(target=_run, daemon=True)
        self._worker.start()

    # -----------------
    # job
    # -----------------
    def _perform_cycle(self, start_ckpt: Optional[Dict[str, Any]], resume_mode: bool) -> bool:
        from DSR_ROBOT2 import (
            posx, movel, wait, get_current_posx,
            release_compliance_ctrl, release_force,
            set_ref_coord, task_compliance_ctrl, set_desired_force, check_force_condition,
            DR_FC_MOD_REL, DR_AXIS_Z, DR_BASE, DR_TOOL
        )

        def disable_compliance():
            try: release_force()
            except Exception: pass
            try: release_compliance_ctrl()
            except Exception: pass

        def safe_movel(p, **kwargs) -> bool:
            if self._check_abort():
                self._worker_err = "stopped"
                return False
            try:
                if "ref" not in kwargs:
                    kwargs["ref"] = DR_BASE
                movel(posx(list(p)), **kwargs)
                time.sleep(0.2)
                return True
            except Exception as e:
                if self._stop_soft:
                    self._worker_err = "stopped"
                else:
                    self._worker_err = f"movel failed: {e}"
                return False

        def probe_single_point(p_safe) -> Optional[float]:
            if not safe_movel(p_safe, vel=VELOCITY, acc=ACC, ref=DR_BASE):
                return None

            set_ref_coord(DR_TOOL)
            task_compliance_ctrl(stx=[3000, 3000, 20, 200, 200, 200], time=0.0)
            wait(0.2)
            set_desired_force(fd=[0, 0, 30.0, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

            t0 = time.time()
            try:
                while (time.time() - t0) < float(PROBE_TIMEOUT_S):
                    if self._check_abort():
                        return None
                    if check_force_condition(DR_AXIS_Z, min=0, max=5.0) == -1:
                        disable_compliance()
                        wait(0.2)
                        cur_pos, _ = get_current_posx(DR_BASE)
                        return float(cur_pos[2])
                    wait(0.05)
                self.get_logger().warn("[INSPECT] probe timeout")
                return None
            finally:
                disable_compliance()

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

        ck = start_ckpt or self._checkpoint or {"phase": "INSPECT_START", "tile_i": 1}

        # resume_mode면 안전 진입을 더 넣고 싶으면 여기 추가
        # (현재는 검사만 해서 생략)

        self._set_status(self.STEP_INSPECT, "타일 기울기(단차) 전수 검사 중")

        start_i = int(ck.get("tile_i", 1))
        start_i = max(1, min(9, start_i))

        for idx in range(start_i - 1, len(place_targets)):
            tile_i, center_pos = place_targets[idx]
            self._set_ckpt("INSPECT", tile_i)
            self._pub_press_tile(tile_i)

            z_safe = center_pos[2] - 40.0
            rx, ry, rz = center_pos[3], center_pos[4], center_pos[5]
            offset = 30.0
            pts = [
                [center_pos[0] + offset, center_pos[1] + offset, z_safe, rx, ry, rz],
                [center_pos[0] - offset, center_pos[1] + offset, z_safe, rx, ry, rz],
                [center_pos[0] - offset, center_pos[1] - offset, z_safe, rx, ry, rz],
            ]

            z_results: List[float] = []
            for p in pts:
                z = probe_single_point(p)
                if z is None:
                    self._worker_err = "stopped" if self._stop_soft else "probe_failed"
                    return False
                z_results.append(float(z))

            z_diff = float(max(z_results) - min(z_results))
            is_bad = z_diff >= float(BAD_Z_DIFF_MM)

            self._needs_compaction[tile_i] = bool(is_bad)
            self._press_error_mm[tile_i] = float(z_diff)
            self._pub_press_error(tile_i, z_diff)

            # 다음 타일로 넘어가기 전 잠깐
            if not self._sleep_interruptible(0.05):
                self._worker_err = "stopped"
                return False

        # 결과는 다른 노드가 활용하도록 status payload로도 힌트 제공 가능(선택)
        bad_tiles = [k for k, v in self._needs_compaction.items() if v]
        self.get_logger().info(f"[INSPECT] bad_tiles={bad_tiles}")

        return True


def main(args=None):
    rclpy.init(args=args)
    cfg = RobotConfig()

    boot = rclpy.create_node("dsr_boot_tile_inspect", namespace=cfg.robot_id)
    DR_init.__dsr__id = cfg.robot_id
    DR_init.__dsr__model = cfg.robot_model
    DR_init.__dsr__node = boot

    import DSR_ROBOT2  # noqa: F401

    node = TileInspectMotionNode(cfg, boot)

    from rclpy.executors import SingleThreadedExecutor
    ex = SingleThreadedExecutor()
    ex.add_node(node)

    try:
        while rclpy.ok():
            ex.spin_once(timeout_sec=0.1)
            node.tick()
    finally:
        try: ex.remove_node(node)
        except Exception: pass
        try:
            node.destroy_node()
            boot.destroy_node()
        except Exception:
            pass
        try: rclpy.shutdown()
        except Exception: pass


if __name__ == "__main__":
    main()