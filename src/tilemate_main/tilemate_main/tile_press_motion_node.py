#!/usr/bin/env python3
# tilemate_main/tile_compact_node.py
#
# ✅ Option B: Stop -> Resume(앱 레벨 재시작)
# ✅ /tile/step
#   6: COMPACT(압착 시작)
#   7: DONE(압착 끝/전체 종료)  (※ 이 노드는 압착만 담당하므로 DONE은 "압착 종료" 의미)
#
# I/O
# - In : /tile/compact/run_once (Int32 token)
# - In : /tile/compact/resume   (Bool)
# - In : /task/pause            (Bool)
# - In : /task/stop_soft        (Bool)
# - Out: /tile/compact/status   (String)  done:<tok> | stopped:<tok>:<ckpt> | error:<tok>:<err>
# - Out: /tile/step             (Int32)   6 during compact, 7 when compact finished
# - Out: /robot/state           (String)
#
# - In : /robot/press (Float32MultiArray) [tile_i, z_diff_mm, depth_mm] from inspect
#        -> z_diff_mm >= threshold이면 압착 대상
#
# - Out: /robot/press           (Float32MultiArray) [tile_i, z_diff_mm, depth_mm] (depth_mm 업데이트)
# - Out: /robot/press_tile      (Int32) tile_i currently compacting target
# - Out: /robot/pressing        (Int32) tile_i currently pressing (0=idle)

import time
import traceback
import threading
from typing import Any, Dict, Optional, List, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, Float32MultiArray, String, Float64

import DR_init
from tilemate_main.robot_config import RobotConfig

# ----------------------------
# params
# ----------------------------
VELOCITY = 30
ACC = 30

BAD_Z_DIFF_MM = 1.5  # inspect와 동일 기준(구독한 값으로 판단)
COMPACT_CLOSE_W = 0.005

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

COMPACT_TOOL_ABOVE = [531.2, -101.3 - 87, 220.0, 169.29, 177.87, 169.98]
COMPACT_TOOL_DOWN  = [531.2, -101.3 - 87, 146.0, 169.29, 177.87, 169.98]
COMPACT_TOOL_WAYPOINT = [470.0, 24.0, 230.0, 6.0, -179.0, 97.0]


class _GripperClient:
    def __init__(self, node: Node):
        self._node = node
        self._pub = node.create_publisher(Float64, "/gripper/width_m", 10)

    def set_width(self, width_m: float):
        msg = Float64()
        msg.data = float(width_m)
        self._pub.publish(msg)
        self._node.get_logger().info(f"[GRIPPER->CMD] width_m={msg.data:.4f}")

    def release(self):
        self._node.get_logger().info("[GRIPPER] release")
        self.set_width(0.040)
        time.sleep(0.5)


class TilePressMotionNode(Node):
    STEP_IDLE = 0
    STEP_COMPACT = 6
    STEP_DONE = 7

    def __init__(self, cfg: RobotConfig, boot_node: Node):
        super().__init__("tile_compact_node", namespace=cfg.robot_id)
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
        self._checkpoint: Optional[Dict[str, Any]] = None  # {"phase":"COMPACT","tile_i":k}
        self._stopped = False

        # targets (from /robot/press)
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
        self.pub_status = self.create_publisher(String, "/tile/compact/status", 10)
        self.pub_step   = self.create_publisher(Int32,  "/tile/step", 10)
        self.pub_state  = self.create_publisher(String, "/robot/state", 10)

        self.pub_press      = self.create_publisher(Float32MultiArray, "/robot/press", 10)
        self.pub_press_tile = self.create_publisher(Int32, "/robot/press_tile", 10)
        self.pub_pressing   = self.create_publisher(Int32, "/robot/pressing", 10)

        # subs
        self.create_subscription(Int32, "/tile/compact/run_once", self._cb_run_once, 10)
        self.create_subscription(Bool,  "/tile/compact/resume",   self._cb_resume, 10)
        self.create_subscription(Bool,  "/task/pause",            self._cb_pause, 10)
        self.create_subscription(Bool,  "/task/stop_soft",        self._cb_stop_soft, 10)

        # ✅ inspect 결과 구독 (/robot/press): [tile_i, z_diff, depth]
        self.create_subscription(Float32MultiArray, "/robot/press", self._cb_press, 10)

        self.gripper = _GripperClient(self)

        self._initialize_robot()
        self._set_status(self.STEP_IDLE, "compact 대기중")
        self._pub_pressing(0)
        self.get_logger().info("TileCompactNode ready!!!")

    # -----------------
    # init / helpers
    # -----------------
    def _initialize_robot(self):
        from DSR_ROBOT2 import set_tool, set_tcp, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS, set_robot_mode
        self.get_logger().info("[COMPACT] initialize_robot()")
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
        self.get_logger().info(f"[COMPACT] step={m_step.data} state='{m_state.data}'")

    def _publish_status(self, s: str):
        m = String(); m.data = str(s)
        self.pub_status.publish(m)
        self.get_logger().info(f"[COMPACT->STATUS] {m.data}")

    def _pub_press_tile(self, tile_i: int):
        m = Int32(); m.data = int(tile_i)
        self.pub_press_tile.publish(m)

    def _pub_pressing(self, tile_i: int):
        m = Int32(); m.data = int(tile_i)
        self.pub_pressing.publish(m)

    def _pub_press_update(self, tile_i: int, z_diff_mm: float, depth_mm: float):
        m = Float32MultiArray()
        m.data = [float(tile_i), float(z_diff_mm), float(depth_mm)]
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
            self.get_logger().warn("[COMPACT] stop requested (stop_soft=True)")
            return True
        self._wait_if_paused()
        return bool(self._stop_soft)

    def _set_ckpt(self, phase: str, tile_i: int):
        new_ckpt = {"phase": str(phase), "tile_i": int(tile_i)}
        if self._checkpoint == new_ckpt:
            return
        self._checkpoint = new_ckpt
        self.get_logger().info(f"[COMPACT][CKPT] set {self._checkpoint_to_string()}")

    def _checkpoint_to_string(self) -> str:
        if not self._checkpoint:
            return "none"
        return f"{self._checkpoint.get('phase','none')}:{int(self._checkpoint.get('tile_i',1))}"

    # -----------------
    # callbacks
    # -----------------
    def _cb_press(self, msg: Float32MultiArray):
        # 기대 포맷: [tile_i, z_diff_mm, depth_mm]
        try:
            if not msg.data or len(msg.data) < 2:
                return
            tile_i = int(round(float(msg.data[0])))
            z_diff = float(msg.data[1])
            if tile_i < 1 or tile_i > 9:
                return
            self._press_error_mm[tile_i] = z_diff
            self._needs_compaction[tile_i] = (z_diff >= float(BAD_Z_DIFF_MM))
        except Exception:
            return

    def _cb_run_once(self, msg: Int32):
        if self._running:
            self.get_logger().warn("[COMPACT] run_once ignored (already running)")
            return
        self._pending_token = int(msg.data)
        self.get_logger().info(f"[COMPACT] received token={self._pending_token}")

    def _cb_pause(self, msg: Bool):
        self._pause = bool(msg.data)
        self.get_logger().warn(f"[COMPACT] pause={self._pause}")

    def _cb_stop_soft(self, msg: Bool):
        self._stop_soft = bool(msg.data)
        self.get_logger().warn(f"[COMPACT] stop_soft={self._stop_soft}")

    def _cb_resume(self, msg: Bool):
        if not bool(msg.data):
            return
        self.get_logger().warn("[COMPACT] resume requested (/tile/compact/resume)")
        self._resume_requested = True

    # -----------------
    # tool handling
    # -----------------
    def handle_compact_tool(self, action: str) -> bool:
        from DSR_ROBOT2 import movel, wait, posx, DR_BASE
        action = str(action).upper().strip()
        if action not in ("GRAB", "RETURN"):
            self.get_logger().error(f"[COMPACT_TOOL] invalid action={action}")
            return False

        if self._check_abort():
            self._worker_err = "stopped"
            return False

        try:
            movel(posx(list(COMPACT_TOOL_WAYPOINT)), vel=VELOCITY, acc=ACC, ref=DR_BASE)
            if not self._sleep_interruptible(0.2): return False

            movel(posx(list(COMPACT_TOOL_ABOVE)), vel=VELOCITY, acc=ACC, ref=DR_BASE)
            if not self._sleep_interruptible(0.2): return False

            if action == "GRAB":
                self.gripper.release()
                movel(posx(list(COMPACT_TOOL_DOWN)), vel=VELOCITY, acc=ACC, ref=DR_BASE)
                wait(0.5)
                self.gripper.set_width(COMPACT_CLOSE_W)
                time.sleep(0.6)
                movel(posx(list(COMPACT_TOOL_ABOVE)), vel=VELOCITY, acc=ACC, ref=DR_BASE)
            else:
                movel(posx(list(COMPACT_TOOL_DOWN)), vel=VELOCITY, acc=ACC, ref=DR_BASE)
                wait(0.5)
                self.gripper.release()
                movel(posx(list(COMPACT_TOOL_ABOVE)), vel=VELOCITY, acc=ACC, ref=DR_BASE)

            movel(posx(list(COMPACT_TOOL_WAYPOINT)), vel=VELOCITY, acc=ACC, ref=DR_BASE)
            if not self._sleep_interruptible(0.2): return False
            return True

        except Exception as e:
            if self._stop_soft:
                self._worker_err = "stopped"
                return False
            self._worker_err = f"compact_tool_failed:{e}"
            return False

    # -----------------
    # tick / worker orchestration
    # -----------------
    def tick(self):
        if self._running and self._worker_done:
            tok = self._worker_tok
            ok = self._worker_ok
            err = self._worker_err

            self._pub_pressing(0)

            if ok and not self._stop_soft:
                self._publish_status(f"done:{tok}")
                self._stopped = False
                self._checkpoint = None
                self._set_status(self.STEP_DONE, "압착 종료(done)")

            elif err == "stopped":
                ck = self._checkpoint_to_string()
                self._publish_status(f"stopped:{tok}:{ck}")
                self._stopped = True
                self._last_token = tok
                self._set_status(self.STEP_IDLE, f"압착 중단(stopped) ckpt={ck}")

            else:
                if self._stop_soft and not err:
                    err = "aborted(stop_soft)"
                self._publish_status(f"error:{tok}:{err or 'aborted/failed'}")
                self._stopped = False
                self._checkpoint = None
                self._set_status(self.STEP_IDLE, f"압착 에러(error) {err}")

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
                self.get_logger().warn("[COMPACT] resume pending: stop_soft=True")
                return
            self._resume_requested = False

            if not self._stopped or self._last_token is None or self._checkpoint is None:
                self.get_logger().warn(
                    f"[COMPACT] resume ignored: not stopped (stopped={self._stopped}, "
                    f"last_token={self._last_token}, ckpt={self._checkpoint_to_string()})"
                )
                return

            tok = int(self._last_token)
            ckpt = dict(self._checkpoint)
            self.get_logger().warn(f"[COMPACT] resume start tok={tok} from ckpt={self._checkpoint_to_string()}")
            self._start_worker(tok, ckpt, True)
            return

        if self._pending_token is None:
            return

        tok = int(self._pending_token)
        self._pending_token = None

        if self._stop_soft:
            self.get_logger().warn("[COMPACT] stop_soft=True -> skip token")
            return

        self._checkpoint = None
        self._stopped = False
        self._last_token = tok
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
                self.get_logger().info(f"[COMPACT] worker start tok={tok} resume={resume_mode}")
                ok = self._perform_cycle(start_ckpt=start_ckpt, resume_mode=resume_mode)
                self._worker_ok = bool(ok)
                if not ok and not self._worker_err:
                    self._worker_err = "stopped" if self._stop_soft else "aborted/failed"
            except Exception as e:
                if self._stop_soft:
                    self._worker_err = "stopped"
                else:
                    self.get_logger().error(f"[COMPACT] exception: {e}")
                    self.get_logger().error(traceback.format_exc())
                    self._worker_err = str(e)
                self._worker_ok = False
            finally:
                self._pub_pressing(0)
                self._worker_done = True

        self._worker = threading.Thread(target=_run, daemon=True)
        self._worker.start()

    # -----------------
    # job
    # -----------------
    def _perform_cycle(self, start_ckpt: Optional[Dict[str, Any]], resume_mode: bool) -> bool:
        from DSR_ROBOT2 import (
            posx, movel, wait, set_ref_coord, task_compliance_ctrl, set_desired_force,
            check_force_condition, release_force, release_compliance_ctrl,
            DR_FC_MOD_REL, DR_AXIS_Z, DR_BASE, DR_TOOL, get_current_posx, get_current_posj, amovej
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

        def smart_twist_compaction(timeout_s=20.0) -> Tuple[bool, float]:
            self._wait_if_paused()
            if self._stop_soft:
                return (False, 0.0)

            set_ref_coord(DR_TOOL)
            task_compliance_ctrl(stx=[3000, 3000, 20, 200, 200, 200], time=0.0)
            wait(0.2)
            set_desired_force(fd=[0, 0, 30.0, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

            t0 = time.time()
            touched = False
            contact_z = 0.0
            contact_joint = None

            try:
                while (time.time() - t0) < float(timeout_s):
                    self._wait_if_paused()
                    if self._stop_soft:
                        return (False, 0.0)

                    if check_force_condition(DR_AXIS_Z, min=0, max=6.0) == -1:
                        touched = True
                        contact_pos, _ = get_current_posx(DR_BASE)
                        contact_z = float(contact_pos[2])
                        contact_joint = get_current_posj()
                        break
                    wait(0.05)

                if not touched or contact_joint is None:
                    return (False, 0.0)

                # 비비기(조인트6 왕복)
                set_desired_force(fd=[0, 0, 10.0, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
                press_t0 = time.time()
                direction_flag = 1
                last_switch = time.time()

                while (time.time() - press_t0) < 3.0:
                    if self._check_abort():
                        return (False, 0.0)
                    if time.time() - last_switch > 0.5:
                        direction_flag *= -1
                        last_switch = time.time()

                    target_joint = list(contact_joint)
                    target_joint[5] = contact_joint[5] + (10.0 * direction_flag)
                    amovej(target_joint, vel=80, acc=80)
                    wait(0.1)

                amovej(contact_joint, vel=40, acc=40)
                if not self._sleep_interruptible(1.0):
                    return (False, 0.0)

                final_pos, _ = get_current_posx(DR_BASE)
                final_z = float(final_pos[2])
                depth = float(contact_z - final_z)
                return (True, depth)

            finally:
                disable_compliance()
                wait(0.1)

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

        ck = start_ckpt or self._checkpoint or {"phase": "COMPACT_TOOL_GRAB", "tile_i": 1}

        # ✅ 압착 시작 step=6
        self._set_status(self.STEP_COMPACT, "불량 타일 스마트 압착 진행")

        # 1) tool grab
        if ck.get("phase") == "COMPACT_TOOL_GRAB":
            self._set_ckpt("COMPACT_TOOL_GRAB", 0)
            if not self.handle_compact_tool("GRAB"):
                return False
            ck = {"phase": "COMPACT", "tile_i": 1}
            self._checkpoint = ck

        # 2) compact loop
        if ck.get("phase") == "COMPACT":
            start_i = int(ck.get("tile_i", 1))
            start_i = max(1, min(9, start_i))

            self._pub_pressing(0)

            for idx in range(start_i - 1, len(place_targets)):
                tile_i, place_pos = place_targets[idx]
                if not self._needs_compaction.get(tile_i, False):
                    continue

                self._set_ckpt("COMPACT", tile_i)
                self._pub_press_tile(tile_i)
                self._pub_pressing(tile_i)

                safe_place = list(place_pos)
                safe_place[2] -= 35.0
                if not safe_movel(safe_place, vel=VELOCITY, acc=ACC, ref=DR_BASE):
                    return False

                ok, depth_mm = smart_twist_compaction(timeout_s=20.0)
                if not ok:
                    self._worker_err = "stopped" if self._stop_soft else "press_failed"
                    return False

                z_diff = float(self._press_error_mm.get(tile_i, 0.0))
                self._pub_press_update(tile_i, z_diff, float(depth_mm))

                self._pub_pressing(0)

                if not safe_movel(safe_place, vel=VELOCITY, acc=ACC, ref=DR_BASE):
                    return False

                if self._check_abort():
                    self._worker_err = "stopped"
                    return False

            ck = {"phase": "COMPACT_TOOL_RETURN", "tile_i": 0}
            self._checkpoint = ck

        # 3) tool return
        if ck.get("phase") == "COMPACT_TOOL_RETURN":
            self._set_ckpt("COMPACT_TOOL_RETURN", 0)
            if not self.handle_compact_tool("RETURN"):
                return False
            ck = {"phase": "DONE", "tile_i": 0}
            self._checkpoint = ck

        # 4) done
        if ck.get("phase") == "DONE":
            self._set_ckpt("DONE", 0)
            self._pub_pressing(0)
            self._set_status(self.STEP_DONE, "압착 완료/전체 종료")
            return True

        self._worker_err = f"invalid_phase:{ck.get('phase')}"
        return False


def main(args=None):
    rclpy.init(args=args)
    cfg = RobotConfig()

    boot = rclpy.create_node("dsr_boot_tile_compact", namespace=cfg.robot_id)
    DR_init.__dsr__id = cfg.robot_id
    DR_init.__dsr__model = cfg.robot_model
    DR_init.__dsr__node = boot

    import DSR_ROBOT2  # noqa: F401

    node = TilePressMotionNode(cfg, boot)

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