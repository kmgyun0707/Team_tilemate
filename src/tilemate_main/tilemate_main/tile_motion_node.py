#!/usr/bin/env python3
# tilemate_main/tile_motion_node.py

import time
import traceback

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

OPEN_W  = 0.025
CLOSE_W = 0.005

# ----------------------------
# positions (absolute posx list)
# ----------------------------
pick_above = [344, -101, 300, 92, 179, 92]

# pick sources (A/B)
TILE_PICK_A = [345.35345458984375, -95.80809020996094, 175.74713134765625,
               92, 179, 92]
TILE_POSE_B = [436.9603576660156, -94.84788513183594, 172.90957641601562,
               92, 179, 92]

# place targets (1..9)
PLACE_1 = [396.5, 160.5, 200, 92, 179, 92]
PLACE_2 = [461.5, 160.5, 200, 92, 179, 92]
PLACE_3 = [526.5, 160.5, 200, 92, 179, 92]
PLACE_4 = [396.5, 95.5, 200, 92, 179, 92]
PLACE_5 = [461.5, 95.5, 200, 92, 179, 92]
PLACE_6 = [526.5, 95.5, 200, 92, 179, 92]
PLACE_7 = [396.5, 30.5, 200, 92, 179, 92]
PLACE_8 = [461.5, 30.5, 200, 92, 179, 92]
PLACE_9 = [526.5, 30.5, 200, 92, 179, 92]


class _GripperClient:
    def __init__(self, node: Node):
        self._node = node
        self._pub = node.create_publisher(Float64, "/gripper/width_m", 10)

    def set_width(self, width_m: float):
        msg = Float64()
        msg.data = float(width_m)
        self._pub.publish(msg)
        self._node.get_logger().info(f"[GRIPPER->CMD] width_m={msg.data:.4f}")


class TileMotionNode(Node):
    def __init__(self, cfg: RobotConfig, boot_node: Node):
        super().__init__("tile_motion_node", namespace=cfg.robot_id)
        self.cfg = cfg
        self._boot_node = boot_node

        self._pause = False
        self._stop_soft = False
        self._pending_token = None
        self._running = False

        # pubs
        self.pub_status = self.create_publisher(String, "/tile/status", 10)
        self.pub_state = self.create_publisher(String, "/robot/state", 10)
        self.pub_step = self.create_publisher(Int32, "/robot/step", 10)
        self.pub_completed_jobs = self.create_publisher(Int32, "/robot/completed_jobs", 10)
        self._completed_jobs = 0

        # subs
        self.create_subscription(Int32, "/tile/run_once", self._cb_run_once, 10)
        self.create_subscription(Bool,  "/task/pause", self._cb_pause, 10)
        self.create_subscription(Bool,  "/task/stop_soft", self._cb_stop_soft, 10)

        self.gripper = _GripperClient(self)

        self._initialize_robot()
        self.get_logger().info("TileMotionNode ready: sub /tile/run_once")

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
        time.sleep(1.0)

    def _set_robot_status(self, step: int, state: str):
        m_step = Int32()
        m_step.data = int(step)
        m_state = String()
        m_state.data = str(state)
        self.pub_step.publish(m_step)
        self.pub_state.publish(m_state)
        self.get_logger().info(f"[STATUS] step={m_step.data} state='{m_state.data}'")

    def _publish_status(self, s: str):
        m = String()
        m.data = s
        self.pub_status.publish(m)
        self.get_logger().info(f"[TILE->STATUS] {m.data}")

    def _wait_if_paused(self):
        while rclpy.ok() and self._pause and not self._stop_soft:
            time.sleep(0.05)

    # -----------------
    # callbacks
    # -----------------
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

    # -----------------
    # main tick
    # -----------------
    def tick(self):
        if self._pending_token is None or self._running:
            return

        tok = self._pending_token
        self._pending_token = None

        if self._stop_soft:
            self.get_logger().warn("[TILE] stop_soft=True -> skip token")
            return

        self._running = True
        try:
            self._wait_if_paused()
            if self._stop_soft:
                self._publish_status(f"error:{tok}:aborted(stop_soft)")
                return

            self.get_logger().info(f"[TILE] run_once start token={tok}")

            ok = self._perform_task_pick_place_sequence_pick_contact_only()

            if ok and not self._stop_soft:
                self._completed_jobs += 1
                m = Int32()
                m.data = int(self._completed_jobs)
                self.pub_completed_jobs.publish(m)
                self._set_robot_status(5, "타일 작업 완료")
                self._publish_status(f"done:{tok}")
            else:
                self._publish_status(f"error:{tok}:aborted/failed")

        except Exception as e:
            self.get_logger().error(f"[TILE] exception: {e}")
            self.get_logger().error(traceback.format_exc())
            self._publish_status(f"error:{tok}:{e}")

        finally:
            self._running = False

    # ============================================================
    # Pick 전용: "쭉" 하강하면서 raw force 출력/판정
    # 판정되면 10mm 상승(move_relative)만 수행
    # ============================================================
    def down_until_contact_then_up(
        self,
        max_down_mm: float = 200.0,
        axis: int = 2,
        trigger_mode: str = "abs",   # "abs" | "gt" | "lt"
        threshold_n: float = 15.0,
        vel_down: float = 8.0,
        acc_down: float = 8.0,
        poll_dt: float = 0.01,
        settle_sec: float = 0.3,
        need_true: int = 3,
        lift_up_mm: float = 10.0,
    ) -> bool:
        import DSR_ROBOT2 as dr
        from DSR_ROBOT2 import (
            posx, movel, wait, get_current_posx,
            set_ref_coord, task_compliance_ctrl, release_compliance_ctrl,
            amovel, check_motion,
            DR_BASE
        )

        def move_relative(dx: float, dy: float, dz: float):
            cur, _ = get_current_posx(DR_BASE)
            target = [cur[0] + dx, cur[1] + dy, cur[2] + dz, cur[3], cur[4], cur[5]]
            movel(posx(target), ref=DR_BASE, vel=10, acc=10)
            time.sleep(0.2)

        # force getter
        if not hasattr(dr, "get_tool_force"):
            self.get_logger().error("[FORCE] get_tool_force() not found in this DSR_ROBOT2")
            return False
        force_fn = dr.get_tool_force
        self.get_logger().info("[FORCE] using get_tool_force() (RAW)")

        self._wait_if_paused()
        if self._stop_soft:
            return False

        # compliance ON
        set_ref_coord(DR_BASE)
        task_compliance_ctrl(stx=[3000, 3000, 80, 200, 200, 200], time=0.0)
        wait(float(settle_sec))

        # descend async
        cur0, _ = get_current_posx(DR_BASE)
        target = [cur0[0], cur0[1], cur0[2] - float(max_down_mm), cur0[3], cur0[4], cur0[5]]
        amovel(posx(target), ref=DR_BASE, vel=vel_down, acc=acc_down)

        def is_trigger(raw_val: float) -> bool:
            if trigger_mode == "abs":
                return abs(raw_val) >= float(threshold_n)
            if trigger_mode == "gt":
                return raw_val >= float(threshold_n)
            if trigger_mode == "lt":
                return raw_val <= float(threshold_n)
            return False

        hit = False
        true_count = 0

        try:
            while True:
                if self._stop_soft:
                    break
                self._wait_if_paused()

                try:
                    f = force_fn()
                    fx, fy, fz = float(f[0]), float(f[1]), float(f[2])
                    raw_val = float(f[axis])
                    print(f"[FORCE RAW] Fx={fx:.2f} Fy={fy:.2f} Fz={fz:.2f} -> chk={raw_val:.2f}")

                    if is_trigger(raw_val):
                        true_count += 1
                        if true_count >= int(need_true):
                            hit = True
                            self.get_logger().info(
                                f"[CONTACT RAW] mode={trigger_mode} axis={axis} raw={raw_val:.2f} thr={threshold_n:.2f}"
                            )
                            break
                    else:
                        true_count = 0
                except Exception as e:
                    print("[FORCE RAW] read fail:", e)

                try:
                    if check_motion() == 0:
                        self.get_logger().warn("[CONTACT RAW] motion finished before trigger")
                        break
                except Exception:
                    pass

                dr.wait(float(poll_dt))

        finally:
            try:
                release_compliance_ctrl()
            except Exception:
                pass
            wait(0.05)

        # lift up a bit if hit
        if hit and (not self._stop_soft):
            move_relative(0.0, 0.0, float(lift_up_mm))

        return hit

    # ============================================================
    # Pick&Place "이동만" + Pick에서만 접촉 하강 포함
    # ============================================================
    def pick_and_place_move_only_with_pick_contact(self, pick_pos: list, place_pos: list, label: str = "") -> None:
        from DSR_ROBOT2 import movel, posx, DR_BASE, wait

        self._wait_if_paused()
        if self._stop_soft:
            return

        # 1) PICK 이동
        self.get_logger().info(f"[P&P] {label} -> move to PICK")
        movel(posx(pick_pos), ref=DR_BASE, vel=VELOCITY, acc=ACC)
        wait(0.2)

        # 2) PICK: 하강(쭉) + 힘 감지되면 상승
        self.get_logger().info(f"[P&P] {label} -> pick contact descend")
        hit = self.down_until_contact_then_up(
            max_down_mm=80.0,        # ★ 먼저 짧게 테스트 추천 (필요하면 200으로)
            trigger_mode="abs",
            threshold_n=15.0,        # ★ raw 임계치 튜닝
            vel_down=8.0,
            acc_down=8.0,
            settle_sec=0.2,
            need_true=3,
            poll_dt=0.01,
            lift_up_mm=40.0,
        )
        self.get_logger().info(f"[P&P] {label} -> pick_contact hit={hit}")

        self._wait_if_paused()
        if self._stop_soft:
            return

        # 3) PLACE 이동 (현재는 이동만)
        self.get_logger().info(f"[P&P] {label} -> move to PLACE (move only)")
        movel(posx(place_pos), ref=DR_BASE, vel=VELOCITY, acc=ACC)
        wait(0.2)

    # ============================================================
    # A1, B2, A3, B4... 시퀀스
    # ============================================================
    def make_sequence_A1_B2(self, places: list) -> list:
        pickA = TILE_PICK_A
        pickB = TILE_POSE_B
        seq = []
        for i, place in enumerate(places, start=1):
            pick = pickA if (i % 2 == 1) else pickB
            label = ("A" if (i % 2 == 1) else "B") + str(i)
            seq.append((pick, place, label))
        return seq

    # ============================================================
    # demo task
    # ============================================================
    def _perform_task_pick_place_sequence_pick_contact_only(self) -> bool:
        from DSR_ROBOT2 import movej, movel, posx, DR_BASE, wait

        JReady = [0, 0, 90, 0, 90, 90]
        self._set_robot_status(0, "JReady 이동")
        movej(JReady, vel=VELOCITY, acc=ACC)
        wait(0.2)

        # self._set_robot_status(1, "시작 위치 이동(pick_above)")
  
        places = [PLACE_1, PLACE_2, PLACE_3, PLACE_4, PLACE_5, PLACE_6, PLACE_7, PLACE_8, PLACE_9]
        seq = self.make_sequence_A1_B2(places)

        self._set_robot_status(2, "A1,B2,A3... (Pick=접촉하강, Place=이동만)")
        for pick_pos, place_pos, label in seq:
            if self._stop_soft:
                self.get_logger().warn("[TILE] stop_soft -> abort sequence")
                return False
            self._wait_if_paused()

            self.get_logger().info(f"[SEQ] {label} start")
            self.pick_and_place_move_only_with_pick_contact(pick_pos, place_pos, label=label)
            self.get_logger().info(f"[SEQ] {label} done")

        self._set_robot_status(3, "시퀀스 종료")
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

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.tick()
    finally:
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