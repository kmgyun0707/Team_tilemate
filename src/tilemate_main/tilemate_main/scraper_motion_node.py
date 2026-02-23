#!/usr/bin/env python3
# tilemate_main/scraper_motion_node.py

import time
import traceback

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
    """
    Inputs:
      - /scraper/run_once (Int32 token)
      - /task/pause (Bool)
      - /task/stop_soft (Bool)

    Outputs:
      - /scraper/status (String): "done:<tok>" or "error:<tok>:<msg>"
      - /scraper/step (Int32), /scraper/state (String)
    """

    STEP_PREPARE  = 0
    STEP_GRIPPING = 1
    STEP_COATING  = 2
    STEP_FINISH   = 3

    def __init__(self, cfg: RobotConfig, boot_node: Node):
        super().__init__("scraper_motion_node", namespace=cfg.robot_id)
        self.cfg = cfg
        self._boot_node = boot_node

        self._pause = False
        self._stop_soft = False
        self._pending_token = None
        self._running = False

        # pubs
        self.pub_status = self.create_publisher(String, "/scraper/status", 10)
        self.pub_state  = self.create_publisher(String, "/robot/state", 10)
        self.pub_step   = self.create_publisher(Int32,  "/scraper/step", 10)

        # subs
        self.create_subscription(Int32, "/scraper/run_once", self._cb_run_once, 10)
        self.create_subscription(Bool,  "/task/pause", self._cb_pause, 10)
        self.create_subscription(Bool,  "/task/stop_soft", self._cb_stop_soft, 10)

        self.gripper = _GripperClient(self)

        self._initialize_robot()
        self._set_scraper_status(self.STEP_PREPARE, "대기(run_once 기다리는 중)")

        self.get_logger().info("ScraperMotionNode ready: sub /scraper/run_once")

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
        time.sleep(1.0)

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
        if self._pause:
            self._set_scraper_status(self.STEP_PREPARE, "일시정지(pause)")
        while rclpy.ok() and self._pause and not self._stop_soft:
            time.sleep(0.05)

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

    # -----------------
    # tick
    # -----------------
    def tick(self):
        if self._pending_token is None or self._running:
            return

        tok = self._pending_token
        self._pending_token = None

        if self._stop_soft:
            self.get_logger().warn("[SCRAPER] stop_soft=True -> skip token")
            return

        self._running = True
        try:
            self._wait_if_paused()
            if self._stop_soft:
                self._publish_status(f"error:{tok}:aborted(stop_soft)")
                return

            self.get_logger().info(f"[SCRAPER] run_once start token={tok}")
            ok = self._perform_cycle()

            if ok and not self._stop_soft:
                self._publish_status(f"done:{tok}")
            else:
                self._publish_status(f"error:{tok}:aborted/failed")

        except Exception as e:
            self.get_logger().error(f"[SCRAPER] exception: {e}")
            self.get_logger().error(traceback.format_exc())
            self._publish_status(f"error:{tok}:{e}")

        finally:
            self._running = False
            # if not self._stop_soft:
            #     self._set_scraper_status(self.STEP_PREPARE, "대기(run_once 기다리는 중)")

    # -----------------
    # DSR motions
    # -----------------
    def _perform_cycle(self) -> bool:
        from DSR_ROBOT2 import (
            posx,
            posj,
            movej,
            movel,
            get_current_posx,
            add_tcp,
            set_tcp,
            set_robot_mode,
            # 순응/힘제어
            release_compliance_ctrl,
            task_compliance_ctrl,
            set_desired_force,
            check_force_condition,
            DR_TOOL,
            DR_BASE,
            DR_WORLD,
            DR_FC_MOD_ABS,
            ROBOT_MODE_MANUAL,
            ROBOT_MODE_AUTONOMOUS,
        )

        def set_gripper(w: float):
            self.gripper.set_width(w)
            time.sleep(0.05)

        # ✅ 베이스 좌표계 기준 상대 좌표 이동 (사용자 수정본 유지)
        def move_relative(dx: float, dy: float, dz: float):
            cur, _ = get_current_posx(DR_BASE)
            target = [cur[0] + dx, cur[1] + dy, cur[2] + dz, cur[3], cur[4], cur[5]]
            movel(posx(target), ref=DR_BASE, vel=30, acc=30)
            time.sleep(0.5)

        # ----------------------------
        # 순응/힘제어 유틸 (안전 접촉 -> 힘 유지)
        # ----------------------------
        def enable_soft_touch_compliance(stx=(4000, 4000, 80, 200, 200, 200)):
            # 힘 없이(Z만 매우 부드럽게) 접촉 만들기용
            task_compliance_ctrl(stx=list(stx), time=0.0)

        def enable_press_force(fz=-20.0):
            # 접촉 이후 계속 누르기(압력 유지)
            fd = [0.0, 0.0, float(fz), 0.0, 0.0, 0.0]
            direction = [0, 0, 1, 0, 0, 0]  # TCP Z축 방향
            set_desired_force(fd, direction, 0, DR_FC_MOD_ABS)

        def disable_compliance():
            try:
                release_compliance_ctrl()
            except Exception:
                pass

        def approach_until_contact_world_z(
            axis=2,               # 0:x 1:y 2:z
            threshold=4.0,        # N
            max_down_mm=15.0,
            step_mm=0.5,
            timeout_sec=8.0,
        ):
            """
            ✅ 월드(DR_WORLD) 기준 Z로 상대 하강하면서
            ✅ get_tool_force()로 비블로킹 접촉 판정
            """
            import DSR_ROBOT2 as dr
            from DSR_ROBOT2 import posx, movel, get_current_posx, DR_WORLD

            if not hasattr(dr, "get_tool_force"):
                self.get_logger().warn("[TOUCH] get_tool_force() not found -> fallback: just go down w/o contact detect")
                # fallback: 그냥 max_down_mm만큼 내려가기
                cur, _ = get_current_posx(DR_WORLD)
                target = [cur[0], cur[1], cur[2] - float(max_down_mm), cur[3], cur[4], cur[5]]
                movel(posx(target), ref=DR_WORLD, vel=3, acc=3)
                return False

            t0 = time.time()
            moved = 0.0

            while moved < max_down_mm:
                if self._stop_soft:
                    return False
                self._wait_if_paused()

                if (time.time() - t0) > float(timeout_sec):
                    self.get_logger().warn("[TOUCH] timeout -> no contact detected")
                    return False

                d = min(step_mm, max_down_mm - moved)

                cur, _ = get_current_posx(DR_WORLD)
                target = [cur[0], cur[1], cur[2] - d, cur[3], cur[4], cur[5]]
                movel(posx(target), ref=DR_WORLD, vel=3, acc=3)

                moved += d
                time.sleep(0.02)

                try:
                    f = dr.get_tool_force()
                    raw = float(f[axis])
                    self.get_logger().info(f"[TOUCH] force[{axis}]={raw:.2f}N thr={threshold:.2f}N moved={moved:.1f}mm")
                    if abs(raw) >= float(threshold):
                        self.get_logger().info("[TOUCH] contact detected")
                        return True
                except Exception as e:
                    self.get_logger().warn(f"[TOUCH] force read failed: {e}")

            return False
        # ----------------------------
        # 왕복하며 펴바르기 (기존 유지)
        # ----------------------------
        def do_stroke():
            movel(posx([0, 0, 0, 0, -20, 0]), ref=DR_TOOL, time=5.0)
            time.sleep(0.2)

            move_relative(0.0, 120.0, 0.0)
            time.sleep(0.2)

            movel(posx([0, 0, 0, 0, 40, 0]), ref=DR_TOOL, time=5.0)
            time.sleep(0.2)

            move_relative(0.0, -190.0, 0.0)
            time.sleep(0.2)

            movel(posx([0, 0, 0, 0, -20, 0]), ref=DR_TOOL, time=5.0)
            time.sleep(0.2)

        # ----------------------------
        # positions
        # ----------------------------
        JReady = [0, 0, 90, 0, 90, 0]

        pre_grasp = posx([608.63, 69.05, 210.0,  52.0011, 179.0943,  52.3476])
        grasp     = posx([608.63, 69.05, 173.76, 52.0011, 179.0943,  52.3476])

        pre_mid = posx([480.8698, 91.12, 210.0, 59.9196, 179.1564, 60.5511])
        mid     = posx([480.8698, 91.12, 190.0, 59.9196, 179.1564, 60.5511])
        rotate  = posj([6.671, 16.319, 72.358, 0.566, 90.818, 95.63])

        # ----------------------------
        # 준비
        # ----------------------------
        self._set_scraper_status(self.STEP_PREPARE, "스크래퍼 파지 준비")
        movej(JReady, vel=self.cfg.vel, acc=self.cfg.acc)
        set_gripper(0.060)
        time.sleep(2.0)

        # ----------------------------
        # 파지
        # ----------------------------
        self._set_scraper_status(self.STEP_GRIPPING, "스크래퍼 파지중")
        movel(pre_grasp, vel=60, acc=60); time.sleep(1.0)
        movel(grasp, vel=60, acc=60); time.sleep(1.0)
        set_gripper(0.003); time.sleep(4.0)
        movel(pre_grasp, vel=60, acc=60); time.sleep(1.0)

        # ----------------------------
        # 이동
        # ----------------------------
        movel(pre_mid, vel=60, acc=60); time.sleep(1.0)
        movel(mid, vel=60, acc=60); time.sleep(1.0)
        movej(rotate, vel=40, acc=40); time.sleep(1.0)

        # ----------------------------
        # 도포: TCP + 안전접촉(순응만) -> 힘유지 -> 스트로크
        # ----------------------------
        try:
            # TCP 오프셋 추가
            set_robot_mode(ROBOT_MODE_MANUAL)
            tcp_name = "scraper"
            tcp_offset = [0, 0, 224.911 + 52.0, 0, 0, 0]
            add_tcp(tcp_name, tcp_offset)
            set_tcp(tcp_name)
            set_robot_mode(ROBOT_MODE_AUTONOMOUS)
            time.sleep(0.2)

            self._set_scraper_status(self.STEP_COATING, "접착제 도포중")

            # 1) ✅ 힘 없이 컴플라이언스만 켜고(충격 완화) 살살 접촉 만들기
            enable_soft_touch_compliance(stx=(4000, 4000, 80, 200, 200, 200))
            approach_until_contact_world_z(threshold=4.0, max_down_mm=15.0, step_mm=0.5)

            # 2) ✅ 접촉 후에 목표힘 ON (계속 아래로 누르면서 유지)
            enable_press_force(fz=-5.0) 
            ##############################################################
            # 튜닝 값
            ##############################################################


            # 3) 스트로크(이동 중에도 힘 유지 + Z는 순응)
            do_stroke()

            move_relative(-50.0, 80.0, 0.0)
            do_stroke()

            move_relative(100.0, 80.0, 0.0)
            do_stroke()

        finally:
            # 순응/힘 OFF
            disable_compliance()

            # TCP 원복
            try:
                set_robot_mode(ROBOT_MODE_MANUAL)
                set_tcp(self.cfg.tcp)
                set_robot_mode(ROBOT_MODE_AUTONOMOUS)
            except Exception:
                pass

        # ----------------------------
        # 반납
        # ----------------------------
        self._set_scraper_status(self.STEP_FINISH, "접착제 도포 끝")
        movel(pre_mid, vel=60, acc=60); time.sleep(1.0)
        movel(pre_grasp, vel=60, acc=60); time.sleep(1.0)
        movel(grasp, vel=60, acc=60); time.sleep(1.0)
        set_gripper(0.040); time.sleep(4.0)
        movel(pre_grasp, vel=60, acc=60); time.sleep(1.0)

        return True


def main(args=None):
    rclpy.init(args=args)
    cfg = RobotConfig()

    # ✅ DSR_ROBOT2 import 전에 boot node 먼저 만들고 DR_init.__dsr__node 세팅
    boot = rclpy.create_node("dsr_boot_scraper", namespace=cfg.robot_id)
    DR_init.__dsr__id = cfg.robot_id
    DR_init.__dsr__model = cfg.robot_model
    DR_init.__dsr__node = boot

    import DSR_ROBOT2  # noqa: F401

    node = ScraperMotionNode(cfg, boot)

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