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
            if not self._stop_soft:
                self._set_scraper_status(self.STEP_PREPARE, "대기(run_once 기다리는 중)")

    # -----------------
    # DSR motions
    # -----------------
    def _perform_cycle(self) -> bool:
        from DSR_ROBOT2 import (
            posx,
            posj,
            movej,
            movel,
            amovej,
            amovel,
            wait,
            get_current_posx,
            add_tcp,
            get_tcp,
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
            ROBOT_MODE_MANUAL,
            ROBOT_MODE_AUTONOMOUS,
        )

        def set_gripper(w: float):
            self.gripper.set_width(w)
            time.sleep(0.05)

        # 베이스 좌표계 기준 상대 좌표 이동
        def move_relative(dx: float, dy: float, dz: float):
            cur, _ = get_current_posx(DR_BASE)   # cur: [x,y,z,rx,ry,rz]
            target = [cur[0] + dx, cur[1] + dy, cur[2] + dz, cur[3], cur[4], cur[5]]
            movel(posx(target), ref=DR_BASE, vel=30, acc=30)
            time.sleep(0.5)

        # ----------------------------
        # 순응(Compliance) + 힘제어(Force) 유틸
        # ----------------------------
        def enable_coating_compliance(
            stx=(3000, 3000, 200, 200, 200, 200),  # X,Y 강 / Z 약 (예시)
            fz=-15.0,                              # 툴 Z방향 누르는 힘 (부호는 환경에 맞게 튜닝)
            ref=DR_TOOL,
        ):
            # 1) 순응 ON (Z만 부드럽게)
            task_compliance_ctrl(stx=list(stx), time=0.0)

            # 2) 목표 힘 ON (Z만)
            # fd: [Fx,Fy,Fz,Mx,My,Mz], dir: [x,y,z,rx,ry,rz] (1이면 적용)
            fd = [0.0, 0.0, float(fz), 0.0, 0.0, 0.0]
            direction = [0, 0, 1, 0, 0, 0]
            set_desired_force(fd=fd, dir=direction, ref=ref)

        def disable_coating_compliance():
            try:
                release_compliance_ctrl()
            except Exception:
                pass

        def approach_until_contact(
            axis=2,             # check_force_condition 규약이 환경마다 다름 (Z=2 가정)
            threshold=10.0,     # N
            max_down_mm=10.0,
            step_mm=0.5,
        ):
            """
            (선택) 접촉을 확실히 만든 뒤 스트로크 시작하고 싶을 때.
            check_force_condition() 인자 규약이 다르면 TypeError 날 수 있어 try로 감쌈.
            """
            moved = 0.0
            while moved < max_down_mm:
                movel(posx([0, 0, -step_mm, 0, 0, 0]), ref=DR_TOOL, vel=5, acc=5)
                moved += step_mm
                time.sleep(0.05)

                try:
                    if check_force_condition(axis=axis, min=threshold):
                        return True
                except TypeError:
                    try:
                        if check_force_condition(axis, threshold):
                            return True
                    except Exception:
                        pass

            return True

        # ----------------------------
        # 왕복하며 펴바르기
        # ----------------------------
        def do_stroke():
            movel(posx([0, 0, 0, 0, -20, 0]), ref=DR_TOOL, time=5.0)
            time.sleep(0.2)

            move_relative(0.0, 120.0, 0.0)   # +Y 120
            time.sleep(0.2)

            movel(posx([0, 0, 0, 0, 40, 0]), ref=DR_TOOL, time=5.0)
            time.sleep(0.2)

            move_relative(0.0, -190.0, 0.0)  # -Y 190
            time.sleep(0.2)

            movel(posx([0, 0, 0, 0, -20, 0]), ref=DR_TOOL, time=5.0)
            time.sleep(0.2)

        JReady = [0, 0, 90, 0, 90, 0]

        pre_grasp = posx([608.63, 69.05, 210.0,  52.0011, 179.0943,  52.3476])
        grasp     = posx([608.63, 69.05, 173.76, 52.0011, 179.0943,  52.3476])

        pre_mid = posx([480.8698, 91.12, 210.0, 59.9196, 179.1564, 60.5511])
        mid     = posx([480.8698, 91.12, 190.0, 59.9196, 179.1564, 60.5511])
        rotate  = posj([6.671, 16.319, 72.358, 0.566, 90.818, 95.63])

        # 준비
        self._set_scraper_status(self.STEP_PREPARE, "스크래퍼 파지 준비")
        movej(JReady, vel=self.cfg.vel, acc=self.cfg.acc)
        set_gripper(0.060)
        time.sleep(2.0)

        # 파지
        self._set_scraper_status(self.STEP_GRIPPING, "스크래퍼 파지중")
        movel(pre_grasp, vel=60, acc=60)
        time.sleep(1.0)
        movel(grasp, vel=60, acc=60)
        time.sleep(1.0)
        set_gripper(0.003)
        time.sleep(4.0)
        movel(pre_grasp, vel=60, acc=60)
        time.sleep(1.0)

        # 이동
        movel(pre_mid, vel=60, acc=60)
        time.sleep(1.0)
        movel(mid, vel=60, acc=60)
        time.sleep(1.0)
        movej(rotate, vel=40, acc=40)
        time.sleep(1.0)

        # ----------------------------
        # 도포: TCP 오프셋 추가 + 순응(Compliance) 추가
        # ----------------------------
        try:
            # TCP 오프셋 추가
            set_robot_mode(ROBOT_MODE_MANUAL)      # TCP 설정을 위해 수동 모드로 전환
            tcp_name = "scraper"
            tcp_offset = [0, 0, 224.911 + 52.0, 0, 0, 0]  # x, y, z, a, b, c
            add_tcp(tcp_name, tcp_offset)          # 수동 모드에서 호출
            set_tcp(tcp_name)
            set_robot_mode(ROBOT_MODE_AUTONOMOUS)
            time.sleep(0.2)

            self._set_scraper_status(self.STEP_COATING, "접착제 도포중")

            # ✅ 순응 + 목표힘 ON
            enable_coating_compliance(
                stx=(3000, 3000, 200, 200, 200, 200),
                fz=-15.0,
                ref=DR_TOOL,
            )

            # (선택) 접촉을 확실히 만들고 시작하려면 주석 해제
            approach_until_contact(threshold=4.0, max_down_mm=15.0)

            # 도포 스트로크
            do_stroke()  # 중심점에서 1회

            move_relative(-50.0, 80.0, 0.0)  # X축 -100mm로 이동 후 반복 
            do_stroke()

            move_relative(100.0, 80.0, 0.0)  # X축 +200mm로 이동 후 반복
            do_stroke()

        finally:
            #  순응 OFF 
            disable_coating_compliance()

            # TCP 원복 (도포 중 예외가 나도 복구)
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
        movel(pre_mid, vel=60, acc=60)
        time.sleep(1.0)
        movel(pre_grasp, vel=60, acc=60)
        time.sleep(1.0)
        movel(grasp, vel=60, acc=60)
        time.sleep(1.0)

        set_gripper(0.040)
        time.sleep(4.0)

        movel(pre_grasp, vel=60, acc=60)
        time.sleep(1.0)

        return True


def main(args=None):
    rclpy.init(args=args)
    cfg = RobotConfig()

    # DSR_ROBOT2 import 전에 boot node 먼저 만들고 DR_init.__dsr__node 세팅
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