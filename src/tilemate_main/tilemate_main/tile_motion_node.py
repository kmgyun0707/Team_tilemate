#!/usr/bin/env python3
# tilemate_main/tile_motion_node.py

import time
import traceback

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, Float64, String

import DR_init
from tilemate_main.robot_config import RobotConfig


VELOCITY = 30
ACC = 30

OPEN_W  = 0.025
CLOSE_W = 0.005

PICK_DY_MM = 32.5

pick_above = [344, -101, 210, 50, 179, 140]

PLACE_TILT_BASE01 = [401, 22, 210, 8, -179, 98]
PLACE_TILT_BASE02 = [415, 24, 210, 6, -179, 97]
PLACE_TILT_BASE03 = [541, 19, 210, 156, 180, -115]

pick_down  = [344, -101, 190, 50, 179, 140]
place_down = [401,  22, 170, 8, -179, 98]


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

        self.pub_status = self.create_publisher(String, "/tile/status", 10)
        self.pub_state = self.create_publisher(String, "/robot/state", 10)
        self.pub_step = self.create_publisher(Int32, "/robot/step", 10)
        self.pub_completed_jobs = self.create_publisher(Int32, "/robot/completed_jobs", 10)
        self._completed_jobs = 0

        self.create_subscription(Int32, "/tile/run_once", self._cb_run_once, 10)
        self.create_subscription(Bool,  "/task/pause", self._cb_pause, 10)
        self.create_subscription(Bool,  "/task/stop_soft", self._cb_stop_soft, 10)

        self.gripper = _GripperClient(self)

        self._initialize_robot()
        self.get_logger().info("TileMotionNode ready: sub /tile/run_once")

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
        m_step.data = step
        m_state = String()
        m_state.data = state
        self.pub_step.publish(m_step)
        self.pub_state.publish(m_state)
        self.get_logger().info(f"[STATUS] step={step} state='{state}'")

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

    def _wait_if_paused(self):
        while rclpy.ok() and self._pause and not self._stop_soft:
            time.sleep(0.05)

    def _publish_status(self, s: str):
        m = String()
        m.data = s
        self.pub_status.publish(m)

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
                self.get_logger().warn("[TILE] stop_soft=True during pause -> abort")
                return

            self.get_logger().info(f"[TILE] run_once start token={tok}")
            self._perform_task_2x2()

            if not self._stop_soft:
                self._set_robot_status(5, "타일 작업 완료")

            self._publish_status(f"done:{tok}")

        except Exception as e:
            self.get_logger().error(f"[TILE] exception: {e}")
            self.get_logger().error(traceback.format_exc())
            self._publish_status(f"error:{tok}:{e}")

        finally:
            self._running = False

    def _perform_task_2x2(self):
        from DSR_ROBOT2 import movej, movel, wait, posx
        from DSR_ROBOT2 import (
            set_ref_coord, task_compliance_ctrl, release_compliance_ctrl,
            set_desired_force, release_force, check_force_condition,
            DR_BASE, DR_TOOL, DR_AXIS_Z, DR_FC_MOD_REL, get_current_posx
        )

        def hold_with_force_4n(hold_n=4.0, timeout_s=3.0):
            set_ref_coord(DR_TOOL)
            task_compliance_ctrl(stx=[1000, 1000, 100, 200, 200, 200])
            wait(0.3)

            set_desired_force(
                fd=[0, 0, hold_n, 0, 0, 0],
                dir=[0, 0, 1, 0, 0, 0],
                mod=DR_FC_MOD_REL
            )

            t0 = time.time()
            while True:
                ret = check_force_condition(DR_AXIS_Z, min=0, max=hold_n)
                if ret == -1:
                    break
                if time.time() - t0 > timeout_s:
                    self.get_logger().warn("[HOLD] force condition timeout -> release")
                    break
                wait(0.2)

            release_force()
            release_compliance_ctrl()
            set_ref_coord(DR_BASE)
            wait(0.2)

        def is_external_force_detected_z(threshold_n=2.0):
            try:
                ret = check_force_condition(DR_AXIS_Z, min=0, max=float(threshold_n))
                self.get_logger().info(f"[CONTACT] check_force_condition ret={ret} (th={threshold_n}N)")
                return ret != 0
            except Exception as e:
                self.get_logger().warn(f"[CONTACT] check_force_condition failed: {e}")
                return False

        # ------------------------------------------------------------
        # ✅ [추가] "누르고 유지" 시작/종료
        # - 타일을 바닥에 고정한 채(정상력 N↑) 틸트해서 박리할 때 사용
        # - 여기서는 force+compliance를 켜둔 상태로 유지하고,
        #   틸트 동작이 끝나면 반드시 release
        # ------------------------------------------------------------
        def press_hold_begin(
            press_fz_n=-10.0,          # ✅ 아래로 누르는 힘(부호 반대면 +로)
            stx=None,                  # ✅ Z/회전은 약간 부드럽게 두는 게 박리에 유리
            settle_s=0.2,
        ):
            if stx is None:
                stx = [3000, 3000, 200, 80, 80, 80]

            # 혹시 남아있는 모드 정리
            try:
                release_force()
            except Exception:
                pass
            try:
                release_compliance_ctrl()
            except Exception:
                pass

            set_ref_coord(DR_BASE)
            task_compliance_ctrl(stx=stx)
            wait(settle_s)

            set_desired_force(
                fd=[0.0, 0.0, float(press_fz_n), 0.0, 0.0, 0.0],
                dir=[0,   0,   1,               0,   0,   0],
                mod=DR_FC_MOD_REL
            )
            wait(settle_s)

        def press_hold_end():
            try:
                release_force()
            except Exception:
                pass
            try:
                release_compliance_ctrl()
            except Exception:
                pass
            set_ref_coord(DR_BASE)
            wait(0.1)

        # ------------------------------------------------------------
        # ✅ 스텝 틸트(천천히) - "누르고 유지" 상태에서 호출하는 버전
        # - 내부에서 compliance/force를 건드리지 않음 (이미 press_hold_begin에서 ON)
        # ------------------------------------------------------------
        def tilt_stepwise_under_press(
            dz_mm=-0.2,
            dp_deg=+15.0,
            vel=0.15,
            acc=0.15,
            steps=25,
            step_wait_s=0.02,
        ):
            if steps < 1:
                steps = 1

            dz_step = float(dz_mm) / float(steps)
            dp_step = float(dp_deg) / float(steps)

            self.get_logger().info(
                f"[TILT_UNDER_PRESS] steps={steps} total_dz={dz_mm}mm total_dp={dp_deg}deg "
                f"(dz_step={dz_step:.4f}, dp_step={dp_step:.3f})"
            )

            for _ in range(steps):
                cur, _ = get_current_posx(DR_BASE)
                x, y, z, r, p, yaw = cur

                target = posx([
                    float(x),
                    float(y),
                    float(z + dz_step),
                    float(r),
                    float(p + dp_step),
                    float(yaw),
                ])

                movel(target, vel=vel, acc=acc, ref=DR_BASE)
                if step_wait_s > 0:
                    wait(float(step_wait_s))

        # ------------------------------------------------------------
        # Task sequence
        # ------------------------------------------------------------
        JReady = [0, 0, 90, 0, 90, 90]

        self.get_logger().info("[TILE] Move to JReady")
        movej(JReady, vel=VELOCITY, acc=ACC)

        # ---------------- PICK ----------------
        self._set_robot_status(1, "타일 파지 준비(검정 타일함 상부)")
        movel(pick_above, vel=VELOCITY, acc=ACC)

        self._set_robot_status(2, "타일 파지 하강")
        movel(pick_down, vel=10, acc=10)

        self._set_robot_status(3, "타일 파지 힘제어 고정")
        hold_with_force_4n(hold_n=5.0, timeout_s=10.0)

        wait(0.3)
        self._set_robot_status(3, "타일 파지 상승")
        movel(pick_above, vel=VELOCITY, acc=ACC)

        # ---------------- PLACE ----------------
        self._set_robot_status(4, "타일 배치 위치 상부 이동(1번)")
        movel(PLACE_TILT_BASE01, vel=VELOCITY, acc=ACC)

        self._set_robot_status(4, "타일 배치 하강(1번)")
        movel(place_down, vel=20, acc=20)

        self._set_robot_status(4, "타일 배치 힘제어 고정(1번)")
        hold_with_force_4n(hold_n=5.0, timeout_s=10.0)

        # ------------------------------------------------------------
        # ✅ 핵심 변경: "누르고(press) -> 그 상태로 틸트(stepwise) -> 해제"
        # ------------------------------------------------------------
        self._set_robot_status(4, "누르고 유지(press hold) 시작")
        # 타일이 딸려 올라오면 press_fz_n 절댓값을 키워(예:-15~-30)
        press_hold_begin(
            press_fz_n=-15.0,
            stx=[3000, 3000, 150, 60, 60, 60],
            settle_s=0.2,
        )

        try:
            self._set_robot_status(4, "스텝 틸트(누른 상태에서)")
            tilt_stepwise_under_press(
                dz_mm=-4.0,      # ✅ 내리면서 기울이기 (필요시 -0.2 ~ -1.5 튜닝)
                dp_deg=+15.0,    # ✅ 5~10deg부터 시작 추천
                vel=0.10,
                acc=0.10,
                steps=1,
                step_wait_s=0.02,
            )
        finally:
            self._set_robot_status(4, "press hold 종료(해제)")
            press_hold_end()

        self._set_robot_status(4, "타일 배치 상부 복귀(1번)")
        movel(PLACE_TILT_BASE01, vel=VELOCITY, acc=ACC)


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