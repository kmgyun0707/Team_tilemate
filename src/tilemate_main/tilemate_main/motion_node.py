#!/usr/bin/env python3
import time
import threading
import traceback

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import Bool, Int32, Float64

import DR_init
from tilemate_main.robot_config import RobotConfig


# -------------------------
# gripper: motion -> gripper_node#
# -------------------------
class _GripperClient:
    def __init__(self, node: Node):
        self._node = node
        self._pub = node.create_publisher(Float64, "/gripper/width_m", 10)

    def set_width(self, width_m: float):
        msg = Float64()
        msg.data = float(width_m)
        self._pub.publish(msg)
        self._node.get_logger().info(f"[GRIPPER->CMD] width_m={msg.data:.4f}")


# -------------------------
# control node: subscribe flags/tokens (this node is spun)
# -------------------------
class _ControlNode(Node):
    def __init__(self):
        super().__init__("motion_ctrl_node")

        self.pause = False
        self.stop_soft = False
        self.run_token = 0          # latest received token

        self.create_subscription(Int32, "/task/run_once", self._cb_run_once, 10)
        self.create_subscription(Bool,  "/task/pause", self._cb_pause, 10)
        self.create_subscription(Bool,  "/task/stop_soft", self._cb_stop_soft, 10)

        self.get_logger().info("ControlNode ready: sub /task/run_once, /task/pause, /task/stop_soft")

    def _cb_run_once(self, msg: Int32):
        tok = int(msg.data)
        if tok > self.run_token:
            self.run_token = tok
            self.get_logger().info(f"[CTRL] run_once token={self.run_token}")

    def _cb_pause(self, msg: Bool):
        self.pause = bool(msg.data)
        self.get_logger().warn(f"[CTRL] pause={self.pause}")

    def _cb_stop_soft(self, msg: Bool):
        self.stop_soft = bool(msg.data)
        self.get_logger().warn(f"[CTRL] stop_soft={self.stop_soft}")


def _spin_ctrl_node(ctrl_node: Node, stop_event: threading.Event):
    ex = SingleThreadedExecutor()
    ex.add_node(ctrl_node)
    try:
        while rclpy.ok() and not stop_event.is_set():
            ex.spin_once(timeout_sec=0.1)
    finally:
        try:
            ex.remove_node(ctrl_node)
        except Exception:
            pass


# -------------------------
# "move_scraper.py와 동일" 로직 함수들
# -------------------------
def initialize_robot(cfg: RobotConfig):
    from DSR_ROBOT2 import (
        set_tool, set_tcp, get_tool, get_tcp,
        ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS,
        get_robot_mode, set_robot_mode
    )

    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(cfg.tool)
    set_tcp(cfg.tcp)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(1.0)

    print("#" * 50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {cfg.robot_id}")
    print(f"ROBOT_MODEL: {cfg.robot_model}")
    print(f"ROBOT_TCP: {get_tcp()}")
    print(f"ROBOT_TOOL: {get_tool()}")
    print(f"ROBOT_MODE (0:수동, 1:자동): {get_robot_mode()}")
    print(f"VELOCITY: {cfg.vel}")
    print(f"ACC: {cfg.acc}")
    print("#" * 50)


def run_once_like_script(motion_node: Node, gripper: _GripperClient, cfg: RobotConfig, ctrl: _ControlNode):
    """
    ✅ 너가 올린 단독 스크립트 perform_task()의 while 내부 1사이클을 그대로 함수로 뽑은 것.
    pause/stop_soft는 중간중간 체크해서 즉시 대기/중단.
    """
    from DSR_ROBOT2 import posx, posj, movej, movel

    def wait_if_paused_or_stopped():
        # stop_soft면 바로 중단
        if ctrl.stop_soft:
            return False
        # pause면 풀릴 때까지 대기
        while rclpy.ok() and ctrl.pause and not ctrl.stop_soft:
            time.sleep(0.05)
        return not ctrl.stop_soft

    def set_gripper(width_m: float):
        gripper.set_width(width_m)
        time.sleep(0.05)

    def pick_scraper():
        pre = posx([634.6129150390625, 70.18247985839844, 216.4523162841797,
                    52.00111389160156, 179.09434509277344, 52.347633361816406])
        movel(pre, vel=60, acc=60)
        time.sleep(1.0)
        if not wait_if_paused_or_stopped(): return False

        grasp = posx([635.6554565429688, 69.99335479736328, 156.9518280029297,
                      122.97427368164062, 179.7987518310547, 123.16593170166016])
        movel(grasp, vel=60, acc=60)
        time.sleep(1.0)
        if not wait_if_paused_or_stopped(): return False

        set_gripper(0.003)
        time.sleep(4.0)
        return wait_if_paused_or_stopped()

    def place_scraper():
        pre = posx([634.6129150390625, 70.18247985839844, 216.4523162841797,
                    52.00111389160156, 179.09434509277344, 52.347633361816406])
        movel(pre, vel=60, acc=60)
        time.sleep(1.0)
        if not wait_if_paused_or_stopped(): return False

        place = posx([635.6554565429688, 69.99335479736328, 186.9518280029297,
                      122.97427368164062, 179.7987518310547, 123.16593170166016])
        movel(place, vel=60, acc=60)
        time.sleep(1.0)
        if not wait_if_paused_or_stopped(): return False

        set_gripper(0.04)
        time.sleep(4.0)
        return wait_if_paused_or_stopped()

    # --- 스크립트와 동일 ---
    JReady = [0, 0, 90, 0, 90, 0]

    motion_node.get_logger().info("[TASK] movej(JReady)")
    movej(JReady, vel=cfg.vel, acc=cfg.acc)

    set_gripper(0.06)
    time.sleep(2.0)
    if not wait_if_paused_or_stopped(): return

    if not pick_scraper(): return

    pos3 = posx([634.6129150390625, 70.18247985839844, 216.4523162841797,
                 52.00111389160156, 179.09434509277344, 52.347633361816406])
    movel(pos3, vel=60, acc=60)
    time.sleep(1.0)
    if not wait_if_paused_or_stopped(): return

    pos4 = posx([480.86981201171875, 68.99758911132812, 167.26080322265625,
                 59.91958999633789, 179.1564178466797, 60.55112075805664])
    movel(pos4, vel=60, acc=60)
    time.sleep(1.0)
    if not wait_if_paused_or_stopped(): return

    p5 = posj([6.65, 16.37, 75.84, 0.56, 87.29, 95.63])
    movej(p5, vel=40, acc=40)
    time.sleep(1.0)
    if not wait_if_paused_or_stopped(): return

    motion_node.get_logger().info("시멘트 펴바르기 작업 시작")

    tilt_right = posx([465.40, -34.13, 160.68, 92.35, -142.05, 179.91])
    movel(tilt_right, vel=40, acc=40)
    time.sleep(1.0)
    if not wait_if_paused_or_stopped(): return

    pos7 = posx([465.40, 250.13, 160.68, 92.35, -142.05, 179.91])
    movel(pos7, vel=40, acc=40)
    time.sleep(1.0)
    if not wait_if_paused_or_stopped(): return

    tilt_left = posx([462.86, 274.11, 160.68, 93.23, 148.62, -179.15])
    movel(tilt_left, vel=20, acc=20)
    time.sleep(1.0)
    if not wait_if_paused_or_stopped(): return

    pos9 = posx([528.40, -34.13, 160.68, 93.23, 148.62, -179.15])
    movel(pos9, vel=40, acc=40)
    time.sleep(1.0)
    if not wait_if_paused_or_stopped(): return

    motion_node.get_logger().info("시멘트 펴바르기 작업 종료")

    movej(JReady, vel=cfg.vel, acc=cfg.acc)
    time.sleep(2.0)
    if not wait_if_paused_or_stopped(): return

    place_scraper()


def main(args=None):
    rclpy.init(args=args)

    cfg = RobotConfig()
    stop_event = threading.Event()

    motion_node = None
    ctrl_node = None
    ctrl_thread = None

    try:
        # ✅ 너 스크립트처럼 create_node로 모션노드 생성 (DSR import 성공 패턴)
        motion_node = rclpy.create_node("motion_node", namespace=cfg.robot_id)

        DR_init.__dsr__id = cfg.robot_id
        DR_init.__dsr__model = cfg.robot_model
        DR_init.__dsr__node = motion_node

        import DSR_ROBOT2  # noqa: F401
        print("[DBG] DSR_ROBOT2 import OK")

        # control node는 별도로 spin
        ctrl_node = _ControlNode()
        ctrl_thread = threading.Thread(target=_spin_ctrl_node, args=(ctrl_node, stop_event), daemon=True)
        ctrl_thread.start()

        gripper = _GripperClient(motion_node)

        initialize_robot(cfg)
        motion_node.get_logger().info("Motion loop ready. Waiting /task/run_once...")

        # ✅ 스크립트와 동일한 “메인 while 루프” (spin 없음)
        last_done_token = 0

        while rclpy.ok():
            # stop_soft면 아무 것도 안 하고 대기(원하면 break로 완전 중단도 가능)
            if ctrl_node.stop_soft:
                time.sleep(0.05)
                continue

            # pause면 대기
            if ctrl_node.pause:
                time.sleep(0.05)
                continue

            # run_once 토큰이 증가했을 때만 1회 실행
            if ctrl_node.run_token > last_done_token:
                tok = ctrl_node.run_token
                motion_node.get_logger().info(f"[TASK] run_once start token={tok}")
                run_once_like_script(motion_node, gripper, cfg, ctrl_node)
                motion_node.get_logger().info(f"[TASK] run_once done token={tok}")
                last_done_token = tok
            else:
                time.sleep(0.02)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"[ERROR] {e}")
        traceback.print_exc()
    finally:
        stop_event.set()
        time.sleep(0.2)

        if ctrl_node is not None:
            try:
                ctrl_node.destroy_node()
            except Exception:
                pass
        if motion_node is not None:
            try:
                motion_node.destroy_node()
            except Exception:
                pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
