#!/usr/bin/env python3
# cobot1/task_controller_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32, Int16


class TaskControllerNode(Node):
    """
    /robot/command(String) -> 내부 이벤트 토픽 발행 + 상태(step) 기반 체이닝

    - start/once:
        stop_soft=false, pause=false
        -> scraper 1회 트리거

    - scraper가 /robot/step = 3(끝) publish:
        -> tile 1회 트리거 (자동)

    pause/resume:
        -> /task/pause

    stop/reset:
        -> /task/stop_soft (하드정지는 CommandNode가 별도 처리)
    """

    STEP_PREPARE = 0
    STEP_GRIPPING = 1
    STEP_COATING = 2
    STEP_DONE = 3

    def __init__(self):
        super().__init__("task_controller_node")

        # 토큰(각 작업별로 따로 관리)
        self._scraper_token = 0
        self._tile_token = 0

        # step 체이닝 제어용
        self._last_step = None
        self._tile_triggered_for_scraper_token = 0  # "이 scraper 토큰에 대해 tile을 이미 쐈는가"

        # 퍼블리셔
        self.pub_scraper_run = self.create_publisher(Int32, "/scraper/run_once", 10)
        self.pub_tile_run = self.create_publisher(Int32, "/tile/run_once", 10)

        self.pub_pause = self.create_publisher(Bool, "/task/pause", 10)
        self.pub_stop_soft = self.create_publisher(Bool, "/task/stop_soft", 10)

        # 구독자
        self.create_subscription(String, "/robot/command", self._cb_cmd, 10)
        self.create_subscription(Int32, "/robot/step", self._cb_step, 10)

        self.get_logger().info("TaskControllerNode ready: sub /robot/command, /robot/step")

    # -----------------
    # publish helpers
    # -----------------
    def _publish_pause(self, v: bool):
        m = Bool()
        m.data = bool(v)
        self.pub_pause.publish(m)

    def _publish_stop_soft(self, v: bool):
        m = Bool()
        m.data = bool(v)
        self.pub_stop_soft.publish(m)

    def _publish_scraper_run_once(self):
        self._scraper_token += 1
        m = Int32()
        m.data = self._scraper_token
        self.pub_scraper_run.publish(m)
        self.get_logger().info(f"[TASK] trigger SCRAPER run_once token={m.data}")

    def _publish_tile_run_once(self):
        self._tile_token += 1
        m = Int32()
        m.data = self._tile_token
        self.pub_tile_run.publish(m)
        self.get_logger().info(f"[TASK] trigger TILE run_once token={m.data}")

    # -----------------
    # callbacks
    # -----------------
    def _cb_cmd(self, msg: String):
        cmd = msg.data.strip().lower()

        if cmd == "start":
            self.get_logger().info("[CMD] start")
            self._publish_stop_soft(False)
            self._publish_pause(False)

            # start를 "1회 스크래퍼 실행"으로 정의
            self._publish_scraper_run_once()

        elif cmd == "once":
            self.get_logger().info("[CMD] once")
            self._publish_stop_soft(False)
            self._publish_pause(False)
            self._publish_scraper_run_once()

        elif cmd == "pause":
            self.get_logger().warn("[CMD] pause")
            self._publish_pause(True)

        elif cmd == "resume":
            self.get_logger().info("[CMD] resume")
            self._publish_pause(False)

        elif cmd == "stop":
            self.get_logger().warn("[CMD] stop (soft)")
            self._publish_pause(False)
            self._publish_stop_soft(True)

        elif cmd == "reset":
            self.get_logger().warn("[CMD] reset")
            self._publish_pause(False)
            self._publish_stop_soft(True)

            # 선택: reset 시 토큰/체이닝 상태 초기화하고 싶으면 아래 켜기
            # self._scraper_token = 0
            # self._tile_token = 0
            # self._tile_triggered_for_scraper_token = 0

        else:
            self.get_logger().warn(f"[CMD] unknown: {cmd}")

    def _cb_step(self, msg: Int32):
        step = int(msg.data)

        # 같은 step이 계속 들어오면 중복 처리 방지
        if self._last_step == step:
            return
        self._last_step = step

        self.get_logger().info(f"[STEP] /robot/step={step}")

        # 스크래퍼 완료(step=3) -> 타일 배치 자동 트리거
        if step == self.STEP_DONE:
            if self._tile_triggered_for_scraper_token == self._scraper_token:
                self.get_logger().info("[CHAIN] tile already triggered for this scraper token. skip.")
                return

            if self._scraper_token <= 0:
                self.get_logger().warn("[CHAIN] got step=3 but scraper_token=0. skip.")
                return

            self.get_logger().info("[CHAIN] scraper done -> trigger tile placement")
            self._publish_tile_run_once()
            self._tile_triggered_for_scraper_token = self._scraper_token


def main(args=None):
    rclpy.init(args=args)
    node = TaskControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
