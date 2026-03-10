#!/usr/bin/env python3
import time
import rclpy
import DR_init

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from tilemate_msgs.srv import Inspect
from tilemate_main.robot_config import RobotConfig, GripperConfig


class InspectService(Node):

    def __init__(self, boot_node: Node):

        super().__init__("inspect_service", namespace=RobotConfig.robot_id)

        self._boot_node = boot_node
        self.robot_cfg = RobotConfig
        self.gripper_cfg = GripperConfig

        self.cb_group = ReentrantCallbackGroup()

        self.srv = self.create_service(
            Inspect,
            "tile/inspect",
            self.inspect_callback,
            callback_group=self.cb_group,
        )

        self.initialize_robot()

        self.get_logger().info("\033[94m [5/5] [INSPECT] initialize Done!\033[0m")

    # --------------------------------------------------
    # Robot init
    # --------------------------------------------------
    def initialize_robot(self):

        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            get_tool,
            get_tcp,
            ROBOT_MODE_MANUAL,
            ROBOT_MODE_AUTONOMOUS,
            get_robot_mode,
            set_robot_mode,
            wait,
        )

        set_robot_mode(ROBOT_MODE_MANUAL)
        set_tool(self.robot_cfg.robot_id)
        set_tcp(self.robot_cfg.tcp)
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)

        time.sleep(1.0)
        wait(1.0)


    # --------------------------------------------------
    # helper
    # --------------------------------------------------
    def move_relative(self, dx: float, dy: float, dz: float):

        from DSR_ROBOT2 import get_current_posx, movel, posx, mwait, DR_BASE

        cur, _ = get_current_posx(DR_BASE)

        target = [
            cur[0] + dx,
            cur[1] + dy,
            cur[2] + dz,
            cur[3],
            cur[4],
            cur[5],
        ]

        movel(posx(target), ref=DR_BASE, vel=30, acc=30)
        mwait()

    # --------------------------------------------------
    # Service callback
    # --------------------------------------------------
    def inspect_callback(self, request, response):

        del request

        from DSR_ROBOT2 import get_current_posx, movel, posx, mwait, DR_BASE, posj, movej

        try:

            self.get_logger().info("[INSPECT] start inspection")

            # 2. pre_place 이동 (Joint, 특이점 회피)
            pre_place = posj([-25.894,29.976,114.271,67.522,103.095,-54.547])
            movej(pre_place, vel=30, acc=30)
            mwait()

            # 카메라 관측 위치 이동
            self.move_relative(0.0, -50.0, -70.0)

            cur_pos, _ = get_current_posx(DR_BASE)
            self.get_logger().info(f"[INSPECT] TCP before align: {cur_pos}")

            # orientation 정렬
            target_pos = [
                cur_pos[0],
                cur_pos[1],
                cur_pos[2],
                90.0,
                90.0,
                90.0,
            ]

            movel(posx(target_pos), ref=DR_BASE, vel=40, acc=40)
            mwait()

            cur_pos, _ = get_current_posx(DR_BASE)
            self.get_logger().info(f"[INSPECT] TCP after align: {cur_pos}")

            # dummy anomaly score
            scores = [0.05, 0.12, 0.03]

            response.success = True
            response.message = "inspect_complete"
            response.anomaly_scores = scores

            return response

        except Exception as e:

            self.get_logger().error(f"[INSPECT] failed: {e}")

            response.success = False
            response.message = f"exception:{e}"
            response.anomaly_scores = []

            return response


def main(args=None):

    rclpy.init(args=args)

    boot = rclpy.create_node("dsr_boot_inspect", namespace=RobotConfig.robot_id)

    DR_init.__dsr__node = boot
    DR_init.__dsr__id = RobotConfig.robot_id
    DR_init.__dsr__model = RobotConfig.robot_model

    import DSR_ROBOT2  # noqa

    node = InspectService(boot)

    ex = MultiThreadedExecutor()
    ex.add_node(node)

    try:
        ex.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")

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

        rclpy.shutdown()


if __name__ == "__main__":
    main()