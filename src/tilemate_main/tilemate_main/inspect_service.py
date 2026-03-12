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

    def __init__(self, robot_cfg: RobotConfig, gripper_cfg: GripperConfig, boot_node: Node):

        super().__init__("inspect_service", namespace=RobotConfig.robot_id)

        self._boot_node = boot_node
        self.robot_cfg = RobotConfig
        self.gripper_cfg = GripperConfig

        from tilemate_main.onrobot import RG
        self.gripper = RG(
            gripper_cfg.GRIPPER_NAME,
            gripper_cfg.TOOLCHARGER_IP,
            gripper_cfg.TOOLCHARGER_PORT,
        )
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

        from DSR_ROBOT2 import get_current_posx, movel, posx, mwait, DR_BASE, posj, movej,movesx

        try:

            self.get_logger().info("[INSPECT] start inspection")
            self.gripper.open_gripper()

            candidates = [
                posx([380.6733093261719, 177.2272491455078, 179.8480987548828, 89.89385223388672, 91.91939544677734, 92.74739837646484]),
                posx([380.705, 157.182, 139.804, 90.000, 90.001, 89.999]),
                posx([380.705, 127.182, 109.804, 90.000, 90.001, 89.999]),
            ]
            
            movesx(candidates, vel=80, acc=80)
            mwait()

            self.get_logger().info(f"{get_current_posx()}")


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
    robot_cfg = RobotConfig()
    gripper_cfg = GripperConfig()

    boot = rclpy.create_node("dsr_boot_inspect", namespace=RobotConfig.robot_id)

    DR_init.__dsr__node = boot
    DR_init.__dsr__id = RobotConfig.robot_id
    DR_init.__dsr__model = RobotConfig.robot_model

    import DSR_ROBOT2  # noqa

    node = InspectService(robot_cfg, gripper_cfg, boot)

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