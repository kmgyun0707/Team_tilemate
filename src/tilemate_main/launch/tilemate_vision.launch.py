from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory("tilemate_main")

    npy_path = os.path.join(pkg_share, "resource", "T_gripper2camera.npy")
    rule_based_img_path = os.path.join(pkg_share, "resource", "rule_based_img")

    yolo_node = Node(
        package="tilemate_main",
        executable="yolo_obb_cls_node",
        output="screen",
    )

    inspect_node = Node(
        package="tilemate_main",
        executable="inspect_action_server",
        output="screen",
        parameters=[{
            "gripper2cam_path": npy_path
        }],
    )

    pattern_node = Node(
        package="tilemate_main",
        executable="pattern_inspect_action_server",
        namespace="dsr01",
        output="screen",
    )

    return LaunchDescription([
        yolo_node,

        TimerAction(
            period=2.0,
            actions=[inspect_node],
        ),

        TimerAction(
            period=4.0,
            actions=[pattern_node],
        ),
    ])