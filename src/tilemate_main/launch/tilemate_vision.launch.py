from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory("tilemate_main")

    npy_path = os.path.join(pkg_share, "resource", "T_gripper2camera.npy")
    rule_based_img_path = os.path.join(pkg_share, "resource", "rule_based_img")

    # -----------------------------
    # launch args
    # -----------------------------
    debug_show_image_arg = DeclareLaunchArgument(
        "debug_show_image",
        default_value="false",
        description="Show OpenCV debug visualization window",
    )

    debug_window_name_arg = DeclareLaunchArgument(
        "debug_window_name",
        default_value="wall_tile_debug",
        description="OpenCV debug window name",
    )

    debug_wait_ms_arg = DeclareLaunchArgument(
        "debug_wait_ms",
        default_value="1",
        description="cv2.waitKey delay in ms",
    )

    debug_show_image = LaunchConfiguration("debug_show_image")
    debug_window_name = LaunchConfiguration("debug_window_name")
    debug_wait_ms = LaunchConfiguration("debug_wait_ms")

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
            "gripper2cam_path": npy_path,
            "debug_show_image": True,
            "debug_window_name": "wall_tile_debug",
            "debug_wait_ms": 1,
        }],
    )

    pattern_node = Node(
        package="tilemate_main",
        executable="pattern_inspect_action_server",
        namespace="dsr01",
        output="screen",
    )

    return LaunchDescription([
        debug_show_image_arg,
        debug_window_name_arg,
        debug_wait_ms_arg,

        yolo_node,

        TimerAction(
            period=2.0,
            actions=[pattern_node],
        ),

        TimerAction(
            period=4.0,
            actions=[inspect_node],
        ),
    ])