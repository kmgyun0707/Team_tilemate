# launch/tilemate.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("tilemate_main")
    npy_path = os.path.join(
        pkg_share,
        "resource",
        "T_gripper2camera.npy"
    )

    return LaunchDescription([
        # 노드의 로봇 ns는 내부에서 namespace=dsr01로 생성됨
        # 타일 피킹 노드
        Node(
            package="tilemate_main",
            executable="pick_tile_action_server",
            output="screen",
        ),
        #협업 노드
        Node(
            package="tilemate_main",
            executable="cowork_action_server",
            output="screen",
        ),
        # 타일 배치 노드
        Node(
            package="tilemate_main",
            executable="place_tile_action_server",
            output="screen",
        ),
        # 타일 압착 노드
        Node(
            package="tilemate_main",
            executable="press_action_server",
            output="screen",
        ),
        # 상태 관리 노드
        Node(
            package="tilemate_main",
            executable="task_manager",
            output="screen",
        ),
    ])
