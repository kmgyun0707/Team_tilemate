# launch/tilemate.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 노드의 로봇 ns는 내부에서 namespace=dsr01로 생성됨
        # 타일 피킹 노드
        Node(
            package="tilemate_main",
            executable="pick_tile_action_server",
            name="pick_tile_action_server",
            output="screen",
        ),
        # 타일 배치 노드
        Node(
            package="tilemate_main",
            executable="place_tile_action_server",
            name="place_tile_action_server",
            output="screen",
        ),
        # # 타일 검사 노드
        Node(
            package="tilemate_main",
            executable="inspect_service",
            name="inspect_service",
            output="screen",
        ),
        # # 타일 압착 노드
        # Node(
        #     package="tilemate_main",
        #     executable="tile_compact_motion_node",
        #     name="tile_compact_motion_node",
        #     output="screen",
        # ),
        # 상태 관리 노드
        Node(
            package="tilemate_main",
            executable="task_manager",
            name="task_manager",
            output="screen",
        ),
    ])
