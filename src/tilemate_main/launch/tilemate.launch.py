# launch/tilemate.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # OnRobot 그리퍼 폭 명령 변환 노드
        Node(
            package="tilemate_main",
            executable="gripper_node",
            name="gripper_node",
            output="screen",
        ),
        # 이하 노드의 로봇 ns는 내부에서 namespace=dsr01로 생성됨
        # 스크래퍼 파지 및 접착제 도포 노드
        Node(
            package="tilemate_main",
            executable="scraper_motion_node",
            name="scraper_motion_node",
            output="screen",
        ),
        # 타일 배치 노드
        Node(
            package="tilemate_main",
            executable="tile_motion_node",
            name="tile_motion_node",
            output="screen",
        ),
        # 타일 검사 노드
        Node(
            package="tilemate_main",
            executable="tile_inspect_motion_node",
            name="tile_inspect_motion_node",
            output="screen",
        ),
        # 타일 압착 노드
        Node(
            package="tilemate_main",
            executable="tile_compact_motion_node",
            name="tile_compact_motion_node",
            output="screen",
        ),
        # 상태 관리 노드
        Node(
            package="tilemate_main",
            executable="task_manager_node",
            name="task_manager_node",
            output="screen",
        ),
        # 인터럽트 노드 (비상정지 , 복구)
        Node(
            package="tilemate_main",
            executable="interrupt_node",
            name="interrupt_node",
            output="screen",
        ),
    ])
