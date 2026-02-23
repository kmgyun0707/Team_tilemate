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
        # 내부에서 namespace=dsr01로 생성됨
        Node(
            package="tilemate_main",
            executable="scraper_motion_node",
            name="scraper_motion_node",
            output="screen",
        ),
        # tile배치 노드
        Node(
            package="tilemate_main",
            executable="tile_motion_node",
            name="tile_motion_node",
            output="screen",
        ),
        # 3) /robot/command -> /task/* 이벤트로 변환(상태/트리거)
        Node(
            package="tilemate_main",
            executable="task_manager_node",
            name="task_manager_node",
            output="screen",
        ),
        # 4) /robot/command 중 stop/pause/reset에 대해 MoveStop 호출(하드정지)
        Node(
            package="tilemate_main",
            executable="command_node",
            name="command_node",
            output="screen",
        ),
    ])
