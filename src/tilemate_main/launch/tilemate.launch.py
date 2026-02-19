# launch/tilemate.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 1) OnRobot 그리퍼 폭 명령 변환 노드
        Node(
            package="cobot1",
            executable="gripper_node",
            name="gripper_node",
            output="screen",
        ),

        # 2) DSR_ROBOT2 모션 전용 노드 (내부에서 namespace=dsr01로 생성됨)
        Node(
            package="cobot1",
            executable="motion_node",
            name="motion_node",
            output="screen",
        ),

        # 3) /robot/command -> /task/* 이벤트로 변환(상태/트리거)
        Node(
            package="cobot1",
            executable="task_controller_node",
            name="task_controller_node",
            output="screen",
        ),

        # 4) /robot/command 중 stop/pause/reset에 대해 MoveStop 호출(하드정지)
        Node(
            package="cobot1",
            executable="command_node",
            name="command_node",
            output="screen",
        ),
    ])
