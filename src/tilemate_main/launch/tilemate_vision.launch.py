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
    rule_based_img_path = os.path.join(
        pkg_share,
        "resource",
        "rule_based_img"
    )

    return LaunchDescription([
        # 노드의 로봇 ns는 내부에서 namespace=dsr01로 생성됨
        # 타일 단차 검사 노드
        Node(
            package="tilemate_main",
            executable="inspect_action_server",
            output="screen",
            parameters=[
                {
                    "gripper2cam_path": npy_path
                }
            ],
        ),
        # 타일 패턴 검사 노드
        Node(
            package="tilemate_main",
            executable="pattern_inspect_action_server",
            output="screen",
            # parameters=[
            #     {
            #         "rule_based_img_path": rule_based_img_path
            #     }
            # ],
        ),
        # Yolo모델 로드
        Node(
            package="tilemate_main",
            executable="yolo_obb_cls_node",
            output="screen",
        ),
    ])
