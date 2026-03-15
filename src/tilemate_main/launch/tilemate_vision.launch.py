from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory("tilemate_main")

    npy_path = os.path.join(pkg_share, "resource", "T_gripper2camera.npy")

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
        parameters=[{
            # anomalib checkpoint 파일명 (share/tilemate_main/resource 기준)
            "ckpt_filename": "dataset_1280_type6_patchcore.ckpt",
            # 빈 문자열("")이면 모든 패턴 대상, 기본은 pattern_5 집중 검사
            "target_pattern": "pattern_5",
            # True면 bilateral+CLAHE 전처리 후 모델 입력
            "use_canny_filter": False,
            # -1.0이면 체크포인트 내 threshold 자동 사용
            "anomaly_threshold": -1.0,
        }],
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