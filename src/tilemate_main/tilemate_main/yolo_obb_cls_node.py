import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from rclpy.qos import qos_profile_sensor_data, ReliabilityPolicy, DurabilityPolicy, QoSProfile
from ament_index_python.packages import get_package_share_directory # 추가
import os


# 🌟 방금 만든 커스텀 메시지 임포트!
from custom_tile_msgs.msg import Tile, TileArray

def order_points(pts):
    """4개의 꼭짓점을 좌상, 우상, 우하, 좌하 순서로 정렬 (크롭 필수)"""
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    return rect

class YoloTwoStageNode(Node):
    def __init__(self):
        super().__init__('yolo_two_stage_node')
        self.bridge = CvBridge()

        # 동적 홈 디렉토리 경로 가져오기 
        package_name = 'tilemate_main' 
        
        # 패키지의 share 디렉토리 경로를 동적으로 가져옴
        pkg_share_dir = get_package_share_directory(package_name)
        
        # ==========================================
        # 🧠 1. 두 개의 모델 로드
        # ==========================================
        obb_model_path = os.path.join(pkg_share_dir, 'yolo26s-obb_v5.pt')
        cls_model_path = os.path.join(pkg_share_dir, 'yolov8n-cls_v1.pt')
  
        try:
            self.obb_model = YOLO(obb_model_path)
            self.cls_model = YOLO(cls_model_path)
            self.get_logger().info(" OBB(탐지) & CLS(분류) 모델 로드 성공!")
        except Exception as e:
            self.get_logger().error(f" 모델 로드 실패: {e}")
            raise e

        # 통신 품질(QoS) 설정
        custom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        # 구독(카메라)
        self.image_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, custom_qos)
            
        # 🌟 발행: 기존 개별 퍼블리셔를 TileArray 하나로 통합
        self.tile_array_pub = self.create_publisher(TileArray, '/yolo/tile_array', 10)
        self.annotated_pub = self.create_publisher(Image, '/yolo/annotated_image', 10)

        self.get_logger().info("\033[94m [1/3] [YOLO_OBB] initialize Done!\033[0m")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            annotated_frame = frame.copy() 

            # ==========================================
            # 🎯 Stage 1: 타일 위치 및 각도 찾기 (OBB)
            # ==========================================
            obb_results = self.obb_model(frame, verbose=False, conf=0.5)
            
            # 🌟 1프레임 분량의 데이터를 담을 빈 Array 객체 생성
            tile_array_msg = TileArray()
            tile_array_msg.header = msg.header # 타임스탬프 동기화
            
            if hasattr(obb_results[0], 'obb') and obb_results[0].obb is not None:
                for obb in obb_results[0].obb:
                    # --- 1. 위치 및 회전 정보 추출 ---
                    cx, cy, w, h, angle_rad = obb.xywhr[0].cpu().numpy()
                    
                    # --- 2. 꼭짓점 정렬 및 너비/높이 계산 ---
                    pts = obb.xyxyxyxy[0].cpu().numpy()
                    rect = order_points(pts)
                    (tl, tr, br, bl) = rect

                    widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
                    widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
                    maxWidth = max(int(widthA), int(widthB))

                    heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
                    heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
                    maxHeight = max(int(heightA), int(heightB))

                    # 노이즈 필터링 (너무 작은 박스 무시)
                    if maxWidth > 20 and maxHeight > 20:
                        # --- 3. 타일 반듯하게 펴기 (Perspective Transform) ---
                        dst = np.array([[0, 0], [maxWidth - 1, 0], [maxWidth - 1, maxHeight - 1], [0, maxHeight - 1]], dtype="float32")
                        M = cv2.getPerspectiveTransform(rect, dst)
                        cropped_tile = cv2.warpPerspective(frame, M, (maxWidth, maxHeight))

                        # ==========================================
                        # 🔍 Stage 2: 잘라낸 타일의 무늬 분류 (CLS)
                        # ==========================================
                        cls_results = self.cls_model(cropped_tile, verbose=False)
                        
                        top1_id = cls_results[0].probs.top1
                        pattern_name = cls_results[0].names[top1_id]
                        conf_score = cls_results[0].probs.top1conf.item()

                        # 🌟 개별 타일 정보 커스텀 메시지에 포장
                        tile_msg = Tile()
                        tile_msg.cropped_image = self.bridge.cv2_to_imgmsg(cropped_tile, encoding="bgr8")
                        tile_msg.pose.x = float(cx)
                        tile_msg.pose.y = float(cy)
                        tile_msg.pose.theta = float(angle_rad)
                        tile_msg.width = float(w)
                        tile_msg.height = float(h)
                        tile_msg.pattern_name = pattern_name
                        tile_msg.conf_score = float(conf_score)
                        
                        # 🌟 배열에 추가
                        tile_array_msg.tiles.append(tile_msg)

                        # ==========================================
                        # 🎨 시각화
                        # ==========================================
                        cv2.polylines(annotated_frame, [np.int32(rect)], isClosed=True, color=(0, 255, 0), thickness=2)
                        label_text = f"{pattern_name} ({conf_score:.2f})"
                        cv2.putText(annotated_frame, label_text, (int(tl[0]), int(tl[1]) - 10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            # 🌟 for문이 끝나고 배열에 타일이 1개 이상 담겨있다면 한 번에 발행!
            if len(tile_array_msg.tiles) > 0:
                self.tile_array_pub.publish(tile_array_msg)

            # 시각화 이미지 토픽 발행 (rqt_image_view에서 확인)
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            self.annotated_pub.publish(annotated_msg)
            
        except Exception as e:
            self.get_logger().error(f"🚨 처리 중 오류 발생: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloTwoStageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("종료 중...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()