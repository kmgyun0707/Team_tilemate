import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import math
import os
from torchvision import transforms
from anomalib.models import Patchcore # EfficientAd 제외 (Patchcore 고정)
from ament_index_python.packages import get_package_share_directory # 🌟 동적 경로 탐색 추가

from custom_tile_msgs.msg import TileArray, InspectionResult, InspectionResultArray

class HybridAnomalibNode(Node):
    def __init__(self):
        super().__init__('hybrid_anomalib_node')
        self.bridge = CvBridge()

        # ==========================================
        # 💡 1. 딥러닝 모델 동적 경로 세팅
        # ==========================================
        self.TARGET_PATTERN = "pattern_5"  
        self.USE_CANNY_FILTER = False 

        # 🌟 tilemate_main 패키지의 share 디렉토리 경로를 자동으로 찾음
        package_name = 'tilemate_main'
        pkg_share_dir = get_package_share_directory(package_name)
        
        # resource 폴더 안의 가중치 파일 경로 조립
        ckpt_path = os.path.join(pkg_share_dir, 'resource', 'dataset_1280_type6_patchcore.ckpt')

        self.get_logger().info(f"🔍 딥러닝 뇌 로딩 중... [타겟: {self.TARGET_PATTERN} / Canny 필터: {self.USE_CANNY_FILTER}]")
        self.get_logger().info(f"📁 가중치 파일 탐색 경로: {ckpt_path}")

        if not os.path.exists(ckpt_path):
            raise FileNotFoundError(f"🚨 가중치 파일 없음! 빌드(colcon build) 시 resource 폴더가 포함되었는지 확인하세요: {ckpt_path}")

        # Patchcore 모델 로드
        self.model = Patchcore.load_from_checkpoint(ckpt_path)
        self.model.eval() 
        self.model.to('cuda')
        
        self.threshold = 0.65
        try:
            if hasattr(self.model, 'image_threshold'):
                self.threshold = self.model.image_threshold.value.item()
            elif hasattr(self.model, 'metrics') and hasattr(self.model.metrics, 'image_threshold'):
                self.threshold = self.model.metrics.image_threshold.value.item()
            self.get_logger().info(f"✅ 뇌 로딩 완료! (불량 컷오프: {self.threshold:.4f})")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Threshold 자동 추출 실패, 기본값 {self.threshold} 적용")

        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((512, 512), antialias=True),
            transforms.ToTensor(),
        ])

        # ==========================================
        # 🔥 2. 프레임 누적(버퍼) 트래킹 변수 세팅
        # ==========================================
        self.TARGET_FRAMES = 30     
        self.current_frame_count = 0     
        self.tracked_tiles = []          
        self.MATCH_DIST_THRESHOLD = 50.0 

        # 통신망 세팅
        self.tile_sub = self.create_subscription(TileArray, '/yolo/tile_array', self.inference_callback, 10)
        self.web_pub = self.create_publisher(InspectionResultArray, '/web/inspection_results', 10)
        
        self.heatmap_pub = self.create_publisher(Image, '/anomaly/heatmap', 10)        
        self.pred_mask_pub = self.create_publisher(Image, '/anomaly/pred_mask', 10)    
        self.debug_canny_pub = self.create_publisher(Image, '/anomaly/debug_canny', 10)

        self.get_logger().info("🚀 하이브리드 검사 노드 대기 중 (특정 패턴 전용 다수결 로직 탑재)")

    def apply_industry_filter(self, cv_img):
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        filtered = cv2.bilateralFilter(gray, 9, 75, 75)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        clahe_img = clahe.apply(filtered)
        return cv2.cvtColor(clahe_img, cv2.COLOR_GRAY2RGB)

    def inference_callback(self, msg):
        try:
            if self.current_frame_count >= self.TARGET_FRAMES:
                return

            inspected_any = False 

            for tile in msg.tiles:
                if tile.pattern_name != self.TARGET_PATTERN:
                    continue
                
                inspected_any = True 

                cv_img = self.bridge.imgmsg_to_cv2(tile.cropped_image, desired_encoding='bgr8')
                h, w = cv_img.shape[:2] 

                if self.USE_CANNY_FILTER:
                    model_input_img = self.apply_industry_filter(cv_img)
                else:
                    model_input_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)

                # 딥러닝 추론
                input_tensor = self.transform(model_input_img).unsqueeze(0).to('cuda')
                with torch.no_grad():
                    output = self.model(input_tensor)

                pred_score = float(output.pred_score if hasattr(output, 'pred_score') else output.get('pred_score', output[1] if isinstance(output, tuple) else 0.0))
                anomaly_map = output.anomaly_map if hasattr(output, 'anomaly_map') else output.get('anomaly_map', output[0] if isinstance(output, tuple) else output)
                
                if isinstance(anomaly_map, torch.Tensor):
                    anomaly_map = anomaly_map.detach().cpu().numpy()
                anomaly_map = np.squeeze(np.array(anomaly_map, dtype=np.float32))
                
                is_defect = bool(pred_score > self.threshold)

                # 위치 기반 트래킹
                matched = False
                for tt in self.tracked_tiles:
                    dist = math.hypot(tt['pose'].x - tile.pose.x, tt['pose'].y - tile.pose.y)
                    
                    if dist < self.MATCH_DIST_THRESHOLD: 
                        tt['score_history'].append(pred_score)
                        tt['vote_history'].append(is_defect)
                        tt['pose'] = tile.pose 
                        matched = True
                        break

                if not matched:
                    self.tracked_tiles.append({
                        'pose': tile.pose,
                        'pattern_name': tile.pattern_name,
                        'width': tile.width,
                        'height': tile.height,
                        'score_history': [pred_score],
                        'vote_history': [is_defect]
                    })

                # 시각화 (마지막 프레임에만)
                if self.current_frame_count == self.TARGET_FRAMES - 1:
                    debug_msg = self.bridge.cv2_to_imgmsg(model_input_img, encoding="bgr8")
                    self.debug_canny_pub.publish(debug_msg)
                    
                    norm_max = self.threshold * 2.0 
                    a_map_norm = np.clip(anomaly_map / (norm_max + 1e-5), 0, 1)
                    a_map_resized = cv2.resize((a_map_norm * 255).astype(np.uint8), (int(w), int(h)), interpolation=cv2.INTER_CUBIC)
                    heatmap = cv2.applyColorMap(a_map_resized, cv2.COLORMAP_JET)
                    overlay = cv2.addWeighted(cv_img, 0.5, heatmap, 0.5, 0)
                    self.heatmap_pub.publish(self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8"))

            if inspected_any:
                self.current_frame_count += 1
                self.get_logger().info(f"⏳ [{self.TARGET_PATTERN} 전용] 데이터 누적 중... ({self.current_frame_count}/{self.TARGET_FRAMES})")

            # 60프레임 도달 시 결과 전송
            if self.current_frame_count >= self.TARGET_FRAMES:
                self.publish_final_results(msg.header)
                self.reset_buffer() 

        except Exception as e:
            self.get_logger().error(f"🚨 하이브리드 추론 중 에러 발생: {e}")

    def publish_final_results(self, header):
        result_array_msg = InspectionResultArray()
        result_array_msg.header = header
        
        self.get_logger().info("=========================================")
        self.get_logger().info(f"🎯 {self.TARGET_FRAMES}프레임 다수결 검사 완료! (결과 웹 전송)")

        for idx, tt in enumerate(self.tracked_tiles):
            if len(tt['vote_history']) < (self.TARGET_FRAMES / 2):
                continue

            total_votes = len(tt['vote_history'])
            defect_votes = sum(tt['vote_history'])
            avg_score = sum(tt['score_history']) / total_votes 

            final_is_defect = bool(defect_votes >= (total_votes / 2.0))

            res_msg = InspectionResult()
            res_msg.tile_id = idx + 1
            res_msg.pattern_name = tt['pattern_name']
            res_msg.pose = tt['pose']      
            res_msg.width = tt['width']    
            res_msg.height = tt['height']  


            res_msg.anomaly_score = float(avg_score)
            result_array_msg.results.append(res_msg)

            status = "🚨 최종 폐기" if final_is_defect else "✅ 최종 정상"
            self.get_logger().info(f"[{tt['pattern_name']}] 타일 {idx+1}: {status} | 불량 투표: {defect_votes}/{total_votes}회 (평균 스코어: {avg_score:.3f})")
        
        self.get_logger().info("=========================================")

        result_array_msg.total_tiles = len(result_array_msg.results)
        if result_array_msg.total_tiles > 0:
            self.web_pub.publish(result_array_msg)

    def reset_buffer(self):
        self.current_frame_count = 0
        self.tracked_tiles = []

def main(args=None):
    rclpy.init(args=args)
    node = HybridAnomalibNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("종료 중...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()