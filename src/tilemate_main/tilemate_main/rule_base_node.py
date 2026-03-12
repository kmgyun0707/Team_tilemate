import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import math # 🌟 타일 위치(거리) 계산을 위해 추가
from ament_index_python.packages import get_package_share_directory 

# 커스텀 메시지 배열 임포트
from custom_tile_msgs.msg import TileArray, InspectionResult, InspectionResultArray

class RuleBasedAnomalyNode(Node):
    def __init__(self):
        super().__init__('rule_based_anomaly_node')
        self.bridge = CvBridge()

        # ==========================================
        # 💡 1. 세팅 및 9개 패턴별 안전지대 생성
        # ==========================================  

        # 동적 홈 디렉토리 경로 가져오기 
        package_name = 'tilemate_main' 
        
        # 패키지의 share 디렉토리 경로를 동적으로 가져옴
        pkg_share_dir = get_package_share_directory(package_name)
        img_base_dir = os.path.join(pkg_share_dir, 'rule_based_img')

        self.ref_image_paths = {
            f"pattern_{i}": os.path.join(img_base_dir, f"ref_pattern_{i}.jpg") 
            for i in range(1, 10)
        }
    
        # 생성된 안전지대를 저장할 딕셔너리
        self.safe_zones = {}
        kernel = np.ones((11,11), np.uint8) 
        self.pixel_threshold = 60

        # 딕셔너리를 돌면서 각 패턴별로 Canny 엣지 + 팽창(Dilate) 수행
        for pattern_name, img_path in self.ref_image_paths.items():
            if not os.path.exists(img_path):
                self.get_logger().warn(f"🚨 기준 이미지 없음: {img_path} ({pattern_name} 검사 불가)")
                continue
            
            ref_img = cv2.imread(img_path)
            ref_edges = self.get_canny_edges(ref_img)
            self.safe_zones[pattern_name] = cv2.dilate(ref_edges, kernel, iterations=1)

        self.get_logger().info(f"✅ {len(self.safe_zones)}개 패턴의 안전지대 로드 완료! (허용 기준: {self.pixel_threshold}픽셀)")

        # ==========================================
        # 🔥 2. 프레임 누적(버퍼) 트래킹 변수 세팅
        # ==========================================
        self.TARGET_FRAMES = 60          # 누적 검사할 프레임 수
        self.current_frame_count = 0     # 현재 누적된 프레임
        self.tracked_tiles = []          # 타일별 이력(History)을 저장할 버퍼
        self.MATCH_DIST_THRESHOLD = 50.0 # 이전 프레임 타일과 동일 타일로 간주할 픽셀 거리 오차

        # 3. 통신망 세팅
        self.tile_sub = self.create_subscription(TileArray, '/yolo/tile_array', self.inference_callback, 10)
        self.web_pub = self.create_publisher(InspectionResultArray, '/web/inspection_results', 10)
        self.debug_pub = self.create_publisher(Image, '/anomaly/debug_rule_based', 10)

    # 🛠️ Canny 전처리 함수 (유지)
    def get_canny_edges(self, cv_img):
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5,5), 0)
        clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8)) 
        gray_clahe = clahe.apply(blurred) 
        edges = cv2.Canny(gray_clahe, 35, 95) 

        h, w = edges.shape
        cv2.rectangle(edges, (0, 0), (w - 1, h - 1), 0, thickness=5)
        return edges

    def inference_callback(self, msg):
        try:
            # 🌟 이미 목표 프레임을 다 채웠다면, 다음 촬영 명령(버퍼 초기화)이 올 때까지 대기
            if self.current_frame_count >= self.TARGET_FRAMES:
                return

            for tile in msg.tiles:
                current_pattern = tile.pattern_name
                
                if current_pattern not in self.safe_zones:
                    self.get_logger().warn(f"⚠️ 등록되지 않은 패턴({current_pattern}) 감지! 검사 생략.")
                    continue

                target_safe_zone = self.safe_zones[current_pattern]

                # 1. 이미지 변환 및 리사이즈
                curr_img = self.bridge.imgmsg_to_cv2(tile.cropped_image, desired_encoding='bgr8')
                curr_img = cv2.resize(curr_img, (target_safe_zone.shape[1], target_safe_zone.shape[0]))

                # 2. 엣지 추출 및 빼기 연산
                curr_edges = self.get_canny_edges(curr_img)
                defect_edges = cv2.subtract(curr_edges, target_safe_zone)

                # 3. 단일 프레임 불량 판정
                defect_pixel_count = int(np.sum(defect_edges > 0))
                is_defect = bool(defect_pixel_count > self.pixel_threshold)

                # ==========================================
                # 🔥 4. 위치 기반 트래킹 및 버퍼에 저장
                # ==========================================
                matched = False
                for tt in self.tracked_tiles:
                    # 현재 타일이 버퍼 안의 타일 중 어느 것과 같은지 '거리(distance)'로 매칭
                    dist = math.hypot(tt['pose'].x - tile.pose.x, tt['pose'].y - tile.pose.y)
                    
                    if dist < self.MATCH_DIST_THRESHOLD: 
                        tt['crack_history'].append(defect_pixel_count)
                        tt['vote_history'].append(is_defect)
                        tt['pose'] = tile.pose # 최신 좌표로 갱신
                        matched = True
                        break

                # 처음 보는 위치의 타일이라면 버퍼에 새로 등록
                if not matched:
                    self.tracked_tiles.append({
                        'pose': tile.pose,
                        'pattern_name': current_pattern,
                        'width': tile.width,
                        'height': tile.height,
                        'crack_history': [defect_pixel_count],
                        'vote_history': [is_defect]
                    })

                # 시각화 (디버깅용: 10프레임 중 마지막 프레임에만 화면 발행)
                if self.current_frame_count == self.TARGET_FRAMES - 1:
                    curr_img_debug = curr_img.copy()
                    curr_img_debug[defect_edges > 0] = [0, 0, 255] 
                    debug_msg = self.bridge.cv2_to_imgmsg(curr_img_debug, encoding="bgr8")
                    self.debug_pub.publish(debug_msg)

            # 한 프레임 처리가 끝났으므로 카운트 증가
            self.current_frame_count += 1
            self.get_logger().info(f"⏳ 데이터 누적 중... ({self.current_frame_count}/{self.TARGET_FRAMES} 프레임)")

            # ==========================================
            # 🏆 5. 목표 프레임 도달 시: 다수결 판정 후 웹으로 전송
            # ==========================================
            if self.current_frame_count >= self.TARGET_FRAMES:
                self.publish_final_results(msg.header)
                self.reset_buffer() # 🌟 전송 후 바로 초기화하여 다음 촬영 준비!

        except Exception as e:
            self.get_logger().error(f"🚨 룰 베이스 추론 중 에러: {e}")

    def publish_final_results(self, header):
        result_array_msg = InspectionResultArray()
        result_array_msg.header = header
        
        self.get_logger().info("=========================================")
        self.get_logger().info(f"🎯 {self.TARGET_FRAMES}프레임 다수결 검사 완료! (결과 웹 전송)")

        for idx, tt in enumerate(self.tracked_tiles):
            # 노이즈로 잠깐 나타났다 사라진 유령 타일 무시 (예: 10프레임 중 5프레임 미만으로 보인 경우)
            if len(tt['vote_history']) < (self.TARGET_FRAMES / 2):
                continue

            # 🔥 다수결 로직 적용
            total_votes = len(tt['vote_history'])
            defect_votes = sum(tt['vote_history'])
            avg_cracks = sum(tt['crack_history']) / total_votes 

            # 불량 판정이 절반 이상이면 최종 불량 확정
            final_is_defect = bool(defect_votes >= (total_votes / 2.0))

            # 웹으로 보낼 최종 메시지 폼 작성
            res_msg = InspectionResult()
            res_msg.tile_id = idx + 1
            res_msg.pattern_name = tt['pattern_name']
            res_msg.pose = tt['pose']      
            res_msg.width = tt['width']    
            res_msg.height = tt['height']  
            res_msg.is_defective = final_is_defect
            res_msg.defect_type = "Crack" if final_is_defect else "None"

            result_array_msg.results.append(res_msg)

            status = "🚨 최종 폐기" if final_is_defect else "✅ 최종 정상"
            self.get_logger().info(f"[{tt['pattern_name']}] 타일 {idx+1}: {status} | 불량 판정: {defect_votes}/{total_votes}회 (평균 크랙: {avg_cracks:.1f}px)")
        
        self.get_logger().info("=========================================")

        # 검사된 타일 총 개수 입력 후 발행
        result_array_msg.total_tiles = len(result_array_msg.results)
        if result_array_msg.total_tiles > 0:
            self.web_pub.publish(result_array_msg)

    def reset_buffer(self):
        # 검사가 끝났으므로 변수 초기화. 이제 다음 타일 무더기가 화면에 잡히면 다시 0부터 10까지 센다.
        self.current_frame_count = 0
        self.tracked_tiles = []

def main(args=None):
    rclpy.init(args=args)
    node = RuleBasedAnomalyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("종료 중...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()