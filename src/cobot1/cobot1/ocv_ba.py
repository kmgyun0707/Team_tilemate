import cv2
import numpy as np

def analyze_real_photo(image_path, label_text="Image"):
    # 1. 이미지 읽기
    original_img = cv2.imread(image_path) # 이미지 파일 경로를 입력받아 읽어옴
    if original_img is None:
        print(f"Error: '{image_path}' 이미지를 찾을 수 없습니다!")
        return None, None

    # 2. 이미지 크기 통일 (비교를 위해 필수)
    # 두 사진의 해상도가 다를 수 있으므로 640x640(또는 원하는 크기)으로 리사이징
    target_size = (640, 640) 
    original_img = cv2.resize(original_img, target_size)

    # 3. 관심 영역(ROI) 자동 검출 - 검은색 마카 테두리 찾기
    # 먼저 그레이스케일로 변환
    gray_for_detection = cv2.cvtColor(original_img, cv2.COLOR_BGR2GRAY)
    
    # 검은색 테두리 검출을 위한 이진화 (낮은 임계값으로 어두운 부분 찾기)
    _, dark_binary = cv2.threshold(gray_for_detection, 100, 255, cv2.THRESH_BINARY_INV)
    
    # 윤곽선 찾기
    contours, _ = cv2.findContours(dark_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # 가장 큰 사각형 윤곽선 찾기
    roi_img = None
    for contour in sorted(contours, key=cv2.contourArea, reverse=True):
        # 윤곽선을 사각형으로 근사화
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        
        # 사각형(4개 꼭짓점)이고 충분히 큰 경우
        if len(approx) == 4 and cv2.contourArea(contour) > 10000:
            x, y, w, h = cv2.boundingRect(contour)
            # 안쪽 여백 10픽셀 제거
            roi_img = original_img[y+10:y+h-10, x+10:x+w-10]
            break
    
    # 검은 테두리를 찾지 못한 경우 기존 방식 사용
    if roi_img is None:
        print(f"Warning: '{image_path}'에서 검은 테두리를 찾지 못했습니다. 기본 ROI 사용.")
        h, w = original_img.shape[:2]
        roi_img = original_img[20:h-20, 20:w-20]
    
    # ROI 크기를 600x600으로 통일 (시각화 시 크기 맞추기 위함)
    roi_img = cv2.resize(roi_img, (600, 600)) 

    # 4. 전처리: 그레이스케일 & 블러링
    gray = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # 5. 이진화 (Thresholding)
    # 검은색 배경에 하얀색 접착제이므로, THRESH_BINARY를 사용
    # 반사광을 제거하기 위해 높은 임계값(200) 설정
    # 배경(검은색) -> 0(검정), 접착제(흰색) -> 255(흰색)으로 변환
    _, binary = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)

    # 6. 형태학적 연산 (노이즈 제거 및 구멍 메우기)
    kernel = np.ones((3, 3), np.uint8)
    processed_binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel) # 노이즈 제거
    processed_binary = cv2.morphologyEx(processed_binary, cv2.MORPH_CLOSE, kernel) # 끊김 연결

    # 7. 커버리지 계산
    total_pixels = processed_binary.size # ROI 영역의 총 픽셀 수
    adhesive_pixels = cv2.countNonZero(processed_binary) # 접착제(흰색) 픽셀 수
    coverage = (adhesive_pixels / total_pixels) * 100 # 백분율로 계산

    # 8. 결과 시각화
    # 원본(ROI)에 초록색 덧칠
    display_img = roi_img.copy()
    display_img[processed_binary == 255] = [0, 255, 0] # 접착제 부분을 초록색으로

    # 결과 텍스트 이미지 위에 적기
    cv2.putText(display_img, f"{label_text}: {coverage:.1f}%", (10, 40), 
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)

    return coverage, display_img

# --- 실행 부분 ---

# 1. BEFORE 이미지 3개
before_1 = "/home/sa/cobot_ws/src/cobot1/glue_img/b1.jpg"
before_2 = "/home/sa/cobot_ws/src/cobot1/glue_img/b2.jpg"
before_3 = "/home/sa/cobot_ws/src/cobot1/glue_img/b3.jpg"

# 2. AFTER 이미지 3개
after_1 = "/home/sa/cobot_ws/src/cobot1/glue_img/a1.jpg"
after_2 = "/home/sa/cobot_ws/src/cobot1/glue_img/a2.jpg"
after_3 = "/home/sa/cobot_ws/src/cobot1/glue_img/a3.jpg"

print("=== 접착제 도포 개선 분석 (BEFORE vs AFTER) ===\n")

try:
    # 3. BEFORE 분석
    print("[BEFORE 분석]")
    score_b1, result_b1 = analyze_real_photo(before_1, "BEFORE-1")
    score_b2, result_b2 = analyze_real_photo(before_2, "BEFORE-2")
    score_b3, result_b3 = analyze_real_photo(before_3, "BEFORE-3")
    
    # 4. AFTER 분석
    print("[AFTER 분석]")
    score_a1, result_a1 = analyze_real_photo(after_1, "AFTER-1")
    score_a2, result_a2 = analyze_real_photo(after_2, "AFTER-2")
    score_a3, result_a3 = analyze_real_photo(after_3, "AFTER-3")

    # 수정된 부분: all() 대신 개별 None 체크
    if result_b1 is not None and result_b2 is not None and result_b3 is not None and \
       result_a1 is not None and result_a2 is not None and result_a3 is not None:
        # 5. 평균 계산
        avg_before = (score_b1 + score_b2 + score_b3) / 3
        avg_after = (score_a1 + score_a2 + score_a3) / 3
        
        # 6. 결과 출력
        print("\n" + "="*60)
        print("[BEFORE 도포율]")
        print(f"ㄹ형: {score_b1:.2f}%")
        print(f"사선형: {score_b2:.2f}%")
        print(f"나선형: {score_b3:.2f}%")
        
        print("\n[AFTER 도포율]")
        print(f"  ㄹ형: {score_a1:.2f}%")
        print(f"  사선형: {score_a2:.2f}%")
        print(f"  나선형: {score_a3:.2f}%")
    
        print("="*60)
        
        # 7. 개별 비교
        print("\n[개별 이미지 비교]")
        comparisons = [
            ("ㄹ형", score_b1, score_a1),
            ("사선형", score_b2, score_a2),
            ("나선형", score_b3, score_a3)
        ]
        
        for name, before, after in comparisons:
            diff = after - before
            rate = (diff / before) * 100 if before > 0 else 0
            print(f"  {name}: {before:.2f}% → {after:.2f}% ({diff:+.2f}%p, {rate:+.1f}%)")
        
        # 8. 시각화 - BEFORE 3개 (위)
        before_view = np.hstack((result_b1, result_b2, result_b3))
        
        # 9. 시각화 - AFTER 3개 (아래)
        after_view = np.hstack((result_a1, result_a2, result_a3))
        
        # 10. BEFORE와 AFTER를 세로로 결합
        final_view = np.vstack((before_view, after_view))
        
        cv2.imshow("BEFORE (위) vs AFTER (아래)", final_view)
        
        print("\n결과 창이 열렸습니다.")
        print("- 위: BEFORE (Method 1, 2, 3)")
        print("- 아래: AFTER (Method 1, 2, 3)")
        print("아무 키나 누르면 종료됩니다.")
        
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("이미지 처리에 실패했습니다. 파일 경로를 확인해주세요.")

except Exception as e:
    print(f"에러 발생: {e}")