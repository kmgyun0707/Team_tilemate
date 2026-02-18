import cv2
import numpy as np

def analyze_flatness(image_path):
    """
    접착제 표면의 평탄도를 분석하는 함수
    - 표면 거칠기 (roughness)
    - 높이 차이 (thickness variation)
    - 평탄도 점수 (0-100점)
    """
    
    # 1. 이미지 읽기
    original_img = cv2.imread(image_path)
    if original_img is None:
        print(f"Error: '{image_path}' 이미지를 찾을 수 없습니다!")
        return None
    
    # 2. 이미지 크기 조정
    original_img = cv2.resize(original_img, (800, 800))
    
    # 3. 그레이스케일 변환
    gray = cv2.cvtColor(original_img, cv2.COLOR_BGR2GRAY)
    
    # 4. 접착제 영역 검출 (밝은 부분)
    # 임계값을 높여서 실제 접착제 영역만 검출 (배경 노이즈 제거)
    _, mask = cv2.threshold(gray, 220, 255, cv2.THRESH_BINARY)
    
    # 5. 형태학적 연산으로 노이즈 제거
    kernel = np.ones((7, 7), np.uint8)  # 커널 크기 증가
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)  # 구멍 메우기
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)   # 노이즈 제거
    
    # 외곽 노이즈 추가 제거
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
        # 가장 큰 영역만 유지
        largest_contour = max(contours, key=cv2.contourArea)
        mask_clean = np.zeros_like(mask)
        cv2.drawContours(mask_clean, [largest_contour], -1, 255, -1)
        mask = mask_clean
    
    # 6. 접착제 영역만 추출
    adhesive_region = cv2.bitwise_and(gray, gray, mask=mask)
    
    # 7. 평탄도 분석
    # 7-1. 표준편차 계산 (낮을수록 평탄함)
    adhesive_pixels = gray[mask > 0]
    if len(adhesive_pixels) == 0:
        print("접착제 영역을 찾을 수 없습니다!")
        return None
    
    std_dev = np.std(adhesive_pixels)  # 표준편차
    mean_val = np.mean(adhesive_pixels)  # 평균 밝기
    
    # 7-2. 엣지 검출 (경계선이 많을수록 울퉁불퉁함)
    edges = cv2.Canny(adhesive_region, 50, 150)
    edge_density = np.count_nonzero(edges) / np.count_nonzero(mask) * 100
    
    # 7-3. 라플라시안 분산 (표면 변화율 측정)
    laplacian = cv2.Laplacian(adhesive_region, cv2.CV_64F)
    laplacian_var = laplacian.var()
    
    # 8. 평탄도 점수 계산 (0-100점, 높을수록 평탄함)
    # 변동 계수(Coefficient of Variation) 방식 사용
    
    # 8-1. 변동 계수 계산 (상대적 변동성)
    # CV = (표준편차 / 평균) × 100
    if mean_val > 0:
        cv_value = (std_dev / mean_val) * 100
    else:
        cv_value = 100  # 평균이 0이면 최악
    
    # 8-2. CV를 점수로 변환 (0-100점)
    # 일반적으로 CV: 2~30% 범위
    # CV가 낮을수록 평탄함
    if cv_value <= 2:
        flatness_score = 100  # 거의 완벽
    elif cv_value <= 5:
        flatness_score = 95 - (cv_value - 2) * 5
    elif cv_value <= 10:
        flatness_score = 80 - (cv_value - 5) * 3
    elif cv_value <= 20:
        flatness_score = 65 - (cv_value - 10) * 2
    elif cv_value <= 30:
        flatness_score = 45 - (cv_value - 20) * 1.5
    else:
        flatness_score = max(0, 30 - (cv_value - 30) * 0.5)
    
    flatness_score = max(0, min(100, flatness_score))  # 0-100 범위로 제한
    
    # 9. 평탄도 등급 판정
    if flatness_score >= 85:
        grade = "S (거의 완벽한 평면)"
        color = (0, 255, 0)  # 초록
        status = "EXCELLENT"
    elif flatness_score >= 70:
        grade = "A (매우 평탄함)"
        color = (0, 255, 200)  # 연두
        status = "VERY GOOD"
    elif flatness_score >= 55:
        grade = "B (양호)"
        color = (0, 200, 255)  # 노랑
        status = "GOOD"
    elif flatness_score >= 40:
        grade = "C (약간 울퉁불퉁)"
        color = (0, 165, 255)  # 주황
        status = "FAIR"
    elif flatness_score >= 25:
        grade = "D (울퉁불퉁함)"
        color = (0, 100, 255)  # 진한 주황
        status = "POOR"
    else:
        grade = "F (매우 불량)"
        color = (0, 0, 255)  # 빨강
        status = "VERY POOR"
    
    # 10. 시각화
    # 10-1. 원본 이미지
    result_original = original_img.copy()
    
    # 10-2. 마스크 오버레이
    result_mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    
    # 10-3. 높이 맵 (히트맵) - 먼저 생성
    # 밝기를 높이로 해석하여 컬러맵 적용
    # 밝은 부분(튀어나온 부분) = 빨강, 어두운 부분(평탄한 부분) = 파랑
    adhesive_colored = adhesive_region.copy()
    
    # 반전: 밝은 부분을 높은 값으로 변환
    adhesive_inverted = 255 - adhesive_colored
    adhesive_inverted[mask == 0] = 0  # 배경은 0으로
    
    # JET 컬러맵 적용 (파랑->초록->노랑->빨강)
    adhesive_colored = cv2.applyColorMap(adhesive_inverted, cv2.COLORMAP_JET)
    adhesive_colored[mask == 0] = [0, 0, 0]  # 배경은 검정
    
    # 10-4. 세로 단면 프로파일 (Height Profile)
    # 접착제 영역을 여러 세로 라인으로 잘라서 높이 표시
    result_profile = adhesive_colored.copy()
    
    # 접착제 영역의 경계 찾기
    ys, xs = np.where(mask > 0)
    if len(xs) > 0:
        x_min, x_max = xs.min(), xs.max()
        y_min, y_max = ys.min(), ys.max()
        
        # 세로 라인 개수 (8-12개 정도)
        num_lines = 10
        x_positions = np.linspace(x_min + 30, x_max - 30, num_lines, dtype=int)
        
        # 각 세로 위치에서 수직선 그리기
        for x_pos in x_positions:
            # 해당 x 위치에서 위에서 아래로 스캔하며 접착제 영역 찾기
            for y in range(y_min, y_max):
                if mask[y, x_pos] > 0:  # 접착제 영역이면
                    # 해당 위치의 밝기 가져오기
                    brightness = adhesive_region[y, x_pos]
                    
                    # 밝기에 따라 색상 결정 (밝을수록 더 밝게)
                    color_intensity = int((brightness / 255.0) * 255)
                    # 흰색 선으로 그리기
                    cv2.circle(result_profile, (x_pos, y), 1, (255, 255, 255), -1)
            
            # 전체 수직선 그리기
            y_start, y_end = None, None
            for y in range(y_min, y_max):
                if mask[y, x_pos] > 0:
                    if y_start is None:
                        y_start = y
                    y_end = y
            
            if y_start is not None and y_end is not None:
                cv2.line(result_profile, (x_pos, y_start), (x_pos, y_end), (255, 255, 255), 3)
    
    result_contour_bgr = result_profile
    
    # 11. 결과 텍스트 추가
    cv2.putText(result_original, f"Flatness: {flatness_score:.1f}/100", (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3)
    cv2.putText(result_original, f"{status}", (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3)
    cv2.putText(result_original, f"Grade: {grade}", (10, 140),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
    cv2.putText(result_original, f"CV: {cv_value:.2f}% | Mean: {mean_val:.1f}", (10, 740),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(result_original, f"Std Dev: {std_dev:.2f}", (10, 770),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    # 히트맵에 텍스트
    cv2.putText(adhesive_colored, "Height Map", (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 3)
    # 범례 추가
    cv2.putText(adhesive_colored, "Blue=Flat | Red=High", (10, 770),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    # 높이 프로파일 이미지에 텍스트
    cv2.putText(result_contour_bgr, "Height Profile", (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 3)
    
    # 12. 결과 출력
    print("="*60)
    print("[평탄도 분석 결과]")
    print(f"평탄도 점수: {flatness_score:.1f}/100")
    print(f"상태: {status}")
    print(f"등급: {grade}")
    print("-"*60)
    print("[세부 지표 - 변동 계수(CV) 방식]")
    print(f"변동 계수(CV): {cv_value:.2f}% (낮을수록 평탄)")
    print(f"표준편차: {std_dev:.2f}")
    print(f"평균 밝기: {mean_val:.2f}")
    print(f"엣지 밀도: {edge_density:.2f}% (참고용)")
    print("-"*60)
    print("[해석]")
    if cv_value <= 2:
        print("✓✓ 거의 완벽한 평탄도! (CV ≤ 2%)")
    elif cv_value <= 10:
        print("✓ 매우 우수한 평탄도 (CV ≤ 10%)")
    elif cv_value <= 15:
        print("○ 양호한 평탄도 (CV ≤ 15%)")
    elif cv_value <= 20:
        print("△ 약간의 변동성 있음 (CV ≤ 20%)")
    else:
        print("✗ 표면 변동성이 큽니다. 재도포 권장 (CV > 20%)")
    print("="*60)
    
    # 13. 결과 이미지 - 높이 맵만 표시
    final_result = adhesive_colored
    
    return final_result, flatness_score, grade

# --- 실행 부분 ---

image_path = "/home/sa/cobot_ws/src/cobot1/glue_img/sim2.jpg"

print("=== 접착제 표면 평탄도 분석 시작 ===\n")

try:
    result, score, grade = analyze_flatness(image_path)
    
    if result is not None:
        # 결과 창 표시
        cv2.imshow("Flatness Analysis - Press any key to close", result)
        print("\n높이 맵 결과 창이 열렸습니다.")
        print("아무 키나 누르면 종료됩니다.")
        
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("분석에 실패했습니다.")

except Exception as e:
    print(f"에러 발생: {e}")