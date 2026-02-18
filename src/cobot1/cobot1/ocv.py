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

    # 3. 관심 영역(ROI) 설정
    # 테두리나 배경 노이즈를 피하기 위해 가장자리 20픽셀씩 잘라냄 (필요 시 조절)
    h, w = original_img.shape[:2]
    roi_img = original_img[20:h-20, 20:w-20] 

    # 4. 전처리: 그레이스케일 & 블러링
    gray = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # 5. 이진화 (Thresholding)
    # 검은색 배경에 하얀색 접착제이므로, THRESH_BINARY를 사용
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
    cv2.putText(display_img, f"{label_text}: {coverage:.1f}%", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    return coverage, display_img

# --- 실행 부분 ---

# 1. 비교할 세 이미지 파일명 입력
file_1 = "/home/sa/cobot_ws/src/cobot1/glue_img/a1.jpg"     # 첫 번째 사진
file_2 = "/home/sa/cobot_ws/src/cobot1/glue_img/a2.jpg"     # 두 번째 사진
file_3 = "/home/sa/cobot_ws/src/cobot1/glue_img/a3.jpg"     # 세 번째 사진

print("=== 접착제 도포 비교 분석 시작 (3개 이미지) ===")

try:
    # 2. 각각 분석 수행
    score1, result1 = analyze_real_photo(file_1, "Method A")
    score2, result2 = analyze_real_photo(file_2, "Method B")
    score3, result3 = analyze_real_photo(file_3, "Method C")

    if result1 is not None and result2 is not None and result3 is not None:
        # 3. 콘솔 결과 출력
        print(f"\n[분석 결과]")
        print(f"이미지 1 (Method A - ㄹ형): {score1:.2f}%")
        print(f"이미지 2 (Method B - 사선형): {score2:.2f}%")
        print(f"이미지 3 (Method C - 나선형): {score3:.2f}%")
        
        # 최대/최소값 찾기
        scores = [("Method A", score1), ("Method B", score2), ("Method C", score3)]
        best = max(scores, key=lambda x: x[1])
        worst = min(scores, key=lambda x: x[1])
        
        print(f"\n-> 최고 도포율: {best[0]} ({best[1]:.2f}%)")
        print(f"-> 최저 도포율: {worst[0]} ({worst[1]:.2f}%)")
        print(f"-> 차이: {best[1] - worst[1]:.2f}% 포인트")

        # 4. 이미지 가로로 붙이기 (Stacking)
        # 3개의 결과 이미지를 나란히 붙여서 한 창에 띄움
        final_view = np.hstack((result1, result2, result3))

        cv2.imshow("Comparison: Method A vs Method B vs Method C", final_view)
        print("\n결과 창이 열렸습니다. 아무 키나 누르면 종료됩니다.")
        
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("이미지 처리에 실패했습니다. 파일 경로를 확인해주세요.")

except Exception as e:
    print(f"에러 발생: {e}")