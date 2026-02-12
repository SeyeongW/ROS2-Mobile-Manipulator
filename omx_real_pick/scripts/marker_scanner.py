import cv2
import cv2.aruco as aruco
import sys

# 테스트할 마커 사전 리스트
ARUCO_DICT = {
    "DICT_4X4_50": aruco.DICT_4X4_50,
    "DICT_4X4_100": aruco.DICT_4X4_100,
    "DICT_4X4_250": aruco.DICT_4X4_250,
    "DICT_4X4_1000": aruco.DICT_4X4_1000,
    "DICT_5X5_50": aruco.DICT_5X5_50,
    "DICT_5X5_100": aruco.DICT_5X5_100,
    "DICT_5X5_250": aruco.DICT_5X5_250,
    "DICT_5X5_1000": aruco.DICT_5X5_1000,
    "DICT_6X6_50": aruco.DICT_6X6_50,
    "DICT_6X6_100": aruco.DICT_6X6_100,
    "DICT_6X6_250": aruco.DICT_6X6_250,
    "DICT_6X6_1000": aruco.DICT_6X6_1000,
    "DICT_7X7_50": aruco.DICT_7X7_50,
    "DICT_7X7_100": aruco.DICT_7X7_100,
    "DICT_7X7_250": aruco.DICT_7X7_250,
    "DICT_7X7_1000": aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": aruco.DICT_APRILTAG_36h11
}

def main():
    cap = cv2.VideoCapture(4) # /dev/video4
    
    # 윈도우 크기 조절 가능하게 설정
    cv2.namedWindow("ArUco Scanner", cv2.WINDOW_NORMAL) 

    if not cap.isOpened():
        print("Cannot open camera")
        sys.exit()

    print("Scanning for markers... (Press 'q' to quit)")

    while True:
        ret, frame = cap.read()
        if not ret: break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        detected_info = []

        # 모든 사전을 순회하며 탐지 시도
        for name, dict_enum in ARUCO_DICT.items():
            aruco_dict = aruco.getPredefinedDictionary(dict_enum)
            parameters = aruco.DetectorParameters()
            
            # [중요] 최신 OpenCV 문법 적용
            detector = aruco.ArucoDetector(aruco_dict, parameters)
            corners, ids, _ = detector.detectMarkers(gray)

            if ids is not None:
                # 탐지되면 초록색 박스 그림
                aruco.drawDetectedMarkers(frame, corners, ids)
                detected_info.append(f"{name} -> ID: {ids.flatten()}")

        # 화면에 결과 텍스트 출력
        y_offset = 30
        if detected_info:
            cv2.putText(frame, "DETECTED!", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            for info in detected_info:
                cv2.putText(frame, info, (10, y_offset + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                y_offset += 30
        else:
            cv2.putText(frame, "No Marker Found", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow("ArUco Scanner", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()