import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage

class HSVTuner:
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node('hsv_tuner', anonymous=True)

        # Subscriber 설정
        self.image_sub = rospy.Subscriber('/usb_cam/image_rect_color/compressed', CompressedImage, self.image_callback)
        
        # HSV 범위 초기값 설정
        self.hsv_lower = np.array([0, 30, 30])
        self.hsv_upper = np.array([179, 255, 255])

        # 이진화 임계값 초기값 설정
        self.binary_threshold = 127

        # OpenCV 창 생성 및 트랙바 초기화
        cv2.namedWindow('Original and Mask', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Original and Mask', 800, 600)

        cv2.namedWindow('HSV Tuner', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('HSV Tuner', 400, 300)

        # HSV 트랙바 생성
        cv2.createTrackbar('H Lower', 'HSV Tuner', 0, 179, self.update_hsv)
        cv2.createTrackbar('H Upper', 'HSV Tuner', 179, 179, self.update_hsv)
        cv2.createTrackbar('S Lower', 'HSV Tuner', 30, 255, self.update_hsv)
        cv2.createTrackbar('S Upper', 'HSV Tuner', 255, 255, self.update_hsv)
        cv2.createTrackbar('V Lower', 'HSV Tuner', 30, 255, self.update_hsv)
        cv2.createTrackbar('V Upper', 'HSV Tuner', 255, 255, self.update_hsv)

        # 이진화 임계값 트랙바 생성
        cv2.createTrackbar('Binary Threshold', 'HSV Tuner', 127, 255, self.update_threshold)

        # 트랙바 초기값 설정
        self.update_hsv(0)
        self.update_threshold(0)

        # 이미지 저장
        self.current_image = None

    def update_hsv(self, value):
        """HSV 트랙바 값 업데이트"""
        self.hsv_lower = np.array([
            cv2.getTrackbarPos('H Lower', 'HSV Tuner'),
            cv2.getTrackbarPos('S Lower', 'HSV Tuner'),
            cv2.getTrackbarPos('V Lower', 'HSV Tuner')
        ])
        self.hsv_upper = np.array([
            cv2.getTrackbarPos('H Upper', 'HSV Tuner'),
            cv2.getTrackbarPos('S Upper', 'HSV Tuner'),
            cv2.getTrackbarPos('V Upper', 'HSV Tuner')
        ])

    def update_threshold(self, value):
        """이진화 임계값 업데이트"""
        self.binary_threshold = cv2.getTrackbarPos('Binary Threshold', 'HSV Tuner')

    def image_callback(self, msg):
        """ROS 이미지 콜백 함수"""
        try:
            # ROS CompressedImage -> OpenCV BGR 이미지 변환
            np_arr = np.frombuffer(msg.data, np.uint8)
            bgr_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if bgr_image is None:
                rospy.logerr("Failed to decode image")
                return

            # 이미지 저장
            self.current_image = bgr_image

        except Exception as e:
            rospy.logerr(f"Error in image callback: {e}")

    def process_image(self):
        """HSV 변환 및 필터링, 이진화"""
        if self.current_image is None:
            return

        try:
            # BGR -> HSV 변환
            hsv_image = cv2.cvtColor(self.current_image, cv2.COLOR_BGR2HSV)

            # HSV 범위 필터링
            mask = cv2.inRange(hsv_image, self.hsv_lower, self.hsv_upper)

            # HSV 필터링 결과를 이진화
            _, binary_image = cv2.threshold(mask, self.binary_threshold, 255, cv2.THRESH_BINARY)

            # 결과 표시 (원본, HSV 필터링, 이진화된 결과)
            combined = np.hstack((self.current_image, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), cv2.cvtColor(binary_image, cv2.COLOR_GRAY2BGR)))
            cv2.imshow('Original and Mask', combined)

        except Exception as e:
            rospy.logerr(f"Error in process_image: {e}")

    def run(self):
        """ROS와 OpenCV 이벤트 루프 실행"""
        while not rospy.is_shutdown():
            self.process_image()
            key = cv2.waitKey(1)
            if key == 27:  # ESC 키로 종료
                rospy.signal_shutdown('User Exit')
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        tuner = HSVTuner()
        tuner.run()
    except rospy.ROSInterruptException:
        pass
