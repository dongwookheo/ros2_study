#!/usr/bin/env python3
import asyncio
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from custom_interfaces.action import RecordVideo
from custom_interfaces.srv import RegisterFace
import cv2
import face_recognition
import time
from datetime import datetime
import threading
from queue import Queue
import rclpy.callback_groups
from rclpy.executors import MultiThreadedExecutor


class FaceMovementDetector(Node):
    def __init__(self):
        super().__init__("face_movement_detector")

        # 콜백 그룹과 프레임 큐 추가
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.frame_queue = Queue(maxsize=30)

        # CV Bridge
        self.bridge = CvBridge()

        # 카메라 설정
        self.cap = cv2.VideoCapture(0)

        # 얼굴 데이터베이스
        self.known_faces = {}

        # 토픽 발행자 - 실시간 카메라 피드
        self.image_publisher = self.create_publisher(
            Image, "face_movement_detector/image", 10
        )

        # 타이머 콜백으로 프레임 발행
        self.timer = self.create_timer(
            0.033,
            self.publish_frame,
            callback_group=self.callback_group,
        )

        # 서비스 - 얼굴 등록
        self.face_register_srv = self.create_service(
            RegisterFace,
            "register_face",
            self.register_face_callback,
            callback_group=self.callback_group,
        )

        # 액션 서버 - 움직임 감지 녹화
        self.recording = False
        self.record_server = ActionServer(
            self,
            RecordVideo,
            "record_motion",
            self.record_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info("Face movement detector is running...")

    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            # 얼굴 인식 처리
            face_locations = face_recognition.face_locations(frame)
            for top, right, bottom, left in face_locations:
                cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)

            # 녹화 중이면 프레임을 큐에 추가
            if self.recording:
                if not self.frame_queue.full():
                    self.frame_queue.put(frame.copy())

            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_publisher.publish(msg)

    async def register_face_callback(self, request, response):
        try:
            ret, frame = self.cap.read()
            if ret:
                face_encodings = face_recognition.face_encodings(frame)
                if face_encodings:
                    self.known_faces[request.name] = face_encodings[0]
                    response.success = True
                    response.message = (
                        f"Successfully registered face for {request.name}"
                    )
                else:
                    response.success = False
                    response.message = "No face detected in frame"
            else:
                response.success = False
                response.message = "Failed to capture frame"
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
        return response

    def record_frames(self, duration, goal_handle):
        """별도의 스레드에서 프레임 녹화"""
        feedback_msg = RecordVideo.Feedback()
        result = RecordVideo.Result()

        # 비디오 writer 설정
        fourcc = cv2.VideoWriter_fourcc(*"XVID")
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"motion_{timestamp}.avi"
        video_writer = cv2.VideoWriter(filename, fourcc, 20.0, (640, 480))

        start_time = time.time()
        prev_frame = None
        motion_detected = False

        try:
            while self.recording and (time.time() - start_time) < duration:
                if not self.frame_queue.empty():
                    frame = self.frame_queue.get()

                    if prev_frame is not None:
                        # 움직임 감지 로직
                        diff = cv2.absdiff(prev_frame, frame)
                        gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
                        blur = cv2.GaussianBlur(gray, (5, 5), 0)
                        _, thresh = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY)
                        contours, _ = cv2.findContours(
                            thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
                        )

                        # 움직임이 감지되면
                        if len(contours) > 0 and any(
                            cv2.contourArea(c) > 500 for c in contours
                        ):
                            motion_detected = True
                            # 움직임이 있을 때만 프레임 저장
                            video_writer.write(frame)

                            feedback_msg.motion_detected = True
                            feedback_msg.recording_time = time.time() - start_time
                            goal_handle.publish_feedback(feedback_msg)

                            self.get_logger().info("Motion detected - Recording frame")
                        else:
                            motion_detected = False
                            feedback_msg.motion_detected = False
                            feedback_msg.recording_time = time.time() - start_time
                            goal_handle.publish_feedback(feedback_msg)

                    prev_frame = frame.copy()

        finally:
            video_writer.release()
            result.recorded_file = filename
            result.motion_detected = motion_detected
            self.recording = False
            goal_handle.succeed(result)

    async def record_callback(self, goal_handle):
        self.recording = True

        # 새로운 스레드에서 녹화 시작
        self.recording_thread = threading.Thread(
            target=self.record_frames,
            args=(goal_handle.request.duration, goal_handle),
        )
        self.recording_thread.start()

        # 결과를 기다림
        result = RecordVideo.Result()
        while self.recording:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.recording_thread.join()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = FaceMovementDetector()

    # 멀티 스레드 실행자 사용
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
