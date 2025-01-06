#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from custom_interfaces.action import RecordVideo
from custom_interfaces.srv import RegisterFace
import cv2
import sys
import tkinter as tk
from tkinter import ttk
from PIL import Image as PILImage
from PIL import ImageTk


class DetectorClient(Node):
    def __init__(self):
        super().__init__("detector_client")

        # CV Bridge
        self.bridge = CvBridge()

        # GUI 설정
        self.setup_gui()

        # 이미지 구독
        self.image_subscription = self.create_subscription(
            Image, "face_movement_detector/image", self.image_callback, 10
        )

        # 서비스 클라이언트
        self.face_register_client = self.create_client(RegisterFace, "register_face")

        # 액션 클라이언트
        self.record_client = ActionClient(self, RecordVideo, "record_motion")

        self.get_logger().info("Face movement detector is running...")

    def setup_gui(self):
        self.window = tk.Tk()
        self.window.title("Face Movement Detector")

        # 비디오 프레임
        self.video_label = ttk.Label(self.window)
        self.video_label.grid(row=0, column=0, columnspan=2, padx=10, pady=10)

        # 얼굴 등록 프레임
        register_frame = ttk.LabelFrame(self.window, text="Face Registration")
        register_frame.grid(row=1, column=0, padx=10, pady=5, sticky="ew")

        self.name_entry = ttk.Entry(register_frame)
        self.name_entry.pack(side=tk.LEFT, padx=5, pady=5)

        register_btn = ttk.Button(
            register_frame, text="Register Face", command=self.register_face
        )
        register_btn.pack(side=tk.LEFT, padx=5, pady=5)

        # 녹화 프레임
        record_frame = ttk.LabelFrame(self.window, text="Motion Recording")
        record_frame.grid(row=1, column=1, padx=10, pady=5, sticky="ew")

        self.duration_entry = ttk.Entry(record_frame)
        self.duration_entry.insert(0, "60")
        self.duration_entry.pack(side=tk.LEFT, padx=5, pady=5)

        record_btn = ttk.Button(
            record_frame, text="Start Recording", command=self.start_recording
        )
        record_btn.pack(side=tk.LEFT, padx=5, pady=5)

        # 상태 표시
        self.status_label = ttk.Label(self.window, text="Ready")
        self.status_label.grid(row=2, column=0, columnspan=2, pady=5)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # OpenCV BGR to RGB 변환
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            pil_image = PILImage.fromarray(rgb_image)
            # 이미지 리사이즈
            pil_image = pil_image.resize((640, 480))
            tk_image = ImageTk.PhotoImage(image=pil_image)
            self.video_label.configure(image=tk_image)
            self.video_label.image = tk_image
        except Exception as e:
            self.get_logger().error(f"Error in image callback: {str(e)}")

    def register_face(self):
        name = self.name_entry.get()
        if not name:
            status_msg = "Please enter a name"
            self.status_label["text"] = status_msg
            self.get_logger().warn(status_msg)
            return

        request = RegisterFace.Request()
        request.name = name
        self.get_logger().info(f"Registering face for {name}...")

        future = self.face_register_client.call_async(request)
        future.add_done_callback(self.register_face_callback)

    def register_face_callback(self, future):
        try:
            response = future.result()
            self.status_label["text"] = response.message
            self.get_logger().info(f"Response: {response.message}")
        except Exception as e:
            error_msg = f"Service call failed: {str(e)}"
            self.status_label["text"] = error_msg
            self.get_logger().error(error_msg)

    def start_recording(self):
        try:
            duration = float(self.duration_entry.get())
            goal_msg = RecordVideo.Goal()
            goal_msg.duration = duration

            status_msg = f"Starting recording for {duration}s..."
            self.status_label["text"] = status_msg
            self.get_logger().info(status_msg)

            self.record_client.wait_for_server()

            # 피드백 처리 확인을 위한 로그 추가
            self.get_logger().info("Sending goal with feedback callback...")

            self.send_goal_future = self.record_client.send_goal_async(
                goal_msg, feedback_callback=self.feedback_callback
            )
            self.send_goal_future.add_done_callback(self.goal_response_callback)

            # 피드백 콜백 설정 확인
            self.get_logger().info("Goal sent with feedback callback configured")

        except ValueError:
            error_msg = "Please enter a valid duration"
            self.status_label["text"] = error_msg
            self.get_logger().error(error_msg)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        status_msg = f"Recording: {feedback.recording_time:.1f}s"
        if feedback.motion_detected:
            status_msg += " (Motion Detected!)"

        self.status_label["text"] = status_msg
        self.get_logger().info(status_msg)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            status_msg = "Goal rejected"
            self.status_label["text"] = status_msg
            self.get_logger().warn(status_msg)
            return

        status_msg = "Goal accepted"
        self.status_label["text"] = status_msg
        self.get_logger().info(status_msg)
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status_msg = f"Recording completed: {result.recorded_file}"
        self.status_label["text"] = status_msg
        self.get_logger().info(status_msg)

    def run(self):
        self.window.mainloop()


def main(args=None):
    rclpy.init(args=args)
    client = DetectorClient()

    # GUI 업데이트를 위한 타이머 콜백
    def update_gui():
        rclpy.spin_once(client, timeout_sec=0.1)
        client.window.after(10, update_gui)

    client.window.after(0, update_gui)
    client.run()

    client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
