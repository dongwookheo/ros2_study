#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

# 디폴트 QoS 프로파일
from rclpy.qos import (
    qos_profile_sensor_data,
    qos_profile_services_default,
    # ...
)

# QoS 프로파일을 사용하기 위한 import
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,  # Subscribtion 생성 전, 데이터 처리에 관한 정책
    QoSHistoryPolicy,  # 메시지를 얼마나 저장할 것인지에 관한 정책
    QoSReliabilityPolicy,  # 메시지 전달을 중시할 것인지, 전송 속도를 중시할 것인지에 관한 정책
    QoSLivelinessPolicy,  # 노드의 살아있음을 자동으로 감지할 것인지, 수동으로 감지할 것인지에 관한 정책
)

# Deadline  # 정해진 주기 내에 메시지가 발신 및 수신되지 않으면 event 발생
# Lifespan  # 메시지의 유효기간 설정
from rclpy.duration import Duration


class PoseTalker(Node):
    def __init__(self):
        super().__init__("pose_talker")

        # QoS 프로파일 설정
        qos_profile = QoSProfile(
            depth=10,  # 메시지 큐의 최대 길이
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  #
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
            deadline=Duration(seconds=2),
            lifespan=Duration(seconds=5),
        )

        # Publisher 생성
        self.publisher = self.create_publisher(PoseStamped, "pose_topic", qos_profile)

        # 0.5초마다 타이머 콜백 실행
        self.timer = self.create_timer(0.5, self.timer_callback)

        # position 값을 추적하기 위한 카운터
        self.counter = 0
        self.get_logger().info("Pose Talker node has been started.")

    def timer_callback(self):
        msg = PoseStamped()

        # 현재 시간 설정
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        # position 값 설정 (매 콜백마다 1씩 증가)
        msg.pose.position.x = float(self.counter)
        msg.pose.position.y = float(self.counter)
        msg.pose.position.z = float(self.counter)

        # orientation은 기본값 사용 (쿼터니언)
        msg.pose.orientation.w = 1.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0

        # 메시지 발행
        self.publisher.publish(msg)
        self.get_logger().info(
            f"Publishing: x={self.counter}, y={self.counter}, z={self.counter}"
        )

        # 카운터 증가
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = PoseTalker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
