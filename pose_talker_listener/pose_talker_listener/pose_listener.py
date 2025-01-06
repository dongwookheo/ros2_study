#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile

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


class PoseListener(Node):
    def __init__(self):
        super().__init__("pose_listener")

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

        # Subscriber 생성
        self.subscription = self.create_subscription(
            PoseStamped, "pose_topic", self.listener_callback, qos_profile
        )
        self.get_logger().info("Pose Listener node has been started.")

    def listener_callback(self, msg):
        self.get_logger().info(
            f"Received Pose: Position(x={msg.pose.position.x:.2f}, "
            f"y={msg.pose.position.y:.2f}, z={msg.pose.position.z:.2f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PoseListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
