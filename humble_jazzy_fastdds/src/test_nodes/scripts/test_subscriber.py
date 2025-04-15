#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from test_msgs.msg import TestMessage
import time
from builtin_interfaces.msg import Time

class TestSubscriber(Node):
    def __init__(self):
        super().__init__('python_test_subscriber')
        self.subscription = self.create_subscription(
            TestMessage,
            'test_topic',
            self.topic_callback,
            10)

    def topic_callback(self, msg):
        self.get_logger().info(
            f'수신 중: ID={msg.id}, 메시지="{msg.message}", 데이터={msg.data:.2f}')
        
        # 지연 시간 계산
        current_time = self.get_clock().now()
        # msg.stamp는 이미 builtin_interfaces.msg.Time 타입
        msg_time = rclpy.time.Time.from_msg(msg.stamp)
        latency = current_time - msg_time
        latency_ms = latency.nanoseconds / 1000000.0
        
        self.get_logger().info(f'지연 시간: {latency_ms:.2f} ms')

def main(args=None):
    rclpy.init(args=args)
    node = TestSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 