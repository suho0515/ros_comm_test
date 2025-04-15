#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from test_msgs.msg import TestMessage
import time
from builtin_interfaces.msg import Time

class TestPublisher(Node):
    def __init__(self):
        super().__init__('python_test_publisher')
        self.publisher_ = self.create_publisher(TestMessage, 'test_topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = TestMessage()
        msg.id = self.count
        msg.message = f'Python 테스트 메시지 #{self.count}'
        msg.data = float(self.count) * 1.5
        msg.stamp = self.get_clock().now().to_msg()
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'발행 중: {msg.message}')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 