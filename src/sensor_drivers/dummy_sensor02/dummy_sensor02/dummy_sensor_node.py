#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_interfaces.msg import SensorData
import random

class DummySensor(Node):
    def __init__(self):
        super().__init__('dummy_sensor_node')
        self.publisher_ = self.create_publisher(SensorData, '/sensor/data2', 10)
        timer_period = 0.1  # 0.1秒ごとにセンサーデータを発行（10Hz）
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = SensorData()
        msg.value = random.uniform(0.0, 10.0)
        self.publisher_.publish(msg)
        # self.get_logger().info(f'Published sensor value: {msg.value:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = DummySensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()