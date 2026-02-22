#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gpiozero import LED
from sensor_interfaces.msg import SensorData
from sensor_interfaces.msg import SensorData2

class LEDController(Node):
    def __init__(self):
        super().__init__('led_node')
        self.led = LED(17)
        
        # 2つの値を保存する変数
        self.sensor_temp = 0.0
        self.sensor2_temp = 0.0

        # 温度(dummy01)の購読
        self.sub_temp = self.create_subscription(
            SensorData, '/sensor/data', self.sensor_callback, 10)
        
        # 湿度(dummy02)の購読
        self.sub_humidity = self.create_subscription(
            SensorData2, '/sensor/data2', self.sensor2_callback, 10)

    def sensor_callback(self, msg):
        self.sensor_temp = msg.value
        self.check_conditions()

    def sensor2_callback(self, msg):
        self.sensor2_temp = msg.value
        self.check_conditions()

    def check_conditions(self):
        # AND条件: センサー1 > 3 かつ センサー2 > 5 なら光る
        if self.sensor_temp > 3 and self.sensor2_temp > 5:
            self.led.on()
            self.get_logger().info('Conditions met! LED ON')
        else:
            self.led.off()

def main(args=None):
    rclpy.init(args=args)
    node = LEDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()