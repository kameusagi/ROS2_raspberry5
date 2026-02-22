#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gpiozero import LED
from sensor_interfaces.msg import SensorData 
from sensor_interfaces.msg import JudgeData

class LEDController(Node):
    def __init__(self):
        super().__init__('led_node')
        self.led = LED(17)
        
        # 司令塔 (decision_node) からの命令を購読する
        self.subscription = self.create_subscription(
            JudgeData,
            'system/led_command',
            self.command_callback,
            10 #キューサイズ
        )
        self.get_logger().info('LED Node has been started. Waiting for commands from core...')

    def command_callback(self, msg):
        if msg.value == 1:
            self.led.on()
            # self.get_logger().info('LED ON command received')
        else:
            self.led.off()
            # self.get_logger().info('LED OFF command received')

# def main(args=None):
#     rclpy.init(args=args)
#     node = LEDController()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = LEDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()