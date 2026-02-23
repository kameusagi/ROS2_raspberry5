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
        self.subscription = self.create_subscription(JudgeData,'system/led_command',self.command_callback,10)
        self.last_msg_time = self.get_clock().now()
        
        watchdog_timer_period = 1.0  # 1秒ごとにウォッチドッグチェック
        self.watchdog_timer = self.create_timer(watchdog_timer_period, self.watchdog_check)
        self.get_logger().info('LED Node has been started. Waiting for commands from core...')

    def command_callback(self, msg):
        self.last_msg_time = self.get_clock().now()  # 最後のメッセージ受信時間を更新
        if msg.value == 1:
            self.led.on()
        else:
            self.led.off()

    def watchdog_check(self):
            # 最後の命令から1.5秒以上経過していたら、通信異常と判断
            elapsed = self.get_clock().now() - self.last_msg_time
            if elapsed.nanoseconds > 1.5 * 1e9:
                self.get_logger().error('CRITICAL: Lost connection to Core! Shutting down node...')
                self.led.off()
                # raise SystemExit  # または rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = LEDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()