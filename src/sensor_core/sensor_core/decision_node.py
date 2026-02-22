import rclpy
from rclpy.node import Node
from sensor_interfaces.msg import SensorData
from sensor_interfaces.msg import JudgeData

class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')
        # センサー2つを購読
        self.create_subscription(SensorData, '/sensor/data', self.sensor_callback, 10)
        self.create_subscription(SensorData, '/sensor/data2', self.sensor2_callback, 10)
        
        # 判断結果（LEDへの命令）を送るパブリッシャー
        self.publisher_ = self.create_publisher(JudgeData, 'system/led_command', 10)
        
        # 判断・送信用のタイマー（1Hz = 1秒周期）
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # 最新のセンサー値を保存する変数
        self.sensor = 0.0
        self.sensor2 = 0.0

        self.get_logger().info('Decision Node started with 1Hz control loop.')


    def sensor_callback(self, msg): self.sensor = msg.value
    def sensor2_callback(self, msg): self.sensor2 = msg.value

    def timer_callback(self):
        # 周期が来たら、その時の最新値で計算
        res = self.sensor * self.sensor2
        cmd = JudgeData()
        calculation_result = float(self.sensor * self.sensor2)
        if calculation_result > 5.0 and calculation_result < 50.0:
            cmd.value = 1
        else:
            cmd.value = 0  
        self.get_logger().info(f'sensor={self.sensor:.2f}, sensor2={self.sensor2:.2f}, result={calculation_result:.2f}, sending command={cmd.value}')
        self.publisher_.publish(cmd)

def main():
    rclpy.init(args=None)
    node = DecisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()