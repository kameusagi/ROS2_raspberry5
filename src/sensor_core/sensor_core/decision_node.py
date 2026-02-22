import rclpy
from rclpy.node import Node
from sensor_interfaces.msg import SensorData
from sensor_interfaces.msg import JudgeData

class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')
        # センサー2つを購読
        self.create_subscription(
            SensorData, '/sensor/data', self.sensor_callback, 10)
        self.create_subscription(
            SensorData, '/sensor/data2', self.sensor2_callback, 10)
        
        # 判断結果（LEDへの命令）を送るパブリッシャー
        # 今回はシンプルに「String」で送るか、新しくmsgを作るのもアリですが、
        # まずは既存の SensorData.msg の value を「状態コード」として流用してみます
        self.publisher_ = self.create_publisher(JudgeData, 'system/led_command', 10)
        
        self.sensor = 0.0
        self.sensor2 = 0.0

    def sensor_callback(self, msg):
        self.sensor = msg.value
        self.evaluate_condition()

    def sensor2_callback(self, msg):
        self.sensor2 = msg.value
        self.evaluate_condition()

    def evaluate_condition(self):
        # ここが「司令塔」の心臓部：複雑な条件をここに集約！
        cmd = JudgeData()
        calculation_result = float(self.sensor * self.sensor2)
        if calculation_result > 5.0 and calculation_result < 50.0:
            cmd.value = 1
        else:
            cmd.value = 0  
        self.get_logger().info(f'sensor={self.sensor:.2f}, sensor2={self.sensor2:.2f}, result={calculation_result:.2f}, sending command={cmd.value}')
        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init()
    node = DecisionNode()
    rclpy.spin(node)
    rclpy.shutdown()