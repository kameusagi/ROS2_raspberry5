import rclpy
from rclpy.node import Node
from sensor_interfaces.msg import SensorData, JudgeData
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import threading
import time

class PriorityDecisionNode(Node):
    def __init__(self):
        super().__init__('priority_decision_node')

        # 1. 役割ごとにグループを完全に分ける (相互排他)
        self.group_inference = MutuallyExclusiveCallbackGroup()
        self.group_heartbeat = MutuallyExclusiveCallbackGroup()
        self.group_sensor = MutuallyExclusiveCallbackGroup()
        
        self.sensor = 0.0
        self.sensor2 = 0.0
        self.latest_decision = 0 

        # 2. センサー購読
        self.create_subscription(SensorData, '/sensor/data', self.sensor_callback, 10, callback_group=self.group_sensor)
        self.create_subscription(SensorData, '/sensor/data2', self.sensor2_callback, 10, callback_group=self.group_sensor)
        
        self.publisher_ = self.create_publisher(JudgeData, 'system/led_command', 10)
        
        # 3. 【脳】推論タイマー（重い処理グループへ）
        self.inference_timer = self.create_timer(1.0, self.inference_callback, callback_group=self.group_inference)
        
        # 4. 【心拍】送信タイマー（高優先グループへ）
        self.heartbeat_timer = self.create_timer(0.1, self.heartbeat_callback, callback_group=self.group_heartbeat)

        self.get_logger().info('Priority Guarded Decision Node Started!')

    def sensor_callback(self, msg): 
        self.sensor = msg.value

    def sensor2_callback(self, msg): 
        self.sensor2 = msg.value

    def inference_callback(self):
        """重いAI推論"""
        self.get_logger().info('  [Brain] Inference Start...')
        time.sleep(2.5) # わざと周期を超えて止める
        calculation_result = float(self.sensor * self.sensor2)
        self.latest_decision = 1 if 5.0 < calculation_result < 50.0 else 0
        self.get_logger().warn(f'  [Brain] Inference Done. Result: {self.latest_decision}')

    def heartbeat_callback(self):
        """高頻度の送信（心拍）"""
        cmd = JudgeData()
        cmd.value = self.latest_decision
        self.publisher_.publish(cmd)

def main():
    rclpy.init()
    node = PriorityDecisionNode()

    # --- 優先度ガードの核心部 ---
    
    # A. 【高優先】心拍とセンサー受信を担当するエグゼキュータ
    # 本来はもっと細かく分けられますが、まずは「重い推論」以外をここに入れます
    high_priority_executor = MultiThreadedExecutor()
    high_priority_executor.add_node(node)

    # B. 【低優先】重いAI推論だけを担当するエグゼキュータ
    # これを別スレッドで回すことで、心拍側のループをブロックさせないようにします
    low_priority_executor = SingleThreadedExecutor() 
    low_priority_executor.add_node(node)

    # 別スレッドで高優先エグゼキュータを起動
    high_thread = threading.Thread(target=high_priority_executor.spin, daemon=True)
    high_thread.start()

    try:
        # メインスレッドで低優先（推論）エグゼキュータを起動
        low_priority_executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()