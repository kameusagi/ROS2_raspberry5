import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import csv
from datetime import datetime
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time

class ImageSaver(Node):
    def __init__(self):
        super().__init__('camera_sensor_node')
        self.bridge = CvBridge()
        
        # 1. フォルダ名の設定 (引数から取得、なければ現在時刻)
        self.declare_parameter('folder_name', '')
        param_folder = self.get_parameter('folder_name').get_parameter_value().string_value
        
        if param_folder == '':
            param_folder = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # 保存先のベースパス設定
        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.target_path = os.path.join(current_dir, 'output', param_folder)
        
        # フォルダとCSVの準備
        if not os.path.exists(self.target_path):
            os.makedirs(self.target_path)
        
        self.csv_path = os.path.join(self.target_path, 'timestamps.csv')
        self.init_csv()

        self.count = 0
        
        # QoS設定
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, 
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            qos_profile)
        
        self.get_logger().info(f'保存開始: {self.target_path}')

    def init_csv(self):
        """CSVファイルのヘッダーを作成"""
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['index', 'filename', 'timestamp'])

    def listener_callback(self, msg):
        try:
            time.sleep(1)
            # 変換
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 保存ファイル名の決定
            now = datetime.now()
            timestamp_str = now.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3] # ミリ秒まで
            filename = f'image_{self.count:04d}.jpg'
            save_full_path = os.path.join(self.target_path, filename)
            
            # 画像保存
            success = cv2.imwrite(save_full_path, frame)
            
            if success:
                # CSVにタイムスタンプを追記
                with open(self.csv_path, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([self.count, filename, timestamp_str])
                
                self.get_logger().info(f'Saved: {filename} at {timestamp_str}')
                self.count += 1
                
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopping...')
    finally:
        # 最後にノードを破壊して終了
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
    #フォルダ指定
    #ros2 run camera01 camera_sensor_node --ros-args -p folder_name:=my_test_data
    #フォルダ指定なし
    #ros2 run camera01 camera_sensor_node