from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. カメラドライバノードの起動
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='usb_cam_node',
            parameters=[{
                'video_device': '/dev/video0',
                'image_size': [640, 480]
            }],
            # トピック名を /image_raw に固定（もし変わってしまう場合）
            remappings=[('/image_raw', '/image_raw')]
        ),

        # 2. あなたが作った保存ノードの起動
        Node(
            package='camera01',
            executable='camera_sensor_node',
            name='cam1',
            output='screen'
        )
    ])