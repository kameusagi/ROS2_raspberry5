from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dummy_sensor01',
            executable='dummy_sensor_node',
            name='dummy1'
        ),
        Node(
            package='dummy_sensor02',
            executable='dummy_sensor_node',
            name='dummy2'
        ),
        Node(
            package='led_controller_01',
            executable='led_node',
            name='red_led'
        )
    ])