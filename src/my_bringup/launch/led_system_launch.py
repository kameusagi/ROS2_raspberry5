from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dummy_sensor01',
            executable='dummy_sensor_node',
            name='dummy1_sensor'
        ),
        Node(
            package='dummy_sensor02',
            executable='dummy_sensor_node',
            name='dummy2_sensor'
        ),
        Node(
            package='led_controller_01',
            executable='led_node',
            name='red_led'
        ),
        Node(
            package='sensor_core',
            executable='decision_node',
            name='brain_node'
        )
    ])