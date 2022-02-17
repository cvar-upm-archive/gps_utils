from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gps_utils',
            namespace='',
            executable='gps_translator_node',
            name='gps_translator',
            output='screen',
            emulate_tty=True
        )
    ])