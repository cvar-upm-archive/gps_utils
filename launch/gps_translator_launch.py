from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='gps_utils',
#             namespace='',
#             executable='gps_translator_node',
#             name='gps_translator',
#             output='screen',
#             emulate_tty=True
#         )
#     ])

def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='gps_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='gps_utils',
                    plugin='GpsTranslator::GpsTranslator',
                    name='translator')
            ],
            output='screen',
    )

    return LaunchDescription([container])