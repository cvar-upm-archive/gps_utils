from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', description="Node namespace",
                              default_value=EnvironmentVariable('AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument('global_fix_sub_name', description="Global Fix subscriber topic name",
                              default_value="global_pose/fix"),
        DeclareLaunchArgument('global_ecef_pub', description="Global ECEF publisher topic name.",
                              default_value="global_pose/ecef"),
        DeclareLaunchArgument('local_pub_name', description="Local publisher topic name.",
                              default_value="local_pose"),
        Node(
            package='gps_utils',
            namespace=LaunchConfiguration('namespace'),
            executable='gps_translator_node',
            name='gps_translator',
            parameters=[{
                        'global_fix_sub_name': LaunchConfiguration('global_fix_sub_name'),
                        'global_ecef_pub': LaunchConfiguration('global_ecef_pub'),
                        'local_pub_name': LaunchConfiguration('local_pub_name'),
                        }],
            output='screen',
            emulate_tty=True
        )
    ])