import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('wheel_odom'),
        'config',
        'config.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Path to the configuration file'
        ),
        Node(
            package='wheel_odom',
            executable='wheel_odom_node',
            name='wheel_odom_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')]
        )
    ])