import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    config_node = os.path.join(
        get_package_share_directory('gps_to_odom'),
        'config',
        'params.yaml'
    )
    
    return LaunchDescription([
        
        DeclareLaunchArgument(
            'params_file',
            default_value='',
            description='Path to the parameters file to load'
        ),
        
        Node(
            package='gps_to_odom',
            executable='gps_to_odom_node',
            name='gps_to_odom_node',
            output='screen',
            parameters=[config_node]
        )
    ])