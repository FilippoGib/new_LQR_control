from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('speed_estimator'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='speed_estimator',
            executable='speed_estimator_node',
            name='speed_estimator_node',
            parameters=[config_file]
        )
    ])
