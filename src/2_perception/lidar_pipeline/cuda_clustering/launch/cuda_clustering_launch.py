from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    ld = LaunchDescription()

    config_node = os.path.join(
        get_package_share_directory('clustering'),
        'config',
        'clustering.yaml'
        )

    node=Node(
            package='clustering',
            name='clustering_node',
            executable='clustering_node',
            output='screen',
            parameters=[config_node]
        )

    ld.add_action(node)
    return ld