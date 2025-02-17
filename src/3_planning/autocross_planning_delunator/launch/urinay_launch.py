from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    ld = LaunchDescription()

    config_node = os.path.join(
        get_package_share_directory('urinay'),
        'config',
        'urinay.yaml'
        )

    node=Node(
            package='urinay',
            # namespace='clustering_ground_segmentation',
            name='urinay',
            executable='urinay_exec',
            parameters=[config_node]
        )

    ld.add_action(node)
    return ld