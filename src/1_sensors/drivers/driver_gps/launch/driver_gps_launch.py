from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    ld = LaunchDescription()

    os.system('~/mmr-drive/useful_scripts/connect_gps.sh')

    config_node = os.path.join(
        get_package_share_directory('driver_gps'),
        'config',
        'driver_gps.yaml'
        )
    node=Node(
            package='driver_gps',
            # namespace='driver_gps',
            name='driver_gps_node',
            executable='driver_gps_node',
            output = 'screen',
            parameters=[config_node]
        )

    ld.add_action(node)
    return ld