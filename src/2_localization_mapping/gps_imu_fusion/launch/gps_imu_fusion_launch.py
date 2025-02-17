from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    ld = LaunchDescription()

    config_node = os.path.join(
        get_package_share_directory('gps_imu_fusion'),
        'config',
        'gps_imu_fusion.yaml'
        )

    node=Node(
            package='gps_imu_fusion',
            # namespace='gps_imu_fusion',
            name='gps_imu_fusion_node',
            executable='gps_imu_fusion_node',
            output = 'screen',
            # prefix=["gdbserver localhost:3000"],
            parameters=[config_node],
            emulate_tty=True
        )

    ld.add_action(node)
    return ld