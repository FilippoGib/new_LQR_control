from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[{
                'use_sim_time': False,  # Set to True if using simulation
                'magnetic_declination_radians': 0.0,
                'yaw_offset': 1.5707963, # pi / 2
                'zero_altitude': True,
                'broadcast_cartesian_transform': True,
                'publish_filtered_gps': True,
                'use_odometry_yaw': False,
                'wait_for_datum': False,
                # Frame settings
                'odom_frame': "odom",
                'base_link_frame': "base_link",
                'world_frame': "map"
            }],
            remappings=[
                ('imu', '/mti680G/imu/data'),
                ('gps/fix', '/emlid/navsatfix'),
                ('odometry/filtered', '/fast_limo/body'),
                ('odometry/gps', '/odometry/gps')
            ]
        )
    ])
