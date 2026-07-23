from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start Point-LIO with the RoboSense Airy Lite configuration."""
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz for Point-LIO output.',
    )

    point_lio_params = [
        PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'config',
            'airy_lite.yaml',
        ]),
        {
            # Keep the same runtime overrides as mapping_mid360.launch.py.
            # The topics, Airy Lite parser (lidar_type=4), 24 rings and
            # LiDAR/IMU extrinsics come from airy_lite.yaml.
            'use_imu_as_input': False,
            'prop_at_freq_of_imu': True,
            'check_satu': True,
            'init_map_size': 10,
            'point_filter_num': 3,
            'space_down_sample': True,
            'filter_size_surf': 0.5,
            'filter_size_map': 0.5,
            'ivox_nearby_type': 6,
            'runtime_pos_log_enable': False,
        },
    ]

    point_lio_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=point_lio_params,
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'rviz_cfg',
            'loam_livox.rviz',
        ])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        prefix='nice',
    )

    return LaunchDescription([
        rviz_arg,
        point_lio_node,
        rviz_node,
    ])
