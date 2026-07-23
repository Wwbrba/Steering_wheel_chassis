import os
from launch import LaunchDescription
from launch.actions import TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_rm_nav_bringup = get_package_share_directory('rm_nav_bringup')
    pkg_rslidar_sdk = get_package_share_directory('rslidar_sdk')
    
    rviz_config = os.path.join(pkg_rm_nav_bringup, 'rviz', 'test.rviz')
    slam_config = os.path.join(pkg_rm_nav_bringup, 'config_TEB', 'reality', 'mapper_params_online_async_real.yaml')

    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        # 1. 只启动官方驱动节点；不要 include humble_start.py，
        #    因为该官方 launch 还会无条件启动它自己的 rviz2。
        Node(
            namespace='rslidar_sdk',
            package='rslidar_sdk',
            executable='rslidar_sdk_node',
            output='screen'
        ),
        
        # 2. 静态TF：base_link → rslidar
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'rslidar'],
            output='screen'
        ),
        
        # 3. 点云转激光
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            remappings=[
                ('cloud_in', '/rslidar_points'),
                ('scan', '/scan')
            ],
            parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.1,
                'min_height': -0.30,
                'max_height': 0.30,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0087,
                'range_min': 0.15,
                'range_max': 30.0
            }],
            output='screen'
        ),
        
        # 4. 静态TF：odom → base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'odom', '--child-frame-id', 'base_link'],
            output='screen'
        ),
        
        # 5. SLAM Toolbox
        TimerAction(period=3.0, actions=[Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            parameters=[slam_config, {'use_sim_time': False}],
            arguments=['--ros-args', '--log-level', 'info'],
            output='screen'
        )]),
        
        # 6. 你自己的Rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
    ])
