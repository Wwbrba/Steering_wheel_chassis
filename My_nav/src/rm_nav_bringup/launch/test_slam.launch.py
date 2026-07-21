import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_rm_nav_bringup = get_package_share_directory('rm_nav_bringup')
    pkg_rslidar_sdk = get_package_share_directory('rslidar_sdk')
    
    rviz_config = os.path.join(pkg_rm_nav_bringup, 'rviz', 'test.rviz')
    slam_config = os.path.join(pkg_rm_nav_bringup, 'config_TEB', 'reality', 'mapper_params_online_async_real.yaml')

    return LaunchDescription([
        # 1. 只启动官方驱动节点；不要 include humble_start.py，
        #    因为该官方 launch 还会无条件启动它自己的 rviz2。
        Node(
            namespace='rslidar_sdk',
            package='rslidar_sdk',
            executable='rslidar_sdk_node',
            output='screen',
            additional_env={'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp'}
        ),
        
        # 2. 静态TF：base_link → rslidar（新格式）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'rslidar'],
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
        
        # 4. 静态TF：odom → base_link（新格式，修复 odom pose 警告）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            output='screen'
        ),
        
        # 5. SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            parameters=[slam_config],
            arguments=['--ros-args', '--log-level', 'info'],
            output='screen'
        ),
        
        # 6. 你自己的Rviz2（只启动这一个！）
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
    ])
