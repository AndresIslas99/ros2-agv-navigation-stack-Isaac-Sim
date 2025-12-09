#!/usr/bin/env python3
# =============================================================================
# RTAB-Map SLAM-Only Launch File
# Author: Andrés Islas Bravo
# Description: Launches only RTAB-Map SLAM for mapping (no navigation)
# Usage: ros2 launch ros2_agv_navigation_stack slam_mapping.launch.py
# =============================================================================

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_agv_navigation_stack')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    rtabmap_params_file = LaunchConfiguration('rtabmap_params_file')
    ekf_params_file = LaunchConfiguration('ekf_params_file')
    database_path = LaunchConfiguration('database_path')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )
    
    declare_rtabmap_params = DeclareLaunchArgument(
        'rtabmap_params_file',
        default_value=os.path.join(pkg_share, 'config', 'rtabmap_params.yaml'),
        description='Full path to RTAB-Map parameters'
    )
    
    declare_ekf_params = DeclareLaunchArgument(
        'ekf_params_file',
        default_value=os.path.join(pkg_share, 'config', 'ekf_params.yaml'),
        description='Full path to EKF parameters'
    )
    
    declare_database_path = DeclareLaunchArgument(
        'database_path',
        default_value=os.path.join(os.path.expanduser('~'), 'rtabmap.db'),
        description='Path to save RTAB-Map database'
    )

    # EKF for odometry fusion
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('odometry/filtered', '/odometry/filtered')
        ]
    )

    # RTAB-Map ICP Odometry from 3D LiDAR
    rtabmap_odom_node = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='rtabmap_odom',
        output='screen',
        parameters=[
            rtabmap_params_file,
            {
                'use_sim_time': use_sim_time,
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': False,  # Let EKF handle TF
                'wait_for_transform_duration': 0.2,
                'approx_sync': True,
            }
        ],
        remappings=[
            ('scan_cloud', '/front_3d_lidar/lidar_points'),
        ]
    )

    # RTAB-Map SLAM
    rtabmap_slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            rtabmap_params_file,
            {
                'use_sim_time': use_sim_time,
                'subscribe_depth': False,
                'subscribe_stereo': True,
                'subscribe_scan_cloud': True,
                'approx_sync': True,
                'queue_size': 30,
                'database_path': database_path,
                'Mem/IncrementalMemory': 'true',
                'Mem/InitWMWithAllNodes': 'false',
                'RGBD/CreateOccupancyGrid': 'true',
                'Grid/FromDepth': 'false',
            }
        ],
        remappings=[
            ('scan_cloud', '/front_3d_lidar/lidar_points'),
            ('left/image_rect', '/front_stereo_camera/left/image_raw'),
            ('right/image_rect', '/front_stereo_camera/right/image_raw'),
            ('left/camera_info', '/front_stereo_camera/left/camera_info'),
            ('right/camera_info', '/front_stereo_camera/right/camera_info'),
            ('odom', '/odometry/filtered'),
            ('imu', '/chassis/imu'),
        ],
        arguments=['--delete_db_on_start']
    )

    # RTAB-Map Visualization
    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[
            rtabmap_params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('scan_cloud', '/front_3d_lidar/lidar_points'),
            ('odom', '/odometry/filtered'),
        ]
    )

    # RViz for additional visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'rtabmap_slam.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_rtabmap_params,
        declare_ekf_params,
        declare_database_path,
        ekf_node,
        rtabmap_odom_node,
        rtabmap_slam_node,
        rtabmap_viz_node,
        rviz_node,
    ])
