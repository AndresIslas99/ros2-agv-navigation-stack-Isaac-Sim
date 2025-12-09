#!/usr/bin/env python3
# =============================================================================
# RTAB-Map LiDAR-Only SLAM (Simplified - Start Here!)
# Author: Andrés Islas Bravo
# Description: Uses only 3D LiDAR with ICP odometry - most stable configuration
# Usage: ros2 launch ros2_agv_navigation_stack slam_lidar_only.launch.py
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
    database_path = LaunchConfiguration('database_path')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock from /clock topic'
    )
    
    declare_database_path = DeclareLaunchArgument(
        'database_path',
        default_value=os.path.join(os.path.expanduser('~'), 'rtabmap.db'),
        description='Path to save RTAB-Map database'
    )

    # =========================================================================
    # RTAB-Map ICP Odometry from 3D LiDAR
    # This provides odometry by matching consecutive point clouds
    # =========================================================================
    icp_odometry_node = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'wait_for_transform_duration': 0.5,
            'expected_update_rate': 15.0,
            'guess_frame_id': '',
            
            # ICP Parameters for 3D LiDAR (RTAB-Map params are strings)
            'Icp/Strategy': '1',                    # Point-to-Plane
            'Icp/VoxelSize': '0.1',                 # 10cm voxel (faster)
            'Icp/MaxCorrespondenceDistance': '0.5', # 50cm max correspondence
            'Icp/PointToPlane': 'true',
            'Icp/PointToPlaneK': '20',
            'Icp/Iterations': '30',
            'Icp/Epsilon': '0.001',
            'Icp/MaxTranslation': '0.3',            # Max 30cm per frame
            'Icp/MaxRotation': '0.78',              # Max 45 degrees per frame
            
            # Odometry parameters (RTAB-Map params are strings)
            'Odom/Strategy': '0',                   # Frame-to-Map
            'Odom/GuessMotion': 'true',
            'Odom/ResetCountdown': '1',
            'Odom/ScanKeyFrameThr': '0.7',
            
            # Point cloud processing (ROS params - use native types!)
            'scan_cloud_max_points': 0,             # 0 = no limit
            'scan_voxel_size': 0.1,                 # Downsample input
            'scan_normal_k': 10,
        }],
        remappings=[
            ('scan_cloud', '/front_3d_lidar/lidar_points'),
        ]
    )

    # =========================================================================
    # RTAB-Map SLAM Node
    # Builds the map and performs loop closure
    # =========================================================================
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'database_path': database_path,
            
            # Frame configuration
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'publish_tf': True,
            
            # Subscribe ONLY to scan_cloud (no cameras for now)
            'subscribe_depth': False,
            'subscribe_stereo': False,
            'subscribe_scan': False,              # 2D scan
            'subscribe_scan_cloud': True,         # 3D point cloud
            'subscribe_rgb': False,
            'subscribe_odom_info': True,          # From ICP odometry
            
            # Sync settings (ROS params - native types)
            'approx_sync': True,
            'sync_queue_size': 30,
            'topic_queue_size': 30,
            
            # Core SLAM parameters (RTAB-Map params - strings)
            'Rtabmap/DetectionRate': '1.0',       # 1 Hz SLAM rate
            'RGBD/CreateOccupancyGrid': 'true',
            'RGBD/ProximityBySpace': 'true',
            'RGBD/AngularUpdate': '0.1',          # ~5.7 degrees
            'RGBD/LinearUpdate': '0.1',           # 10cm
            'RGBD/OptimizeMaxError': '3.0',
            
            # Memory
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',
            
            # Loop closure
            'Kp/MaxFeatures': '400',
            'Kp/DetectorStrategy': '8',           # ORB
            
            # ICP registration for loop closure
            'Icp/VoxelSize': '0.1',
            'Icp/PointToPlane': 'true',
            'Icp/Iterations': '30',
            'Icp/MaxCorrespondenceDistance': '0.5',
            
            # Graph optimization  
            'Optimizer/Strategy': '1',            # g2o
            'Optimizer/Iterations': '20',
            
            # Grid map from point cloud
            'Grid/FromDepth': 'false',            # Use scan_cloud
            'Grid/RangeMax': '10.0',
            'Grid/RangeMin': '0.5',
            'Grid/CellSize': '0.05',              # 5cm resolution
            'Grid/MaxGroundHeight': '0.1',
            'Grid/MaxObstacleHeight': '2.0',
            'Grid/3D': 'false',                   # 2D occupancy for Nav2
            'Grid/RayTracing': 'true',
        }],
        remappings=[
            ('scan_cloud', '/front_3d_lidar/lidar_points'),
            ('odom', '/icp_odometry/odom'),
            ('odom_info', '/icp_odometry/odom_info'),
        ],
        arguments=['--delete_db_on_start']
    )

    # =========================================================================
    # RViz Visualization
    # =========================================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'lidar_slam.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_database_path,
        icp_odometry_node,
        rtabmap_node,
        rviz_node,
    ])
