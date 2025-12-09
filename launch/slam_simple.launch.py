#!/usr/bin/env python3
# =============================================================================
# RTAB-Map SLAM using Isaac Sim Odometry (Simplest - Start Here!)
# Author: Andrés Islas Bravo
# Description: Uses wheel odometry from Isaac Sim directly, LiDAR for mapping only
# Usage: ros2 launch ros2_agv_navigation_stack slam_simple.launch.py
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
    # RTAB-Map SLAM Node
    # Uses Isaac Sim's wheel odometry (/chassis/odom) directly
    # LiDAR is used for mapping and loop closure only
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
            'publish_tf': True,                   # Publish map->odom
            'tf_delay': 0.05,
            'tf_tolerance': 0.2,
            'wait_for_transform': 0.5,
            
            # Subscribe to scan_cloud only (no odom_info needed!)
            'subscribe_depth': False,
            'subscribe_stereo': False,
            'subscribe_scan': False,
            'subscribe_scan_cloud': True,
            'subscribe_rgb': False,
            'subscribe_odom_info': False,         # NOT using ICP odometry
            
            # QoS settings for Isaac Sim
            'qos_scan': 1,                        # Best effort for sensor data
            'qos_odom': 1,                        # Best effort for odometry
            
            # Sync settings
            'approx_sync': True,
            'sync_queue_size': 100,               # Larger queue for high-rate LiDAR
            'topic_queue_size': 100,
            
            # Scan processing
            'scan_cloud_max_points': 0,           # No limit
            
            # Core SLAM parameters
            'Rtabmap/DetectionRate': '2.0',       # 2 Hz SLAM rate
            'RGBD/CreateOccupancyGrid': 'true',
            'RGBD/ProximityBySpace': 'true',
            'RGBD/AngularUpdate': '0.05',         # ~3 degrees
            'RGBD/LinearUpdate': '0.05',          # 5cm - more frequent updates
            'RGBD/OptimizeMaxError': '3.0',
            
            # Memory
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',
            
            # ICP for scan registration (not odometry)
            'Icp/VoxelSize': '0.1',
            'Icp/PointToPlane': 'true',
            'Icp/Iterations': '30',
            'Icp/MaxCorrespondenceDistance': '1.0',  # Larger for sim
            'Icp/PointToPlaneK': '10',
            'Icp/PointToPlaneRadius': '0.0',
            
            # Registration strategy
            'Reg/Strategy': '1',                  # ICP registration
            'Reg/Force3DoF': 'true',              # 2D navigation (x, y, yaw)
            
            # Graph optimization  
            'Optimizer/Strategy': '1',            # g2o
            'Optimizer/Iterations': '20',
            
            # Grid map from point cloud
            'Grid/FromDepth': 'false',
            'Grid/RangeMax': '15.0',              # Increased range
            'Grid/RangeMin': '0.3',
            'Grid/CellSize': '0.05',
            'Grid/MaxGroundHeight': '0.15',       # Ground filter
            'Grid/MaxObstacleHeight': '2.0',
            'Grid/MinClusterSize': '5',
            'Grid/3D': 'false',
            'Grid/RayTracing': 'true',
            'Grid/Sensor': '0',                   # Laser sensor
        }],
        remappings=[
            ('scan_cloud', '/front_3d_lidar/lidar_points'),
            ('odom', '/chassis/odom'),            # Use Isaac Sim's wheel odometry directly
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
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'simple_slam.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_database_path,
        rtabmap_node,
        rviz_node,
    ])
