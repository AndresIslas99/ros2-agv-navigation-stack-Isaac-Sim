#!/usr/bin/env python3
# =============================================================================
# Main Navigation Bringup Launch File
# Author: Andrés Islas Bravo
# Description: Launches RTAB-Map SLAM, sensor fusion, and Nav2 navigation
# Usage: ros2 launch ros2_agv_navigation_stack bringup.launch.py
# =============================================================================

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_share = get_package_share_directory('ros2_agv_navigation_stack')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    localization = LaunchConfiguration('localization')
    nav2_params_file = LaunchConfiguration('params_file')
    rtabmap_params_file = LaunchConfiguration('rtabmap_params_file')
    ekf_params_file = LaunchConfiguration('ekf_params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    use_rviz = LaunchConfiguration('use_rviz')
    database_path = LaunchConfiguration('database_path')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Isaac Sim) clock'
    )
    
    declare_slam = DeclareLaunchArgument(
        'slam',
        default_value='true',
        description='Run SLAM for mapping (true) or localization only (false)'
    )
    
    declare_localization = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Run localization in existing map (requires database_path)'
    )
    
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        description='Full path to Nav2 parameters file'
    )
    
    declare_rtabmap_params_file = DeclareLaunchArgument(
        'rtabmap_params_file',
        default_value=os.path.join(pkg_share, 'config', 'rtabmap_params.yaml'),
        description='Full path to RTAB-Map parameters file'
    )
    
    declare_ekf_params_file = DeclareLaunchArgument(
        'ekf_params_file',
        default_value=os.path.join(pkg_share, 'config', 'ekf_params.yaml'),
        description='Full path to robot_localization EKF parameters'
    )
    
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_share, 'rviz', 'rtabmap_nav2.rviz'),
        description='Full path to RViz config file'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 visualization'
    )
    
    declare_database_path = DeclareLaunchArgument(
        'database_path',
        default_value=os.path.join(os.path.expanduser('~'), 'rtabmap.db'),
        description='Path to RTAB-Map database file'
    )

    # =========================================================================
    # Robot Localization EKF (Wheel Odometry + IMU Fusion)
    # =========================================================================
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

    # =========================================================================
    # RTAB-Map SLAM with 3D LiDAR + Stereo Camera
    # =========================================================================
    
    # RTAB-Map Odometry (ICP from 3D LiDAR)
    rtabmap_odom_node = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='rtabmap_odom',
        output='screen',
        parameters=[
            rtabmap_params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('scan_cloud', '/front_3d_lidar/lidar_points'),
            ('odom', '/rtabmap/odom')
        ]
    )

    # RTAB-Map SLAM Node
    rtabmap_slam_node = Node(
        condition=IfCondition(slam),
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
                'qos_image': 2,           # Best effort for images
                'qos_scan': 2,            # Best effort for scan
                'database_path': database_path,
                'Mem/IncrementalMemory': 'true',
                'Mem/InitWMWithAllNodes': 'false',
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
        arguments=['--delete_db_on_start'] if LaunchConfiguration('slam') else []
    )

    # RTAB-Map Localization Node (when not doing SLAM)
    rtabmap_localization_node = Node(
        condition=IfCondition(localization),
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
                'Mem/IncrementalMemory': 'false',  # Localization only
                'Mem/InitWMWithAllNodes': 'true',
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
        ]
    )

    # =========================================================================
    # Nav2 Navigation Stack
    # =========================================================================
    
    # Nav2 Controller Server
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Nav2 Planner Server
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Nav2 Behavior Server
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Nav2 BT Navigator
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Nav2 Waypoint Follower
    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Nav2 Velocity Smoother
    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel')
        ]
    )

    # Nav2 Lifecycle Manager
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother'
            ]}
        ]
    )

    # =========================================================================
    # Visualization
    # =========================================================================
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        # Declare arguments
        declare_use_sim_time,
        declare_slam,
        declare_localization,
        declare_params_file,
        declare_rtabmap_params_file,
        declare_ekf_params_file,
        declare_rviz_config,
        declare_use_rviz,
        declare_database_path,
        
        # Sensor Fusion
        ekf_node,
        
        # RTAB-Map SLAM
        rtabmap_odom_node,
        rtabmap_slam_node,
        rtabmap_localization_node,
        
        # Navigation Stack
        controller_server_node,
        planner_server_node,
        behavior_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        velocity_smoother_node,
        lifecycle_manager_node,
        
        # Visualization
        rviz_node,
    ])
