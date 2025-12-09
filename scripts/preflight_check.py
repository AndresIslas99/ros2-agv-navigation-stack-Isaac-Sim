#!/usr/bin/env python3
# =============================================================================
# Pre-flight Check Script for RTAB-Map SLAM
# Author: Andrés Islas Bravo
# Description: Verifies Isaac Sim topics and TFs before launching SLAM
# Usage: ros2 run ros2_agv_navigation_stack preflight_check.py
# =============================================================================

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, Image, Imu, CameraInfo
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
import tf2_ros
import time
import sys


class PreflightCheck(Node):
    def __init__(self):
        super().__init__('preflight_check')
        
        self.results = {}
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # QoS for sensor data (best effort)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Topics to check
        self.topics_to_check = {
            '/clock': (Clock, 'reliable'),
            '/front_3d_lidar/lidar_points': (PointCloud2, 'best_effort'),
            '/front_stereo_camera/left/image_raw': (Image, 'best_effort'),
            '/front_stereo_camera/right/image_raw': (Image, 'best_effort'),
            '/front_stereo_camera/left/camera_info': (CameraInfo, 'best_effort'),
            '/front_stereo_camera/right/camera_info': (CameraInfo, 'best_effort'),
            '/chassis/odom': (Odometry, 'best_effort'),
            '/chassis/imu': (Imu, 'best_effort'),
        }
        
        # TF frames to check
        self.tf_frames = [
            ('map', 'odom'),
            ('odom', 'base_link'),
            ('base_link', 'front_3d_lidar'),
            ('base_link', 'front_stereo_camera_left'),
            ('base_link', 'chassis_imu'),
        ]
        
        # Subscribe to all topics
        self.subscriptions_list = []
        for topic, (msg_type, qos_type) in self.topics_to_check.items():
            if qos_type == 'best_effort':
                qos = sensor_qos
            else:
                qos = QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1
                )
            
            self.results[topic] = {'received': False, 'hz': 0, 'last_time': None, 'count': 0}
            
            sub = self.create_subscription(
                msg_type,
                topic,
                lambda msg, t=topic: self.topic_callback(msg, t),
                qos
            )
            self.subscriptions_list.append(sub)
        
        self.get_logger().info('Starting preflight check... (waiting 5 seconds)')
        
        # Timer to print results
        self.start_time = time.time()
        self.timer = self.create_timer(0.1, self.timer_callback)
        
    def topic_callback(self, msg, topic):
        now = time.time()
        if self.results[topic]['last_time'] is not None:
            dt = now - self.results[topic]['last_time']
            if dt > 0:
                # Exponential moving average for Hz
                alpha = 0.3
                new_hz = 1.0 / dt
                self.results[topic]['hz'] = alpha * new_hz + (1 - alpha) * self.results[topic]['hz']
        
        self.results[topic]['last_time'] = now
        self.results[topic]['received'] = True
        self.results[topic]['count'] += 1
        
        # Check timestamp
        if hasattr(msg, 'header'):
            stamp = msg.header.stamp
            sim_time = stamp.sec + stamp.nanosec * 1e-9
            self.results[topic]['sim_time'] = sim_time
    
    def timer_callback(self):
        elapsed = time.time() - self.start_time
        
        if elapsed >= 5.0:
            self.print_results()
            self.timer.cancel()
            rclpy.shutdown()
    
    def print_results(self):
        print('\n' + '=' * 70)
        print('PREFLIGHT CHECK RESULTS')
        print('=' * 70)
        
        # Topic results
        print('\n📡 TOPIC STATUS:')
        print('-' * 70)
        
        all_ok = True
        critical_topics = ['/clock', '/front_3d_lidar/lidar_points', '/chassis/odom']
        
        for topic, result in self.results.items():
            status = '✅' if result['received'] else '❌'
            hz_str = f"{result['hz']:.1f} Hz" if result['received'] else 'N/A'
            count_str = f"({result['count']} msgs)"
            
            is_critical = topic in critical_topics
            critical_marker = ' [CRITICAL]' if is_critical else ''
            
            if not result['received']:
                all_ok = False
                
            sim_time = result.get('sim_time', 'N/A')
            if isinstance(sim_time, float):
                sim_time = f"{sim_time:.2f}s"
            
            print(f"  {status} {topic}")
            print(f"      Rate: {hz_str} {count_str}  |  Sim Time: {sim_time}{critical_marker}")
        
        # TF results
        print('\n🔗 TF TRANSFORMS:')
        print('-' * 70)
        
        for parent, child in self.tf_frames:
            try:
                trans = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
                print(f"  ✅ {parent} → {child}")
            except Exception as e:
                print(f"  ❌ {parent} → {child}  (NOT FOUND)")
                if parent == 'odom' or child == 'base_link':
                    all_ok = False
        
        # Summary
        print('\n' + '=' * 70)
        if all_ok:
            print('✅ ALL CRITICAL CHECKS PASSED - Ready to launch SLAM!')
            print('\nRun:')
            print('  ros2 launch ros2_agv_navigation_stack slam_lidar_only.launch.py')
        else:
            print('❌ SOME CHECKS FAILED')
            print('\nCommon fixes:')
            print('  1. Make sure Isaac Sim is PLAYING (not paused)')
            print('  2. Check that Nova Carter robot is loaded')
            print('  3. Verify ROS2 Bridge is enabled in Isaac Sim')
            print('  4. Check Action Graph has Clock publisher node')
            print('\nDebug commands:')
            print('  ros2 topic list')
            print('  ros2 topic hz /front_3d_lidar/lidar_points')
            print('  ros2 run tf2_tools view_frames')
        
        print('=' * 70 + '\n')


def main(args=None):
    rclpy.init(args=args)
    node = PreflightCheck()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    

if __name__ == '__main__':
    main()
