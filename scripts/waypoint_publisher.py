#!/usr/bin/env python3
# =============================================================================
# Waypoint Publisher Node
# Author: Andrés Islas Bravo
# Description: Publishes waypoints for autonomous navigation missions
# =============================================================================

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from visualization_msgs.msg import Marker, MarkerArray
import yaml
import os
import math


class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        
        # Action clients
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        
        # Visualization publisher
        self.marker_pub = self.create_publisher(MarkerArray, 'waypoints', 10)
        
        # Parameters
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('loop', False)
        
        self.waypoints = []
        
        # Load waypoints from file if specified
        waypoints_file = self.get_parameter('waypoints_file').value
        if waypoints_file and os.path.exists(waypoints_file):
            self.load_waypoints(waypoints_file)
        
        self.get_logger().info('Waypoint Publisher initialized')
    
    def load_waypoints(self, filepath: str):
        """Load waypoints from YAML file"""
        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)
        
        self.waypoints = []
        for wp in data.get('waypoints', []):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(wp['x'])
            pose.pose.position.y = float(wp['y'])
            pose.pose.position.z = 0.0
            
            # Convert yaw to quaternion
            yaw = float(wp.get('yaw', 0.0))
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            
            self.waypoints.append(pose)
        
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
        self.publish_waypoint_markers()
    
    def publish_waypoint_markers(self):
        """Publish visualization markers for waypoints"""
        marker_array = MarkerArray()
        
        for i, wp in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'waypoints'
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = wp.pose
            marker.scale.x = 0.5
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)
            
            # Text label
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'waypoint_labels'
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose = wp.pose
            text_marker.pose.position.z = 0.5
            text_marker.scale.z = 0.3
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.text = f'WP{i}'
            marker_array.markers.append(text_marker)
        
        self.marker_pub.publish(marker_array)
    
    def follow_waypoints_mission(self):
        """Execute follow waypoints mission"""
        if not self.waypoints:
            self.get_logger().warn('No waypoints loaded!')
            return
        
        self.follow_waypoints_client.wait_for_server()
        
        goal = FollowWaypoints.Goal()
        goal.poses = self.waypoints
        
        self.get_logger().info(f'Starting mission with {len(self.waypoints)} waypoints')
        future = self.follow_waypoints_client.send_goal_async(
            goal, 
            feedback_callback=self.waypoint_feedback_callback
        )
        future.add_done_callback(self.waypoint_response_callback)
    
    def waypoint_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'At waypoint {feedback.current_waypoint}')
    
    def waypoint_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Mission rejected!')
            return
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.waypoint_result_callback)
    
    def waypoint_result_callback(self, future):
        result = future.result().result
        missed = result.missed_waypoints
        if missed:
            self.get_logger().warn(f'Missed waypoints: {missed}')
        else:
            self.get_logger().info('All waypoints reached!')
        
        if self.get_parameter('loop').value:
            self.get_logger().info('Looping...')
            self.follow_waypoints_mission()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
