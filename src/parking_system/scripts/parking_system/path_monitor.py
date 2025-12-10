#!/usr/bin/env python3
"""
Path Monitor Node
Monitors if the robot is following the planned path and provides feedback
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import String, Float32
import math


class PathMonitor(Node):
    def __init__(self):
        super().__init__('path_monitor')
        
        # Subscribers
        # Nav2 publishes the global plan to /plan topic
        self.path_sub = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10
        )
        
        # Also subscribe to Nav2's global plan topic (alternative)
        self.nav2_path_sub = self.create_subscription(
            Path,
            '/global_plan',
            self.path_callback,
            10
        )
        
        self.current_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',  # From AMCL localization (PoseWithCovarianceStamped)
            self.pose_callback,
            10
        )
        
        # Publishers for feedback
        self.feedback_pub = self.create_publisher(String, '/path_feedback', 10)
        self.distance_pub = self.create_publisher(Float32, '/path_distance_error', 10)
        self.angular_pub = self.create_publisher(Float32, '/path_angular_error', 10)
        
        self.current_path = None
        self.current_pose = None
        self.path_tolerance = 0.15  # 15cm tolerance for path following
        
        self.get_logger().info('Path Monitor initialized')
        
    def path_callback(self, msg):
        """Store the current planned path"""
        self.current_path = msg
        if self.current_path.poses:
            self.get_logger().info(f'Received new path with {len(self.current_path.poses)} waypoints')
    
    def pose_callback(self, msg):
        """Update current robot pose and check path following"""
        # Convert PoseWithCovarianceStamped to PoseStamped
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self.current_pose = pose_stamped
        
        if self.current_path and self.current_path.poses:
            self.check_path_following()
    
    def euclidean_distance(self, pose1, pose2):
        """Calculate Euclidean distance between two poses"""
        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y
        return math.sqrt(dx*dx + dy*dy)
    
    def yaw_from_quaternion(self, orientation):
        """Extract yaw angle from quaternion"""
        # This is a simplified version - in production use tf_transformations
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def check_path_following(self):
        """Check if robot is following the path and provide feedback"""
        if not self.current_pose or not self.current_path:
            return
        
        # Find nearest waypoint on the path
        min_distance = float('inf')
        nearest_waypoint_idx = 0
        
        for i, waypoint in enumerate(self.current_path.poses):
            distance = self.euclidean_distance(self.current_pose, waypoint)
            if distance < min_distance:
                min_distance = distance
                nearest_waypoint_idx = i
        
        # Calculate distance error
        distance_error = min_distance
        
        # Calculate angular error (if near a waypoint)
        angular_error = 0.0
        if nearest_waypoint_idx < len(self.current_path.poses):
            waypoint = self.current_path.poses[nearest_waypoint_idx]
            current_yaw = self.yaw_from_quaternion(self.current_pose.pose.orientation)
            target_yaw = self.yaw_from_quaternion(waypoint.pose.orientation)
            angular_error = abs(target_yaw - current_yaw)
            if angular_error > math.pi:
                angular_error = 2 * math.pi - angular_error
        
        # Publish errors
        distance_msg = Float32()
        distance_msg.data = distance_error
        self.distance_pub.publish(distance_msg)
        
        angular_msg = Float32()
        angular_msg.data = angular_error
        self.angular_pub.publish(angular_msg)
        
        # Generate feedback message
        feedback_msg = String()
        
        if distance_error < self.path_tolerance:
            feedback_msg.data = f'ON_PATH: Distance error: {distance_error:.3f}m, Angular error: {angular_error:.3f}rad'
            self.get_logger().info(f'✓ Robot is on path (error: {distance_error:.3f}m)')
        else:
            feedback_msg.data = f'OFF_PATH: Distance error: {distance_error:.3f}m, Angular error: {angular_error:.3f}rad'
            self.get_logger().warn(f'✗ Robot is off path (error: {distance_error:.3f}m)')
        
        self.feedback_pub.publish(feedback_msg)
        
        # Progress feedback
        progress = (nearest_waypoint_idx / len(self.current_path.poses)) * 100
        if progress > 90:
            progress_msg = String()
            progress_msg.data = f'APPROACHING_GOAL: {progress:.1f}% complete'
            self.feedback_pub.publish(progress_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

