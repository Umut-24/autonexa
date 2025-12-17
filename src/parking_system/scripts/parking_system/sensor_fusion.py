#!/usr/bin/env python3
"""
Sensor Fusion Node
Fuses ArUco marker detection with LiDAR-based localization using Kalman filter
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
import numpy as np
import time
from collections import deque
import tf2_ros
import tf_transformations


class SensorFusion(Node):
    def __init__(self):
        super().__init__('sensor_fusion')
        
        # Publishers
        self.fused_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/fused_pose', 10)
        self.odom_pub = self.create_publisher(Odometry, '/fused_odom', 10)
        
        # Subscribers
        self.aruco_pose_sub = self.create_subscription(
            PoseStamped,
            '/aruco_marker_pose',
            self.aruco_pose_callback,
            10
        )
        
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )
        
        self.target_id_sub = self.create_subscription(
            Int32,
            '/target_marker_id',
            self.target_id_callback,
            10
        )
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Kalman filter state
        self.state = np.zeros(6)  # [x, y, theta, vx, vy, vtheta]
        self.covariance = np.eye(6) * 1000  # High initial uncertainty
        
        # Process noise
        self.Q = np.diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])  # Process noise
        
        # Measurement noise
        self.R_aruco = np.diag([0.05, 0.05, 0.05])  # ArUco measurement noise (x, y, theta)
        self.R_amcl = np.diag([0.01, 0.01, 0.01])   # AMCL measurement noise
        
        # Latest measurements
        self.latest_aruco_pose = None
        self.latest_amcl_pose = None
        self.last_update_time = time.time()
        
        # Target marker info
        self.target_id = 0
        self.marker_positions = {}  # Store known marker positions in map frame
        
        # Timer for fusion updates
        self.timer = self.create_timer(0.05, self.fusion_callback)  # 20 Hz
        
        self.get_logger().info('Sensor Fusion initialized')
    
    def target_id_callback(self, msg):
        """Update target marker ID"""
        self.target_id = msg.data
        self.get_logger().info(f'Target marker ID set to: {self.target_id}')
    
    def aruco_pose_callback(self, msg):
        """Handle ArUco pose measurement"""
        self.latest_aruco_pose = msg
        
        # If we have a known position for this marker, convert to absolute pose
        if self.target_id in self.marker_positions:
            marker_pos = self.marker_positions[self.target_id]
            
            # ArUco gives pose relative to camera, we need to transform to map frame
            # For simplicity, assume camera is on robot and marker position is known
            # In reality, you'd need proper TF transformations
            
            # This is a simplified version - in practice you'd use TF to transform
            # from camera frame to map frame
            absolute_pose = PoseWithCovarianceStamped()
            absolute_pose.header = msg.header
            absolute_pose.header.frame_id = "map"
            absolute_pose.pose.pose.position.x = marker_pos['x']
            absolute_pose.pose.pose.position.y = marker_pos['y']
            absolute_pose.pose.pose.orientation = msg.pose.orientation
            
            # Set covariance (ArUco is less accurate for absolute position)
            absolute_pose.pose.covariance = [0.05, 0, 0, 0, 0, 0,
                                           0, 0.05, 0, 0, 0, 0,
                                           0, 0, 0.05, 0, 0, 0,
                                           0, 0, 0, 0.1, 0, 0,
                                           0, 0, 0, 0, 0.1, 0,
                                           0, 0, 0, 0, 0, 0.1]
            
            self.latest_aruco_pose = absolute_pose
    
    def amcl_pose_callback(self, msg):
        """Handle AMCL pose measurement"""
        self.latest_amcl_pose = msg
    
    def fusion_callback(self):
        """Main fusion loop using Kalman filter"""
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # Prediction step
        self.predict(dt)
        
        # Update with measurements
        if self.latest_aruco_pose:
            self.update_aruco()
            self.latest_aruco_pose = None  # Clear after use
        
        if self.latest_amcl_pose:
            self.update_amcl()
            self.latest_amcl_pose = None  # Clear after use
        
        # Publish fused pose
        self.publish_fused_pose()
    
    def predict(self, dt):
        """Kalman filter prediction step"""
        # State transition matrix (constant velocity model)
        F = np.eye(6)
        F[0, 3] = dt  # x += vx * dt
        F[1, 4] = dt  # y += vy * dt
        F[2, 5] = dt  # theta += vtheta * dt
        
        # Predict state
        self.state = F @ self.state
        
        # Predict covariance
        self.covariance = F @ self.covariance @ F.T + self.Q
    
    def update_aruco(self):
        """Update with ArUco measurement"""
        if not self.latest_aruco_pose:
            return
        
        # Measurement: [x, y, theta]
        z = np.array([
            self.latest_aruco_pose.pose.pose.position.x,
            self.latest_aruco_pose.pose.pose.position.y,
            self.quaternion_to_yaw(self.latest_aruco_pose.pose.pose.orientation)
        ])
        
        # Measurement matrix
        H = np.zeros((3, 6))
        H[0, 0] = 1  # x
        H[1, 1] = 1  # y
        H[2, 2] = 1  # theta
        
        # Innovation
        y = z - H @ self.state
        
        # Innovation covariance
        S = H @ self.covariance @ H.T + self.R_aruco
        
        # Kalman gain
        K = self.covariance @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.state = self.state + K @ y
        
        # Update covariance
        I = np.eye(6)
        self.covariance = (I - K @ H) @ self.covariance
    
    def update_amcl(self):
        """Update with AMCL measurement"""
        if not self.latest_amcl_pose:
            return
        
        # Measurement: [x, y, theta]
        z = np.array([
            self.latest_amcl_pose.pose.pose.position.x,
            self.latest_amcl_pose.pose.pose.position.y,
            self.quaternion_to_yaw(self.latest_amcl_pose.pose.pose.orientation)
        ])
        
        # Measurement matrix
        H = np.zeros((3, 6))
        H[0, 0] = 1  # x
        H[1, 1] = 1  # y
        H[2, 2] = 1  # theta
        
        # Innovation
        y = z - H @ self.state
        
        # Innovation covariance
        S = H @ self.covariance @ H.T + self.R_amcl
        
        # Kalman gain
        K = self.covariance @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.state = self.state + K @ y
        
        # Update covariance
        I = np.eye(6)
        self.covariance = (I - K @ H) @ self.covariance
    
    def publish_fused_pose(self):
        """Publish the fused pose"""
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Position
        pose_msg.pose.pose.position.x = self.state[0]
        pose_msg.pose.pose.position.y = self.state[1]
        pose_msg.pose.pose.position.z = 0.0
        
        # Orientation (from theta)
        quaternion = tf_transformations.quaternion_from_euler(0, 0, self.state[2])
        pose_msg.pose.pose.orientation.x = quaternion[0]
        pose_msg.pose.pose.orientation.y = quaternion[1]
        pose_msg.pose.pose.orientation.z = quaternion[2]
        pose_msg.pose.pose.orientation.w = quaternion[3]
        
        # Covariance (flatten the 6x6 matrix to 36-element array)
        pose_msg.pose.covariance = self.covariance.flatten().tolist()
        
        self.fused_pose_pub.publish(pose_msg)
        
        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header = pose_msg.header
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose = pose_msg.pose
        odom_msg.twist.twist.linear.x = self.state[3]
        odom_msg.twist.twist.linear.y = self.state[4]
        odom_msg.twist.twist.angular.z = self.state[5]
        
        self.odom_pub.publish(odom_msg)
        
        # Broadcast TF
        transform = TransformStamped()
        transform.header = pose_msg.header
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = self.state[0]
        transform.transform.translation.y = self.state[1]
        transform.transform.translation.z = 0.0
        transform.transform.rotation = pose_msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(transform)
    
    def quaternion_to_yaw(self, orientation):
        """Convert quaternion to yaw angle"""
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(q)
        return yaw
    
    def add_marker_position(self, marker_id, x, y):
        """Add known marker position in map frame"""
        self.marker_positions[marker_id] = {'x': x, 'y': y}


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusion()
    
    # Add some example marker positions (you'd load these from a config file)
    # These should be measured positions of markers in your map
    node.add_marker_position(0, 1.0, 0.5)   # Example positions
    node.add_marker_position(1, 1.8, 0.5)
    node.add_marker_position(2, 1.8, 1.8)
    node.add_marker_position(3, 0.5, 1.8)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
