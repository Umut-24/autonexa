#!/usr/bin/env python3
"""
Enhanced Sensor Fusion Node
Uses robot_localization EKF for proper sensor fusion with intermittent ArUco detections
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import time
from collections import deque


class EnhancedSensorFusion(Node):
    def __init__(self):
        super().__init__('enhanced_sensor_fusion')
        
        # Publishers (these will be consumed by robot_localization EKF)
        self.aruco_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/aruco_pose', 10)
        self.fused_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/fused_pose', 10)
        
        # Subscribers
        self.marker_map_sub = self.create_subscription(
            PoseStamped,
            '/aruco/marker_pose_update',
            self.marker_update_callback,
            10
        )
        
        self.ekf_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/odometry/filtered',  # From robot_localization EKF
            self.ekf_pose_callback,
            10
        )
        
        # Parameters for fusion
        self.declare_parameter('aruco_covariance', [0.05, 0.05, 0.05, 0.1, 0.1, 0.1])  # x,y,z,roll,pitch,yaw
        self.declare_parameter('detection_timeout', 2.0)  # seconds
        
        # State
        self.last_aruco_detection = None
        self.last_aruco_time = 0
        self.aruco_covariance = self.get_parameter('aruco_covariance').value
        self.detection_timeout = self.get_parameter('detection_timeout').value
        
        # Marker tracking for intermittent detections
        self.marker_tracks = {}  # {id: {'pose': PoseStamped, 'last_seen': time, 'velocity': [vx,vy]}}
        
        self.get_logger().info('Enhanced Sensor Fusion initialized')
    
    def marker_update_callback(self, msg: PoseStamped):
        """Handle marker pose updates from detector"""
        current_time = time.time()
        
        # Extract marker ID from frame_id
        frame_id = msg.header.frame_id
        if not frame_id.startswith('marker_'):
            return
        
        try:
            marker_id = int(frame_id.split('_')[1])
        except (IndexError, ValueError):
            return
        
        # Update marker track
        if marker_id not in self.marker_tracks:
            self.marker_tracks[marker_id] = {
                'pose': msg,
                'last_seen': current_time,
                'velocity': [0.0, 0.0]
            }
        else:
            # Calculate velocity for prediction
            old_pose = self.marker_tracks[marker_id]['pose']
            dt = current_time - self.marker_tracks[marker_id]['last_seen']
            
            if dt > 0:
                vx = (msg.pose.position.x - old_pose.pose.position.x) / dt
                vy = (msg.pose.position.y - old_pose.pose.position.y) / dt
                self.marker_tracks[marker_id]['velocity'] = [vx, vy]
            
            self.marker_tracks[marker_id]['pose'] = msg
            self.marker_tracks[marker_id]['last_seen'] = current_time
        
        # Publish as pose measurement for EKF
        aruco_pose = PoseWithCovarianceStamped()
        aruco_pose.header = msg.header
        aruco_pose.header.frame_id = "map"  # Assume marker poses are in map frame
        aruco_pose.pose.pose = msg.pose
        
        # Set covariance matrix (6x6)
        covariance = [0.0] * 36
        for i, cov in enumerate(self.aruco_covariance):
            covariance[i*6 + i] = cov  # Diagonal elements
        aruco_pose.pose.covariance = covariance
        
        self.aruco_pose_pub.publish(aruco_pose)
        self.last_aruco_detection = aruco_pose
        self.last_aruco_time = current_time
        
        self.get_logger().debug(f'Published ArUco pose for marker {marker_id}')
    
    def ekf_pose_callback(self, msg: PoseWithCovarianceStamped):
        """Handle fused pose from EKF"""
        # Republish as our fused pose
        self.fused_pose_pub.publish(msg)
        
        # Clean up old marker tracks
        current_time = time.time()
        to_remove = []
        for marker_id, track in self.marker_tracks.items():
            if current_time - track['last_seen'] > self.detection_timeout * 5:
                to_remove.append(marker_id)
        
        for marker_id in to_remove:
            del self.marker_tracks[marker_id]
            self.get_logger().info(f'Removed stale track for marker {marker_id}')
    
    def predict_marker_positions(self):
        """Predict marker positions during occlusions (for future use)"""
        current_time = time.time()
        
        for marker_id, track in self.marker_tracks.items():
            dt = current_time - track['last_seen']
            
            if dt < self.detection_timeout:
                # Predict position using velocity
                predicted_pose = PoseStamped()
                predicted_pose.header = track['pose'].header
                predicted_pose.header.stamp = self.get_clock().now().to_msg()
                predicted_pose.pose = track['pose'].pose
                
                # Simple constant velocity prediction
                predicted_pose.pose.position.x += track['velocity'][0] * dt
                predicted_pose.pose.position.y += track['velocity'][1] * dt
                
                # Could publish predicted poses with higher covariance
                # For now, just maintain tracks


def main(args=None):
    rclpy.init(args=args)
    node = EnhancedSensorFusion()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()