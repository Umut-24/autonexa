#!/usr/bin/env python3
"""
ROS2 ArUco Detection Node
Detects ArUco markers and publishes their poses for navigation
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import threading
import time
from collections import deque


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        
        # Parameters
        self.declare_parameter('marker_size', 10.0)  # cm
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_width', 1280)
        self.declare_parameter('frame_height', 720)
        self.declare_parameter('known_distance', 50.0)  # cm for calibration
        
        # Publishers
        self.marker_pose_pub = self.create_publisher(PoseStamped, '/aruco_marker_pose', 10)
        self.all_markers_pub = self.create_publisher(PoseArray, '/aruco_all_markers', 10)
        self.image_pub = self.create_publisher(Image, '/aruco_debug_image', 10)
        self.telemetry_pub = self.create_publisher(String, '/aruco_telemetry', 10)
        
        # Subscribers
        self.target_id_sub = self.create_subscription(
            Int32,
            '/target_marker_id',
            self.target_id_callback,
            10
        )
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Detection parameters
        self.marker_size = self.get_parameter('marker_size').value
        self.target_id = 0
        self.calibrated = False
        self.distance_scale = None
        self.known_distance = self.get_parameter('known_distance').value
        
        # Camera setup
        self.cap = None
        self.setup_camera()
        
        # ArUco setup
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        
        # Marker storage
        self.detected_markers = {}  # {id: {'pose': PoseStamped, 'last_seen': time}}
        
        # Timer for detection
        self.timer = self.create_timer(0.1, self.detect_callback)  # 10 Hz
        
        self.get_logger().info('ArUco Detector initialized')
    
    def setup_camera(self):
        """Initialize camera"""
        camera_index = self.get_parameter('camera_index').value
        frame_width = self.get_parameter('frame_width').value
        frame_height = self.get_parameter('frame_height').value
        
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            self.get_logger().error(f'Could not open camera {camera_index}')
            return
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
        self.get_logger().info(f'Camera initialized: {frame_width}x{frame_height}')
    
    def target_id_callback(self, msg):
        """Update target marker ID"""
        self.target_id = msg.data
        self.get_logger().info(f'Target marker ID set to: {self.target_id}')
    
    def get_calibration_matrix(self, width, height):
        """Get camera calibration matrix (dummy for now)"""
        focal_length = width
        cx = width / 2
        cy = height / 2
        cam_mtx = np.array([
            [focal_length, 0, cx],
            [0, focal_length, cy],
            [0, 0, 1]
        ], dtype=np.float32)
        dist = np.zeros((4, 1))
        return cam_mtx, dist
    
    def detect_callback(self):
        """Main detection loop"""
        if self.cap is None or not self.cap.isOpened():
            return
        
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Could not read frame')
            return
        
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect markers
        corners, ids, rejected = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.parameters
        )
        
        # Draw detected markers
        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids, (100, 100, 100))
        
        # Process detections
        current_time = self.get_clock().now()
        pose_array = PoseArray()
        pose_array.header.frame_id = "camera_link"
        pose_array.header.stamp = current_time.to_msg()
        
        target_pose = None
        
        if ids is not None:
            h, w = frame.shape[:2]
            cam_mtx, dist = self.get_calibration_matrix(w, h)
            
            for i, marker_id in enumerate(ids.flatten()):
                # Define marker corners in 3D
                obj_points = np.array([
                    [-self.marker_size/2, self.marker_size/2, 0],
                    [self.marker_size/2, self.marker_size/2, 0],
                    [self.marker_size/2, -self.marker_size/2, 0],
                    [-self.marker_size/2, -self.marker_size/2, 0]
                ], dtype=np.float32)
                
                # Solve PnP
                success, rvec, tvec = cv2.solvePnP(
                    obj_points, corners[i], cam_mtx, dist
                )
                
                if success:
                    # Convert to ROS pose
                    pose = self.rvec_tvec_to_pose(rvec, tvec)
                    
                    # Calibration
                    dist_raw = np.linalg.norm(tvec)
                    if not self.calibrated:
                        self.distance_scale = self.known_distance / dist_raw
                        self.calibrated = True
                        self.get_logger().info(f'Calibrated with scale: {self.distance_scale}')
                    
                    if self.calibrated:
                        # Scale the pose
                        pose.position.x *= self.distance_scale
                        pose.position.y *= self.distance_scale
                        pose.position.z *= self.distance_scale
                        
                        # Store marker
                        pose_stamped = PoseStamped()
                        pose_stamped.header = pose_array.header
                        pose_stamped.pose = pose
                        
                        self.detected_markers[marker_id] = {
                            'pose': pose_stamped,
                            'last_seen': time.time()
                        }
                        
                        pose_array.poses.append(pose)
                        
                        # Check if this is target
                        if marker_id == self.target_id:
                            target_pose = pose_stamped
                            
                            # Draw target info on frame
                            c = corners[i][0]
                            center = np.mean(c, axis=0).astype(int)
                            distance = np.linalg.norm([pose.position.x, pose.position.z])
                            bearing = np.degrees(np.arctan2(pose.position.x, pose.position.z))
                            
                            cv2.putText(frame, f"TARGET ID {marker_id}", 
                                      (center[0]-50, center[1]-30),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                            cv2.putText(frame, f"Dist: {distance:.1f}cm", 
                                      (center[0]-50, center[1]-10),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                            cv2.putText(frame, f"Bearing: {bearing:.1f}°", 
                                      (center[0]-50, center[1]+10),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
        
        # Publish target pose if available
        if target_pose:
            self.marker_pose_pub.publish(target_pose)
        
        # Publish all markers
        self.all_markers_pub.publish(pose_array)
        
        # Publish debug image
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            img_msg.header = pose_array.header
            self.image_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish image: {e}')
        
        # Publish telemetry
        telemetry = {
            'target_id': self.target_id,
            'detected_markers': list(self.detected_markers.keys()),
            'calibrated': self.calibrated
        }
        if target_pose:
            telemetry.update({
                'distance': np.linalg.norm([
                    target_pose.pose.position.x,
                    target_pose.pose.position.z
                ]),
                'bearing': np.degrees(np.arctan2(
                    target_pose.pose.position.x,
                    target_pose.pose.position.z
                ))
            })
        
        self.telemetry_pub.publish(String(data=str(telemetry)))
    
    def rvec_tvec_to_pose(self, rvec, tvec):
        """Convert rotation and translation vectors to ROS Pose"""
        # Convert rotation vector to quaternion
        rvec = rvec.flatten()
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        
        # Create quaternion from rotation matrix
        qw = np.sqrt(1 + rotation_matrix[0,0] + rotation_matrix[1,1] + rotation_matrix[2,2]) / 2
        qx = (rotation_matrix[2,1] - rotation_matrix[1,2]) / (4 * qw)
        qy = (rotation_matrix[0,2] - rotation_matrix[2,0]) / (4 * qw)
        qz = (rotation_matrix[1,0] - rotation_matrix[0,1]) / (4 * qw)
        
        pose = Pose()
        pose.position.x = float(tvec[0])
        pose.position.y = float(tvec[1])
        pose.position.z = float(tvec[2])
        pose.orientation.w = float(qw)
        pose.orientation.x = float(qx)
        pose.orientation.y = float(qy)
        pose.orientation.z = float(qz)
        
        return pose
    
    def __del__(self):
        if self.cap:
            self.cap.release()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()</content>
<parameter name="filePath">c:\Users\Anıl\OneDrive\Belgeler\GitHub\autonexa\src\parking_system\scripts\parking_system\aruco_detector.py