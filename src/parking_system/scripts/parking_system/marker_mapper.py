#!/usr/bin/env python3
"""
Marker Mapper Node
Runs during SLAM mapping to record ArUco marker positions in map frame
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml
import os
import time
import tf2_ros
import geometry_msgs.msg as gm
import tf_transformations


class MarkerMapper(Node):
    def __init__(self):
        super().__init__('marker_mapper')
        
        # Parameters
        self.declare_parameter('map_name', 'parking_map')
        self.declare_parameter('marker_size', 10.0)  # cm
        self.declare_parameter('min_detection_confidence', 0.7)
        self.declare_parameter('max_marker_distance', 3.0)  # meters
        
        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.aruco_pose_sub = self.create_subscription(
            PoseStamped,
            '/aruco/marker_pose_update',
            self.aruco_pose_callback,
            10
        )
        
        self.slam_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/slam_toolbox/pose',  # SLAM pose during mapping
            self.slam_pose_callback,
            10
        )
        
        # Publishers
        self.marker_map_pub = self.create_publisher(PoseStamped, '/mapped_marker_pose', 10)
        self.status_pub = self.create_publisher(String, '/mapping_status', 10)
        
        # Services
        self.save_markers_srv = self.create_service(
            Trigger,
            '/save_mapped_markers',
            self.save_markers_callback
        )
        
        # State
        self.current_robot_pose = None
        self.mapped_markers = {}  # {id: {'pose': PoseStamped, 'detections': [], 'confidence': float}}
        self.map_name = self.get_parameter('map_name').value
        
        # Detection history for averaging
        self.detection_history = {}  # {id: [pose_transforms]}
        
        self.get_logger().info(f'Marker Mapper initialized for map: {self.map_name}')
        self.status_pub.publish(String(data=f'MAPPER_READY: Ready to map markers for {self.map_name}'))

    def slam_pose_callback(self, msg: PoseWithCovarianceStamped):
        """Update current robot pose from SLAM"""
        self.current_robot_pose = msg
    
    def aruco_pose_callback(self, msg: PoseStamped):
        """Handle ArUco marker detection during mapping"""
        if not self.current_robot_pose:
            self.get_logger().warn('No SLAM pose available, skipping marker mapping')
            return
        
        # Extract marker ID from frame_id
        frame_id = msg.header.frame_id
        if not frame_id.startswith('marker_'):
            return
        
        try:
            marker_id = int(frame_id.split('_')[1])
        except (IndexError, ValueError):
            return
        
        # Check distance constraint
        marker_distance = (msg.pose.position.x**2 + msg.pose.position.z**2)**0.5
        if marker_distance > self.get_parameter('max_marker_distance').value:
            self.get_logger().debug(f'Marker {marker_id} too far ({marker_distance:.2f}m), skipping')
            return
        
        # Transform marker pose from camera frame to map frame
        try:
            # Get transform from camera_link to map
            transform = self.tf_buffer.lookup_transform(
                'map',
                'camera_link',
                rclpy.time.Time()
            )
            
            # Apply transform to marker pose
            map_pose = self.transform_pose(msg.pose, transform)
            
            # Store detection
            if marker_id not in self.detection_history:
                self.detection_history[marker_id] = []
            
            self.detection_history[marker_id].append({
                'pose': map_pose,
                'timestamp': time.time(),
                'robot_pose': self.current_robot_pose.pose.pose
            })
            
            # Update mapped marker with averaged position
            self.update_mapped_marker(marker_id)
            
            self.get_logger().info(f'Mapped marker {marker_id} at map position: '
                                 f'({map_pose.position.x:.2f}, {map_pose.position.y:.2f})')
            
            # Publish mapped pose
            mapped_msg = PoseStamped()
            mapped_msg.header.frame_id = 'map'
            mapped_msg.header.stamp = self.get_clock().now().to_msg()
            mapped_msg.pose = map_pose
            self.marker_map_pub.publish(mapped_msg)
            
            self.status_pub.publish(String(data=f'MAPPED: Marker {marker_id} position recorded'))
            
        except tf2_ros.LookupException as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(f'TF extrapolation failed: {e}')
    
    def transform_pose(self, pose, transform):
        """Transform pose using TF transform"""
        # Create transform matrix
        trans = transform.transform.translation
        rot = transform.transform.rotation
        
        # Convert to transformation matrix
        transform_matrix = tf_transformations.quaternion_matrix([
            rot.x, rot.y, rot.z, rot.w
        ])
        transform_matrix[0:3, 3] = [trans.x, trans.y, trans.z]
        
        # Convert pose to homogeneous coordinates
        pose_matrix = tf_transformations.quaternion_matrix([
            pose.orientation.x, pose.orientation.y, 
            pose.orientation.z, pose.orientation.w
        ])
        pose_matrix[0:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        
        # Apply transformation
        result_matrix = transform_matrix @ pose_matrix
        
        # Extract pose from result
        result_pose = gm.Pose()
        result_pose.position.x = result_matrix[0, 3]
        result_pose.position.y = result_matrix[1, 3]
        result_pose.position.z = result_matrix[2, 3]
        
        quaternion = tf_transformations.quaternion_from_matrix(result_matrix)
        result_pose.orientation.x = quaternion[0]
        result_pose.orientation.y = quaternion[1]
        result_pose.orientation.z = quaternion[2]
        result_pose.orientation.w = quaternion[3]
        
        return result_pose
    
    def update_mapped_marker(self, marker_id):
        """Update mapped marker position using detection history"""
        detections = self.detection_history[marker_id]
        if len(detections) < 3:  # Need at least 3 detections for stability
            return
        
        # Simple averaging of recent detections (last 10)
        recent_detections = detections[-10:]
        
        # Average positions
        avg_x = sum(d.pose.position.x for d in recent_detections) / len(recent_detections)
        avg_y = sum(d.pose.position.y for d in recent_detections) / len(recent_detections)
        avg_z = sum(d.pose.position.z for d in recent_detections) / len(recent_detections)
        
        # Use orientation from most recent detection
        latest_pose = recent_detections[-1]['pose']
        
        # Create averaged pose
        avg_pose = gm.Pose()
        avg_pose.position.x = avg_x
        avg_pose.position.y = avg_y
        avg_pose.position.z = avg_z
        avg_pose.orientation = latest_pose.orientation
        
        # Calculate confidence based on position variance
        positions = [(d.pose.position.x, d.pose.position.y) for d in recent_detections]
        variance = sum(
            ((x - avg_x)**2 + (y - avg_y)**2) 
            for x, y in positions
        ) / len(positions)
        
        # Higher variance = lower confidence
        confidence = max(0.1, 1.0 - variance * 10)  # Scale variance to confidence
        
        self.mapped_markers[marker_id] = {
            'pose': avg_pose,
            'confidence': confidence,
            'detection_count': len(detections),
            'last_update': time.time()
        }
    
    def save_markers_callback(self, request, response):
        """Save mapped markers to file"""
        try:
            if not self.mapped_markers:
                response.success = False
                response.message = 'No markers mapped yet'
                return response
            
            # Create markers file path
            markers_file = f'/home/autonexa/maps/{self.map_name}_markers.yaml'
            os.makedirs(os.path.dirname(markers_file), exist_ok=True)
            
            # Convert to serializable format
            serializable_markers = {}
            for marker_id, data in self.mapped_markers.items():
                pose = data['pose']
                serializable_markers[marker_id] = {
                    'position': {
                        'x': pose.position.x,
                        'y': pose.position.y,
                        'z': pose.position.z
                    },
                    'orientation': {
                        'x': pose.orientation.x,
                        'y': pose.orientation.y,
                        'z': pose.orientation.z,
                        'w': pose.orientation.w
                    },
                    'confidence': data['confidence'],
                    'detection_count': data['detection_count'],
                    'last_update': data['last_update']
                }
            
            with open(markers_file, 'w') as f:
                yaml.dump({
                    'map_name': self.map_name,
                    'creation_time': time.time(),
                    'markers': serializable_markers
                }, f, default_flow_style=False)
            
            response.success = True
            response.message = f'Saved {len(self.mapped_markers)} markers to {markers_file}'
            self.get_logger().info(response.message)
            
        except Exception as e:
            response.success = False
            response.message = f'Failed to save markers: {str(e)}'
            self.get_logger().error(response.message)
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MarkerMapper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Auto-save on shutdown
        node.save_markers_callback(None, type('Response', (), {'success': False, 'message': ''})())
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
