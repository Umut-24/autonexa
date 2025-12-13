#!/usr/bin/env python3
"""
Marker Map Manager Node
Manages persistent storage and real-time updates of ArUco marker positions
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml
import os
import time
from typing import Dict, Optional


class MarkerMapManager(Node):
    def __init__(self):
        super().__init__('marker_map_manager')
        
        # Parameters
        self.declare_parameter('map_name', 'parking_map')
        self.declare_parameter('auto_load_map_markers', True)
        
        # Publishers
        self.marker_array_pub = self.create_publisher(PoseArray, '/marker_map', 10)
        self.marker_pose_pub = self.create_publisher(PoseStamped, '/marker_map/poses', 10)
        self.status_pub = self.create_publisher(String, '/marker_map_status', 10)
        
        # Subscribers
        self.marker_update_sub = self.create_subscription(
            PoseStamped,
            '/aruco/marker_pose_update',
            self.marker_update_callback,
            10
        )
        
        # Services
        self.save_map_srv = self.create_service(
            Trigger,
            '/save_marker_map',
            self.save_map_callback
        )
        
        self.load_map_srv = self.create_service(
            Trigger,
            '/load_marker_map',
            self.load_map_callback
        )
        
        # Marker storage
        self.marker_map: Dict[int, Dict] = {}  # {id: {'pose': PoseStamped, 'last_update': float, 'confidence': float}}
        self.map_name = self.get_parameter('map_name').value
        self.map_file = f'maps/{self.map_name}_markers.yaml'
        self.map_frame = 'map'  # Fixed frame
        self.update_timeout = 30.0  # seconds
        self.auto_load = self.get_parameter('auto_load_map_markers').value
        
        # Load existing map if auto_load is enabled
        if self.auto_load:
            self.load_map()
        
        # Timer for periodic publishing
        self.timer = self.create_timer(1.0, self.publish_marker_array)
        
        self.get_logger().info(f'Marker Map Manager initialized with {len(self.marker_map)} markers')
    
    def marker_update_callback(self, msg: PoseStamped):
        """Update marker position from detection"""
        # Extract marker ID from frame_id (format: marker_X)
        frame_id = msg.header.frame_id
        if not frame_id.startswith('marker_'):
            self.get_logger().warn(f'Invalid frame_id format: {frame_id}')
            return
        
        try:
            marker_id = int(frame_id.split('_')[1])
        except (IndexError, ValueError):
            self.get_logger().warn(f'Could not parse marker ID from {frame_id}')
            return
        
        # Update or add marker
        current_time = time.time()
        
        if marker_id in self.marker_map:
            # Update existing marker
            old_pose = self.marker_map[marker_id]['pose']
            
            # Simple fusion: weighted average based on confidence
            # In production, use more sophisticated fusion
            confidence = 0.8  # New detection confidence
            old_confidence = self.marker_map[marker_id]['confidence']
            
            # Weighted position update
            new_pose = PoseStamped()
            new_pose.header = msg.header
            new_pose.header.frame_id = self.map_frame
            new_pose.pose.position.x = (
                old_confidence * old_pose.pose.position.x + 
                confidence * msg.pose.position.x
            ) / (old_confidence + confidence)
            new_pose.pose.position.y = (
                old_confidence * old_pose.pose.position.y + 
                confidence * msg.pose.position.y
            ) / (old_confidence + confidence)
            new_pose.pose.position.z = msg.pose.position.z  # Keep new Z
            
            # Orientation update (simplified)
            new_pose.pose.orientation = msg.pose.orientation
            
            self.marker_map[marker_id] = {
                'pose': new_pose,
                'last_update': current_time,
                'confidence': min(old_confidence + confidence, 1.0)
            }
            
            self.get_logger().info(f'Updated marker {marker_id} position')
            
        else:
            # Add new marker
            map_pose = PoseStamped()
            map_pose.header = msg.header
            map_pose.header.frame_id = self.map_frame
            map_pose.pose = msg.pose
            
            self.marker_map[marker_id] = {
                'pose': map_pose,
                'last_update': current_time,
                'confidence': 0.5  # Initial confidence
            }
            
            self.get_logger().info(f'Added new marker {marker_id} to map')
        
        # Publish status
        self.status_pub.publish(String(data=f'UPDATED: Marker {marker_id}'))
    
    def publish_marker_array(self):
        """Publish current marker map as PoseArray and individual poses"""
        marker_array = PoseArray()
        marker_array.header.frame_id = self.map_frame
        marker_array.header.stamp = self.get_clock().now().to_msg()
        
        current_time = time.time()
        
        for marker_id, data in self.marker_map.items():
            # Check if marker is still valid (not too old)
            if current_time - data['last_update'] < self.update_timeout:
                marker_array.poses.append(data['pose'].pose)
                
                # Publish individual pose with marker ID in frame_id
                pose_msg = PoseStamped()
                pose_msg.header = data['pose'].header
                pose_msg.header.frame_id = f'marker_{marker_id}'
                pose_msg.pose = data['pose'].pose
                self.marker_pose_pub.publish(pose_msg)
        
        self.marker_array_pub.publish(marker_array)
    
    def save_map_callback(self, request, response):
        """Save current marker map to file"""
        try:
            # Convert to serializable format
            serializable_map = {}
            for marker_id, data in self.marker_map.items():
                pose = data['pose']
                serializable_map[marker_id] = {
                    'position': {
                        'x': pose.pose.position.x,
                        'y': pose.pose.position.y,
                        'z': pose.pose.position.z
                    },
                    'orientation': {
                        'x': pose.pose.orientation.x,
                        'y': pose.pose.orientation.y,
                        'z': pose.pose.orientation.z,
                        'w': pose.pose.orientation.w
                    },
                    'last_update': data['last_update'],
                    'confidence': data['confidence']
                }
            
            with open(self.map_file, 'w') as f:
                yaml.dump(serializable_map, f, default_flow_style=False)
            
            response.success = True
            response.message = f'Saved {len(self.marker_map)} markers to {self.map_file}'
            self.get_logger().info(response.message)
            
        except Exception as e:
            response.success = False
            response.message = f'Failed to save map: {str(e)}'
            self.get_logger().error(response.message)
        
        return response
    
    def load_map_callback(self, request, response):
        """Load marker map from file"""
        try:
            if not os.path.exists(self.map_file):
                response.success = False
                response.message = f'Map file {self.map_file} does not exist'
                return response
            
            with open(self.map_file, 'r') as f:
                loaded_map = yaml.safe_load(f)
            
            # Convert back to internal format
            self.marker_map = {}
            for marker_id, data in loaded_map.items():
                pose = PoseStamped()
                pose.header.frame_id = self.map_frame
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = data['position']['x']
                pose.pose.position.y = data['position']['y']
                pose.pose.position.z = data['position']['z']
                pose.pose.orientation.x = data['orientation']['x']
                pose.pose.orientation.y = data['orientation']['y']
                pose.pose.orientation.z = data['orientation']['z']
                pose.pose.orientation.w = data['orientation']['w']
                
                self.marker_map[marker_id] = {
                    'pose': pose,
                    'last_update': data['last_update'],
                    'confidence': data.get('confidence', 0.5)
                }
            
            response.success = True
            response.message = f'Loaded {len(self.marker_map)} markers from {self.map_file}'
            self.get_logger().info(response.message)
            
        except Exception as e:
            response.success = False
            response.message = f'Failed to load map: {str(e)}'
            self.get_logger().error(response.message)
        
        return response
    
    def load_map(self):
        """Load map on startup"""
        try:
            self.load_map_callback(None, type('Response', (), {'success': False, 'message': ''})())
        except:
            pass  # Ignore errors on startup


def main(args=None):
    rclpy.init(args=args)
    node = MarkerMapManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Save map on shutdown
        node.save_map_callback(None, type('Response', (), {'success': False, 'message': ''})())
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()