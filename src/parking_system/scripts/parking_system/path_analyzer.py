#!/usr/bin/env python3
"""
Path Analyzer Node
Analyzes the planned navigation path and generates discrete movement commands
like "go straight X cm" and "turn Y degrees"
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import math
import numpy as np


class PathAnalyzer(Node):
    def __init__(self):
        super().__init__('path_analyzer')
        
        # Subscriber to the planned path
        self.path_sub = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10
        )
        
        # Publisher for movement commands
        self.commands_pub = self.create_publisher(String, '/movement_commands', 10)
        
        # Publisher for visualization markers
        self.marker_pub = self.create_publisher(MarkerArray, '/movement_markers', 10)
        
        self.marker_id = 0
        
        # Parameters
        self.declare_parameter('straight_tolerance', 0.1)  # radians, tolerance for considering segments straight
        self.declare_parameter('min_straight_distance', 0.05)  # meters, minimum distance for a straight segment
        self.declare_parameter('min_turn_angle', 0.05)  # radians, minimum angle for a turn
        
        self.get_logger().info('Path Analyzer initialized')
    
    def path_callback(self, msg):
        """Analyze the received path and generate movement commands"""
        if not msg.poses or len(msg.poses) < 2:
            self.get_logger().warn('Received path with insufficient waypoints')
            return
        
        commands = self.analyze_path(msg.poses)
        
        # Publish commands as a single string with newline separators
        commands_str = '\n'.join(commands)
        commands_msg = String()
        commands_msg.data = commands_str
        self.commands_pub.publish(commands_msg)
        
        # Publish visualization markers
        self.publish_command_markers(msg.poses, commands)
        
        self.get_logger().info(f'Generated {len(commands)} movement commands')
        for cmd in commands:
            self.get_logger().info(f'  {cmd}')
    
    def analyze_path(self, poses):
        """Analyze the path poses and generate movement commands"""
        commands = []
        
        straight_tolerance = self.get_parameter('straight_tolerance').value
        min_straight_distance = self.get_parameter('min_straight_distance').value
        min_turn_angle = self.get_parameter('min_turn_angle').value
        
        # Calculate vectors between consecutive points
        vectors = []
        for i in range(len(poses) - 1):
            p1 = poses[i].pose.position
            p2 = poses[i+1].pose.position
            dx = p2.x - p1.x
            dy = p2.y - p1.y
            distance = math.sqrt(dx*dx + dy*dy)
            angle = math.atan2(dy, dx)
            vectors.append((distance, angle))
        
        # Group into straight segments and turns
        i = 0
        while i < len(vectors):
            # Start a straight segment
            start_i = i
            current_angle = vectors[i][1]
            total_distance = vectors[i][0]
            
            # Extend straight segment while angles are similar
            i += 1
            while i < len(vectors):
                angle_diff = abs(self.angle_difference(current_angle, vectors[i][1]))
                if angle_diff > straight_tolerance:
                    break
                total_distance += vectors[i][0]
                i += 1
            
            # Add straight command if distance is significant
            if total_distance >= min_straight_distance:
                commands.append(f"go straight {total_distance*100:.1f} cm")
            
            # If there's a next vector, calculate turn
            if i < len(vectors):
                next_angle = vectors[i][1]
                turn_angle = self.angle_difference(current_angle, next_angle)
                
                if abs(turn_angle) >= min_turn_angle:
                    direction = "left" if turn_angle > 0 else "right"
                    degrees = abs(math.degrees(turn_angle))
                    commands.append(f"turn {degrees:.1f} degrees to {direction}")
        
        return commands
    
    def publish_command_markers(self, poses, commands):
        """Publish markers for visualizing movement commands"""
        marker_array = MarkerArray()
        
        # Clear previous markers
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        if not poses:
            self.marker_pub.publish(marker_array)
            return
        
        # Create text markers for each command
        current_pos = poses[0].pose.position
        cmd_idx = 0
        
        for i, (distance, angle) in enumerate(self.calculate_vectors(poses)):
            if cmd_idx < len(commands):
                # Create text marker at the end of each segment
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "movement_commands"
                marker.id = self.marker_id
                self.marker_id += 1
                marker.type = Marker.TEXT_VIEW_FACING
                marker.action = Marker.ADD
                marker.pose.position.x = current_pos.x + distance * math.cos(angle)
                marker.pose.position.y = current_pos.y + distance * math.sin(angle)
                marker.pose.position.z = 0.5  # Above ground
                marker.pose.orientation.w = 1.0
                marker.scale.z = 0.2  # Text height
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                marker.color.a = 1.0
                marker.text = commands[cmd_idx] if cmd_idx < len(commands) else ""
                marker.lifetime = rclpy.duration.Duration(seconds=30).to_msg()
                
                marker_array.markers.append(marker)
                cmd_idx += 1
            
            # Update position
            current_pos.x += distance * math.cos(angle)
            current_pos.y += distance * math.sin(angle)
        
        self.marker_pub.publish(marker_array)
    
    def calculate_vectors(self, poses):
        """Calculate vectors between consecutive points (helper for markers)"""
        vectors = []
        for i in range(len(poses) - 1):
            p1 = poses[i].pose.position
            p2 = poses[i+1].pose.position
            dx = p2.x - p1.x
            dy = p2.y - p1.y
            distance = math.sqrt(dx*dx + dy*dy)
            angle = math.atan2(dy, dx)
            vectors.append((distance, angle))
        return vectors
    
    def angle_difference(self, angle1, angle2):
        """Calculate the smallest angle difference between two angles"""
        diff = angle2 - angle1
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff


def main(args=None):
    rclpy.init(args=args)
    node = PathAnalyzer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
