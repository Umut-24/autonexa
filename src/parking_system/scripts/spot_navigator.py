#!/usr/bin/env python3
"""
Spot Navigator - Navigate to predefined parking spots
Loads spots from YAML file and provides navigation to selected spots

Features:
- Load parking spots from YAML file
- Interactive spot selection via terminal or topic
- Path planning using Nav2
- Visualization of spots in RViz
- Waypoint-based path following (uses predefined roads)

Usage:
  ros2 run parking_system spot_navigator.py
  
  Then publish to /navigate_to_spot:
    ros2 topic pub --once /navigate_to_spot std_msgs/String "data: 'spot_1'"
  
  Or use interactive mode:
    ros2 run parking_system spot_navigator.py --interactive
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from nav_msgs.msg import Path
import yaml
import os
import math
import sys
import threading


class SpotNavigator(Node):
    def __init__(self, spots_file=None, interactive=False):
        super().__init__('spot_navigator')
        
        # Configuration
        self.spots_file = spots_file or '/home/autonexa/intelligent_parking_ws/maps/parking_spots.yaml'
        
        # Load spots
        self.spots_data = self.load_spots()
        self.parking_spots = self.spots_data.get('parking_spots', {})
        self.waypoints = self.spots_data.get('waypoints', {})
        self.roads = self.spots_data.get('roads', [])
        
        # Action clients for Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_through_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        
        # Subscriber for navigation commands
        self.nav_sub = self.create_subscription(
            String,
            '/navigate_to_spot',
            self.navigate_cmd_callback,
            10
        )
        
        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.status_pub = self.create_publisher(String, '/navigation_status', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/parking_spots_markers', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        
        # Publish markers periodically
        self.marker_timer = self.create_timer(1.0, self.publish_markers)
        
        # State
        self.current_goal = None
        self.is_navigating = False
        
        self.get_logger().info(f'Spot Navigator initialized')
        self.get_logger().info(f'Loaded {len(self.parking_spots)} parking spots')
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
        self.get_logger().info(f'Loaded {len(self.roads)} road connections')
        
        if interactive:
            self.start_interactive_mode()
    
    def load_spots(self):
        """Load spots from YAML file"""
        if os.path.exists(self.spots_file):
            try:
                with open(self.spots_file, 'r') as f:
                    return yaml.safe_load(f) or {}
            except Exception as e:
                self.get_logger().error(f'Failed to load spots: {e}')
        return {}
    
    def reload_spots(self):
        """Reload spots from file"""
        self.spots_data = self.load_spots()
        self.parking_spots = self.spots_data.get('parking_spots', {})
        self.waypoints = self.spots_data.get('waypoints', {})
        self.roads = self.spots_data.get('roads', [])
        self.get_logger().info(f'Reloaded: {len(self.parking_spots)} spots, {len(self.waypoints)} waypoints')
    
    def yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion"""
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return (0.0, 0.0, qz, qw)
    
    def create_pose_stamped(self, x, y, yaw=0.0):
        """Create a PoseStamped message"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        
        q = self.yaw_to_quaternion(yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        
        return pose
    
    def navigate_to_spot(self, spot_id):
        """Navigate to a parking spot"""
        if spot_id not in self.parking_spots:
            msg = f"Unknown spot: {spot_id}. Available: {list(self.parking_spots.keys())}"
            self.get_logger().warn(msg)
            return False, msg
        
        spot = self.parking_spots[spot_id]
        goal_pose = self.create_pose_stamped(spot['x'], spot['y'], spot.get('yaw', 0.0))
        
        # Publish goal for RViz visualization
        self.goal_pub.publish(goal_pose)
        
        # Check if Nav2 action server is available
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            msg = "Nav2 action server not available"
            self.get_logger().error(msg)
            return False, msg
        
        # Send navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        self.get_logger().info(f"Navigating to {spot_id} at ({spot['x']:.2f}, {spot['y']:.2f})")
        
        self.is_navigating = True
        self.current_goal = spot_id
        
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        status = String()
        status.data = f"NAVIGATING: {spot_id}"
        self.status_pub.publish(status)
        
        return True, f"Navigating to {spot_id}"
    
    def navigate_through_waypoints(self, waypoint_ids):
        """Navigate through a series of waypoints"""
        poses = []
        for wp_id in waypoint_ids:
            # Check parking spots first, then waypoints
            if wp_id in self.parking_spots:
                wp = self.parking_spots[wp_id]
            elif wp_id in self.waypoints:
                wp = self.waypoints[wp_id]
            else:
                self.get_logger().warn(f"Unknown waypoint: {wp_id}")
                continue
            
            pose = self.create_pose_stamped(wp['x'], wp['y'], wp.get('yaw', 0.0))
            poses.append(pose)
        
        if not poses:
            return False, "No valid waypoints"
        
        if not self.nav_through_client.wait_for_server(timeout_sec=5.0):
            return False, "Nav2 waypoint follower not available"
        
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses
        
        self.get_logger().info(f"Navigating through {len(poses)} waypoints")
        
        self.is_navigating = True
        
        send_goal_future = self.nav_through_client.send_goal_async(
            goal_msg,
            feedback_callback=self.waypoint_feedback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        return True, f"Navigating through {len(poses)} waypoints"
    
    def navigation_feedback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Distance remaining: {feedback.distance_remaining:.2f}m")
    
    def waypoint_feedback(self, feedback_msg):
        """Handle waypoint navigation feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Waypoint {feedback.current_waypoint} of {feedback.number_of_poses}")
    
    def goal_response_callback(self, future):
        """Handle goal acceptance response"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            self.is_navigating = False
            status = String()
            status.data = "REJECTED"
            self.status_pub.publish(status)
            return
        
        self.get_logger().info('Goal accepted')
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)
    
    def navigation_result_callback(self, future):
        """Handle navigation result"""
        self.is_navigating = False
        result = future.result()
        
        status = String()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info(f'Navigation to {self.current_goal} succeeded!')
            status.data = f"ARRIVED: {self.current_goal}"
        else:
            self.get_logger().warn(f'Navigation failed with status: {result.status}')
            status.data = f"FAILED: {self.current_goal}"
        
        self.status_pub.publish(status)
        self.current_goal = None
    
    def cancel_navigation(self):
        """Cancel current navigation"""
        if self.is_navigating:
            # Would need to track the goal handle to cancel
            self.get_logger().info("Cancellation requested (implement goal handle tracking)")
            return True, "Cancellation requested"
        return False, "No active navigation"
    
    def navigate_cmd_callback(self, msg):
        """Handle navigation commands from topic"""
        cmd = msg.data.strip()
        
        if cmd.startswith('through:'):
            # Navigate through waypoints: "through:wp1,wp2,wp3"
            waypoints = cmd[8:].split(',')
            success, message = self.navigate_through_waypoints(waypoints)
        elif cmd == 'cancel':
            success, message = self.cancel_navigation()
        elif cmd == 'reload':
            self.reload_spots()
            success, message = True, "Spots reloaded"
        else:
            # Navigate to single spot
            success, message = self.navigate_to_spot(cmd)
        
        self.get_logger().info(message)
    
    def publish_markers(self):
        """Publish parking spot markers for RViz visualization"""
        marker_array = MarkerArray()
        
        # Parking spot markers (green cubes)
        for i, (spot_id, spot) in enumerate(self.parking_spots.items()):
            # Spot marker
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'parking_spots'
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = spot['x']
            marker.pose.position.y = spot['y']
            marker.pose.position.z = 0.05
            
            q = self.yaw_to_quaternion(spot.get('yaw', 0.0))
            marker.pose.orientation.x = q[0]
            marker.pose.orientation.y = q[1]
            marker.pose.orientation.z = q[2]
            marker.pose.orientation.w = q[3]
            
            marker.scale.x = 0.15
            marker.scale.y = 0.08
            marker.scale.z = 0.02
            
            # Green for spots, yellow if current goal
            if spot_id == self.current_goal:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                marker.color.r = 0.0
                marker.color.g = 0.8
                marker.color.b = 0.0
            marker.color.a = 0.8
            
            marker_array.markers.append(marker)
            
            # Text label
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'parking_labels'
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = spot['x']
            text_marker.pose.position.y = spot['y']
            text_marker.pose.position.z = 0.15
            text_marker.scale.z = 0.08
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = spot_id
            
            marker_array.markers.append(text_marker)
        
        # Waypoint markers (blue spheres)
        for i, (wp_id, wp) in enumerate(self.waypoints.items()):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'waypoints'
            marker.id = i + 2000
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = wp['x']
            marker.pose.position.y = wp['y']
            marker.pose.position.z = 0.05
            marker.scale.x = 0.08
            marker.scale.y = 0.08
            marker.scale.z = 0.08
            marker.color.r = 0.0
            marker.color.g = 0.5
            marker.color.b = 1.0
            marker.color.a = 0.8
            
            marker_array.markers.append(marker)
        
        # Road connections (blue lines)
        for i, road in enumerate(self.roads):
            from_id = road['from']
            to_id = road['to']
            
            # Get positions
            from_pos = self.get_position(from_id)
            to_pos = self.get_position(to_id)
            
            if from_pos and to_pos:
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = 'roads'
                marker.id = i + 3000
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.scale.x = 0.02  # Line width
                marker.color.r = 0.3
                marker.color.g = 0.7
                marker.color.b = 1.0
                marker.color.a = 0.6
                
                from geometry_msgs.msg import Point
                p1 = Point()
                p1.x = from_pos['x']
                p1.y = from_pos['y']
                p1.z = 0.01
                
                p2 = Point()
                p2.x = to_pos['x']
                p2.y = to_pos['y']
                p2.z = 0.01
                
                marker.points = [p1, p2]
                marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)
    
    def get_position(self, point_id):
        """Get position of a spot or waypoint by ID"""
        if point_id in self.parking_spots:
            return self.parking_spots[point_id]
        if point_id in self.waypoints:
            return self.waypoints[point_id]
        return None
    
    def list_spots(self):
        """List all available spots"""
        result = "\n=== Parking Spots ===\n"
        for spot_id, spot in self.parking_spots.items():
            result += f"  {spot_id}: ({spot['x']:.2f}, {spot['y']:.2f})"
            if spot.get('description'):
                result += f" - {spot['description']}"
            result += "\n"
        
        result += "\n=== Waypoints ===\n"
        for wp_id, wp in self.waypoints.items():
            result += f"  {wp_id}: ({wp['x']:.2f}, {wp['y']:.2f})"
            if wp.get('description'):
                result += f" - {wp['description']}"
            result += "\n"
        
        return result
    
    def start_interactive_mode(self):
        """Start interactive terminal mode"""
        def interactive_loop():
            print("\n" + "="*50)
            print("SPOT NAVIGATOR - Interactive Mode")
            print("="*50)
            print("Commands:")
            print("  go <spot_id>  - Navigate to a parking spot")
            print("  through <wp1,wp2,...> - Navigate through waypoints")
            print("  list          - List all spots and waypoints")
            print("  reload        - Reload spots from file")
            print("  cancel        - Cancel current navigation")
            print("  q             - Quit")
            print("="*50 + "\n")
            
            while rclpy.ok():
                try:
                    cmd = input(">>> ").strip()
                    if not cmd:
                        continue
                    
                    parts = cmd.split()
                    action = parts[0].lower()
                    
                    if action == 'q' or action == 'quit':
                        print("Exiting...")
                        rclpy.shutdown()
                        break
                    
                    elif action == 'go' or action == 'navigate':
                        if len(parts) < 2:
                            print("Usage: go <spot_id>")
                            continue
                        success, msg = self.navigate_to_spot(parts[1])
                        print(msg)
                    
                    elif action == 'through':
                        if len(parts) < 2:
                            print("Usage: through <wp1,wp2,...>")
                            continue
                        waypoints = parts[1].split(',')
                        success, msg = self.navigate_through_waypoints(waypoints)
                        print(msg)
                    
                    elif action == 'list' or action == 'l':
                        print(self.list_spots())
                    
                    elif action == 'reload':
                        self.reload_spots()
                        print("Spots reloaded")
                    
                    elif action == 'cancel':
                        success, msg = self.cancel_navigation()
                        print(msg)
                    
                    else:
                        # Try to navigate directly
                        if action in self.parking_spots:
                            success, msg = self.navigate_to_spot(action)
                            print(msg)
                        else:
                            print(f"Unknown command: {action}")
                
                except EOFError:
                    break
                except Exception as e:
                    print(f"Error: {e}")
        
        thread = threading.Thread(target=interactive_loop, daemon=True)
        thread.start()


def main(args=None):
    rclpy.init(args=args)
    
    interactive = '--interactive' in sys.argv or '-i' in sys.argv
    
    node = SpotNavigator(interactive=interactive)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

