#!/usr/bin/env python3
"""
ArUco Navigation Node
Sets navigation goals based on selected ArUco marker IDs
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, String
from nav2_msgs.action import NavigateToPose
import time


class ArucoNavigation(Node):
    def __init__(self):
        super().__init__('aruco_navigation')
        
        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Subscribers
        self.target_id_sub = self.create_subscription(
            Int32,
            '/target_marker_id',
            self.target_id_callback,
            10
        )
        
        self.marker_map_sub = self.create_subscription(
            PoseStamped,
            '/marker_map/poses',
            self.marker_pose_callback,
            10
        )
        
        self.fused_pose_sub = self.create_subscription(
            PoseStamped,
            '/fused_pose',
            self.fused_pose_callback,
            10
        )
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/navigation_status', 10)
        
        # Marker positions (will be updated from marker map)
        self.marker_positions = {}  # {id: {'x': float, 'y': float, 'yaw': float}}
        
        self.current_goal = None
        self.target_id = None
        
        self.get_logger().info('ArUco Navigation initialized')
    
    def target_id_callback(self, msg):
        """Handle target marker ID selection"""
        marker_id = msg.data
        
        if marker_id not in self.marker_positions:
            self.get_logger().warn(f'Marker ID {marker_id} not found in known positions')
            self.status_pub.publish(String(data=f'ERROR: Marker {marker_id} position unknown'))
            return
        
        self.target_id = marker_id
        self.navigate_to_marker(marker_id)
    
    def fused_pose_callback(self, msg):
        """Monitor current pose for status updates"""
        # Could add logic to check if we've reached the goal
        pass
    
    def marker_pose_callback(self, msg):
        """Update marker position from individual pose message"""
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
        
        # Extract yaw from quaternion
        import tf_transformations
        quaternion = [msg.pose.orientation.x, msg.pose.orientation.y, 
                     msg.pose.orientation.z, msg.pose.orientation.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(quaternion)
        
        self.marker_positions[marker_id] = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'yaw': yaw
        }
        
        self.get_logger().debug(f'Updated position for marker {marker_id}: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
    
    def navigate_to_marker(self, marker_id):
        """Send navigation goal to the specified marker"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            self.status_pub.publish(String(data='ERROR: Navigation server unavailable'))
            return
        
        # Get marker position
        pos = self.marker_positions[marker_id]
        
        # Create goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = pos['x']
        goal_pose.pose.position.y = pos['y']
        goal_pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        import tf_transformations
        quaternion = tf_transformations.quaternion_from_euler(0, 0, pos['yaw'])
        goal_pose.pose.orientation.x = quaternion[0]
        goal_pose.pose.orientation.y = quaternion[1]
        goal_pose.pose.orientation.z = quaternion[2]
        goal_pose.pose.orientation.w = quaternion[3]
        
        # Send goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        self.get_logger().info(f'Sending navigation goal to marker {marker_id} at ({pos["x"]:.2f}, {pos["y"]:.2f})')
        self.status_pub.publish(String(data=f'NAVIGATING: Moving to marker {marker_id}'))
        
        # Send goal asynchronously
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            self.status_pub.publish(String(data='ERROR: Navigation goal rejected'))
            return
        
        self.get_logger().info('Navigation goal accepted')
        self.status_pub.publish(String(data='NAVIGATING: Goal accepted, moving...'))
        
        # Get result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        """Handle navigation completion"""
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Navigation completed successfully')
            self.status_pub.publish(String(data=f'SUCCESS: Reached marker {self.target_id}'))
        elif status == 5:  # CANCELED
            self.get_logger().info('Navigation canceled')
            self.status_pub.publish(String(data='CANCELED: Navigation stopped'))
        elif status == 6:  # ABORTED
            self.get_logger().error('Navigation aborted')
            self.status_pub.publish(String(data=f'ERROR: Navigation to marker {self.target_id} failed'))
        else:
            self.get_logger().warn(f'Navigation ended with status: {status}')
            self.status_pub.publish(String(data=f'UNKNOWN: Navigation ended with status {status}'))


def main(args=None):
    rclpy.init(args=args)
    node = ArucoNavigation()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
