#!/usr/bin/env python3
"""
Parking Coordinator Node
Main coordinator that manages the parking system workflow
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


class ParkingCoordinator(Node):
    def __init__(self):
        super().__init__('parking_coordinator')
        
        # Action client for navigation
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Status subscriber
        self.status_sub = self.create_subscription(
            String,
            '/parking_status',
            self.status_callback,
            10
        )
        
        self.get_logger().info('Parking Coordinator initialized')
        
    def status_callback(self, msg):
        """Handle status updates"""
        self.get_logger().info(f'Status: {msg.data}')
    
    def navigate_to_slot(self, slot_pose):
        """Send navigation goal to Nav2"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = slot_pose
        
        self.nav_to_pose_client.wait_for_server()
        
        self.get_logger().info('Sending navigation goal...')
        self.send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        self.get_logger().info(f'Navigation completed with status: {result}')
    
    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Distance remaining: {feedback.distance_remaining:.2f}m')


def main(args=None):
    rclpy.init(args=args)
    node = ParkingCoordinator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

