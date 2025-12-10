#!/usr/bin/env python3
"""
SLAM Initializer - Ensures SLAM receives initial pose and starts properly
This node monitors SLAM state and automatically sets initial pose if needed
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
import time


class SLAMInitializer(Node):
    def __init__(self):
        super().__init__('slam_initializer')
        
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        self.slam_state_client = self.create_client(
            GetState,
            '/slam_toolbox/get_state'
        )
        
        self.initial_pose_sent = False
        self.check_count = 0
        
        # Wait for services
        self.get_logger().info('Waiting for SLAM service...')
        while not self.slam_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SLAM service not available, waiting...')
        
        # Start checking after a longer delay to let SLAM fully initialize
        # Also send initial pose immediately after SLAM becomes active
        self.timer = self.create_timer(3.0, self.check_and_initialize)
        
        self.get_logger().info('SLAM Initializer started')
        self.get_logger().info('Will automatically set initial pose if SLAM is active but not initialized')
    
    def check_and_initialize(self):
        self.check_count += 1
        
        # Check SLAM state
        try:
            request = GetState.Request()
            future = self.slam_state_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            
            if future.done():
                response = future.result()
                if response is not None:
                    state_id = response.current_state.id
                    state_label = response.current_state.label
                    
                    if state_id == 3 and state_label == 'active':
                        # SLAM is active
                        if not self.initial_pose_sent:
                            self.get_logger().info('SLAM is active. Setting initial pose at origin...')
                            self.send_initial_pose()
                            self.initial_pose_sent = True
                        elif self.check_count % 10 == 0:  # Log every 20 seconds
                            self.get_logger().info(f'SLAM is active (state: {state_label})')
                    else:
                        if self.check_count % 5 == 0:
                            self.get_logger().warn(f'SLAM not active yet (state: {state_label}, id: {state_id})')
        except Exception as e:
            if self.check_count % 5 == 0:
                self.get_logger().warn(f'Error checking SLAM state: {str(e)}')
    
    def send_initial_pose(self):
        """Send initial pose at origin (0, 0, 0) facing forward"""
        # Wait a bit more to ensure SLAM is fully ready
        time.sleep(1.0)
        
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        # Try 'map' frame first, but SLAM might need 'odom' or 'base_link'
        # Actually, for SLAM Toolbox, initial pose should be in 'map' frame
        msg.header.frame_id = 'map'
        
        # Set pose at origin
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        
        # Set covariance (larger uncertainty to help SLAM start)
        msg.pose.covariance[0] = 0.5  # x
        msg.pose.covariance[7] = 0.5  # y
        msg.pose.covariance[35] = 0.1  # yaw
        
        # Publish multiple times with delays to ensure it's received
        self.get_logger().info('Publishing initial pose to /initialpose...')
        for i in range(5):  # More attempts
            msg.header.stamp = self.get_clock().now().to_msg()
            self.initial_pose_pub.publish(msg)
            self.get_logger().info(f'Published initial pose (attempt {i+1}/5)')
            time.sleep(0.3)
        
        self.get_logger().info('✅ Initial pose sent! SLAM should start processing scans now.')
        self.get_logger().info('   If map still doesn\'t appear, try setting 2D Pose Estimate manually in RViz.')


def main(args=None):
    rclpy.init(args=args)
    node = SLAMInitializer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass  # Ignore shutdown errors


if __name__ == '__main__':
    main()

