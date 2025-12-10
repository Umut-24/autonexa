#!/usr/bin/env python3
"""
Localization Initializer - Sets initial pose for SLAM Toolbox localization
This helps SLAM Toolbox localize the robot in the saved map
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
import time


class LocalizationInitializer(Node):
    def __init__(self):
        super().__init__('localization_initializer')
        
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
        self.get_logger().info('Waiting for SLAM Toolbox service...')
        while not self.slam_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SLAM service not available, waiting...')
        
        # Start checking after a delay to let everything initialize
        self.timer = self.create_timer(2.0, self.check_and_initialize)
        
        self.get_logger().info('Localization Initializer started')
        self.get_logger().info('Will automatically set initial pose once SLAM is active')
        self.get_logger().info('You can also set initial pose manually using "2D Pose Estimate" in RViz')
    
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
                            self.get_logger().info('SLAM Toolbox is active. Setting initial pose...')
                            self.get_logger().info('Note: You may need to set initial pose manually in RViz if robot position is unknown')
                            self.send_initial_pose()
                            self.initial_pose_sent = True
                        elif self.check_count % 10 == 0:  # Log every 20 seconds
                            self.get_logger().info(f'SLAM Toolbox is active (state: {state_label})')
                    else:
                        if self.check_count % 5 == 0:
                            self.get_logger().warn(f'SLAM Toolbox not active yet (state: {state_label}, id: {state_id})')
        except Exception as e:
            if self.check_count % 5 == 0:
                self.get_logger().warn(f'Error checking SLAM state: {str(e)}')
    
    def send_initial_pose(self):
        """Send initial pose at origin (0, 0, 0) facing forward"""
        # Wait a bit to ensure SLAM is fully ready
        time.sleep(1.0)
        
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # Set pose at origin (you can adjust this if you know where the robot is)
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        
        # Set covariance (larger uncertainty to help localization)
        msg.pose.covariance[0] = 1.0  # x (larger uncertainty)
        msg.pose.covariance[7] = 1.0  # y (larger uncertainty)
        msg.pose.covariance[35] = 0.2  # yaw (larger uncertainty)
        
        # Publish multiple times to ensure it's received
        for i in range(3):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.initial_pose_pub.publish(msg)
            self.get_logger().info(f'Published initial pose at origin (attempt {i+1}/3)')
            time.sleep(0.5)
        
        self.get_logger().info('✅ Initial pose sent!')
        self.get_logger().info('   If robot is not at origin, use "2D Pose Estimate" in RViz to set correct position')


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationInitializer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

