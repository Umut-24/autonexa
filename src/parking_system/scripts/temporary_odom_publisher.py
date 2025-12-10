#!/usr/bin/env python3
"""
Temporary dynamic transform publisher for map->odom and odom->base_link
This publishes on /tf (dynamic) so SLAM can override it once it starts.
This node monitors if SLAM is publishing and reduces its rate when SLAM is active.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import time


class TemporaryOdomPublisher(Node):
    def __init__(self):
        super().__init__('temporary_odom_publisher')
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.slam_active = False
        self.last_check_time = self.get_clock().now()
        self.check_interval = 2.0  # Check every 2 seconds if SLAM is publishing
        
        # Publish at high frequency (100Hz) to avoid timestamp synchronization issues
        self.timer = self.create_timer(0.01, self.publish_transform)  # 100Hz
        self.check_timer = self.create_timer(self.check_interval, self.check_slam_status)
        
        self.get_logger().info('Temporary transform publisher started')
        self.get_logger().info('Publishing map->odom and odom->base_link (identity)')
        self.get_logger().info('Will reduce rate when SLAM starts publishing')
    
    def check_slam_status(self):
        """Check if SLAM is publishing transforms by looking for non-identity transforms"""
        try:
            # Try to get odom->base_link transform
            # If SLAM is publishing and robot has moved, transform won't be identity
            now = self.get_clock().now()
            transform = self.tf_buffer.lookup_transform(
                'odom', 'base_link', now, timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Check if transform is non-identity (SLAM is tracking movement)
            trans = transform.transform.translation
            rot = transform.transform.rotation
            
            is_identity = (abs(trans.x) < 0.001 and abs(trans.y) < 0.001 and 
                          abs(trans.z) < 0.001 and
                          abs(rot.x) < 0.001 and abs(rot.y) < 0.001 and
                          abs(rot.z) < 0.001 and abs(rot.w - 1.0) < 0.001)
            
            if not is_identity and not self.slam_active:
                self.slam_active = True
                self.get_logger().info('SLAM detected! Transform is non-identity. Reducing publish rate.')
                # Reduce publish rate but don't stop completely (in case SLAM stops)
                self.timer.cancel()
                self.timer = self.create_timer(1.0, self.publish_transform)  # 1Hz fallback
        except Exception as e:
            # Transform not available or lookup failed - SLAM might not be active yet
            pass
    
    def publish_transform(self):
        now = self.get_clock().now()
        
        # Publish map->odom (identity, since we don't have wheel odometry)
        # SLAM will update this dynamically once it starts
        t1 = TransformStamped()
        t1.header.stamp = now.to_msg()
        t1.header.frame_id = 'map'
        t1.child_frame_id = 'odom'
        t1.transform.translation.x = 0.0
        t1.transform.translation.y = 0.0
        t1.transform.translation.z = 0.0
        t1.transform.rotation.x = 0.0
        t1.transform.rotation.y = 0.0
        t1.transform.rotation.z = 0.0
        t1.transform.rotation.w = 1.0
        
        # Publish odom->base_link (identity, will be overridden by SLAM)
        t2 = TransformStamped()
        t2.header.stamp = now.to_msg()
        t2.header.frame_id = 'odom'
        t2.child_frame_id = 'base_link'
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.0
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0
        
        # Publish both transforms separately to ensure they go to /tf (not /tf_static)
        # Using sendTransform with a list should work, but let's be explicit
        self.tf_broadcaster.sendTransform(t1)
        self.tf_broadcaster.sendTransform(t2)


def main(args=None):
    rclpy.init(args=args)
    node = TemporaryOdomPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

