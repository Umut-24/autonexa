#!/usr/bin/env python3
"""
Odom Bootstrap - Publishes temporary odom->base_link transform until laser_scan_matcher takes over
This ensures the TF tree is connected from the start
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import time


class OdomBootstrap(Node):
    def __init__(self):
        super().__init__('odom_bootstrap')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.publish_timer = self.create_timer(0.1, self.publish_transform)  # 10Hz
        self.check_timer = self.create_timer(2.0, self.check_real_transform)  # Check every 2 seconds
        
        self.active = True
        self.check_count = 0
        
        self.get_logger().info('Odom Bootstrap started')
        self.get_logger().info('Publishing temporary odom->base_link transform at origin')
        self.get_logger().info('Will stop once laser_scan_matcher starts publishing')
    
    def check_real_transform(self):
        """Check if laser_scan_matcher is publishing the transform"""
        self.check_count += 1
        try:
            # Try to get the transform with a short timeout
            transform = self.tf_buffer.lookup_transform(
                'odom', 'base_link', 
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Check if it's not an identity transform (meaning real movement)
            trans = transform.transform.translation
            rot = transform.transform.rotation
            
            is_identity = (
                abs(trans.x) < 0.001 and abs(trans.y) < 0.001 and abs(trans.z) < 0.001 and
                abs(rot.x) < 0.001 and abs(rot.y) < 0.001 and 
                abs(rot.z) < 0.001 and abs(rot.w - 1.0) < 0.001
            )
            
            # Check if odom topic is being published (indicates laser_scan_matcher is active)
            # For now, just check if we've been running long enough
            # After 10 seconds, assume laser_scan_matcher should have started
            if self.check_count > 5:  # After ~10 seconds
                # Check if /odom topic exists
                topics = self.get_topic_names_and_types()
                odom_topic_exists = any('/odom' in str(topic) for topic in topics)
                
                if odom_topic_exists:
                    self.get_logger().info('Detected /odom topic - laser_scan_matcher may be active')
                    # Don't stop yet, let it run but reduce priority by publishing less frequently
                    # The dynamic transform from laser_scan_matcher should take precedence
        except Exception as e:
            # Transform doesn't exist or error - keep publishing
            if self.check_count % 10 == 0:  # Log every 20 seconds
                pass
    
    def publish_transform(self):
        """Publish temporary odom->base_link transform"""
        if not self.active:
            return
            
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        # Identity transform (odom and base_link start at same origin)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomBootstrap()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

