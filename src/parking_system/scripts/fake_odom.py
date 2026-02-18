#!/usr/bin/env python3
"""Publish fake odom for testing Nav2"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class FakeOdom(Node):
    def __init__(self):
        super().__init__('fake_odom')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.05, self.publish)
        self.get_logger().info('Fake odom publisher started')
    
    def publish(self):
        now = self.get_clock().now().to_msg()
        
        # Publish odom message
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)
        
        # Publish odom->base_link transform
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = FakeOdom()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
