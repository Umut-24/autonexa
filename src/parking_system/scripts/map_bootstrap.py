#!/usr/bin/env python3
"""
Map Bootstrap - Publishes DYNAMIC map->odom transform until AMCL/SLAM takes over
This ensures the TF tree is connected from the start.
Uses DYNAMIC transforms (on /tf) so AMCL/SLAM can override them.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped


class MapBootstrap(Node):
    def __init__(self):
        super().__init__('map_bootstrap')
        
        # Use TransformBroadcaster which publishes DYNAMIC transforms on /tf
        self.tf_broadcaster = TransformBroadcaster(self)
        self.publish_timer = self.create_timer(0.05, self.publish_transform)  # 20Hz
        
        # Track if AMCL is active by subscribing to its pose output
        self.amcl_active = False
        self.amcl_pose_count = 0
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            qos
        )
        
        self.get_logger().info('Map Bootstrap started - Publishing DYNAMIC map->odom transform')
        self.get_logger().info('Will stop when AMCL becomes active')
    
    def amcl_pose_callback(self, msg):
        """AMCL is publishing poses - it's active"""
        self.amcl_pose_count += 1
        if self.amcl_pose_count >= 3 and not self.amcl_active:
            self.amcl_active = True
            self.get_logger().info('AMCL is now active - Bootstrap stopping')
    
    def publish_transform(self):
        """Publish temporary map->odom transform until AMCL takes over"""
        if self.amcl_active:
            return  # AMCL is publishing, stop bootstrap
        
        # Publish DYNAMIC bootstrap transform (on /tf, not /tf_static)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        
        # Identity transform (map and odom start at same origin)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        # This publishes on /tf (DYNAMIC)
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = MapBootstrap()
    
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
            pass


if __name__ == '__main__':
    main()
