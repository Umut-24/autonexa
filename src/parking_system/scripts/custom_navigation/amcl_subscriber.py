#!/usr/bin/env python3
"""
AMCL Pose Subscriber
Subscribes to AMCL pose and provides current robot pose
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from typing import Optional, Callable
import math


class AMCLSubscriber(Node):
    """Subscribes to AMCL pose updates"""
    
    def __init__(self, callback: Optional[Callable] = None):
        """
        Initialize AMCL subscriber
        
        Args:
            callback: Optional callback function(pose) called on each update
                     pose is (x, y, theta) in meters and radians
        """
        super().__init__('amcl_subscriber')
        self.callback = callback
        self.current_pose = None  # (x, y, theta)
        
        # Subscribe to AMCL pose (standard Nav2 topic)
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
    
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Handle AMCL pose update"""
        pose = msg.pose.pose
        
        # Extract position
        x = pose.position.x
        y = pose.position.y
        
        # Extract orientation (quaternion to yaw)
        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w
        
        # Convert quaternion to yaw (theta)
        # yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2))
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        theta = math.atan2(siny_cosp, cosy_cosp)
        
        self.current_pose = (x, y, theta)
        
        # Call user callback if provided
        if self.callback:
            self.callback(x, y, theta)
    
    def get_pose(self) -> Optional[tuple]:
        """
        Get current robot pose
        
        Returns:
            (x, y, theta) tuple or None if not received yet
        """
        return self.current_pose
    
    def wait_for_pose(self, timeout: float = 5.0) -> Optional[tuple]:
        """
        Wait for first pose update
        
        Args:
            timeout: Maximum time to wait (seconds)
        
        Returns:
            (x, y, theta) or None if timeout
        """
        import time
        start_time = time.time()
        while self.current_pose is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.current_pose

