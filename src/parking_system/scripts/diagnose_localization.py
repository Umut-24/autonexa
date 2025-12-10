#!/usr/bin/env python3
"""
Diagnostic script to check AMCL localization quality
Monitors particle cloud and pose estimates
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import ParticleCloud
import numpy as np
import math


class LocalizationDiagnostic(Node):
    def __init__(self):
        super().__init__('localization_diagnostic')
        
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        
        self.particle_sub = self.create_subscription(
            ParticleCloud,
            '/particlecloud',
            self.particle_callback,
            10
        )
        
        self.last_pose = None
        self.particle_count = 0
        self.pose_variance_history = []
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('AMCL Localization Diagnostic Started')
        self.get_logger().info('Monitoring /amcl_pose and /particlecloud')
        self.get_logger().info('=' * 60)
        
        # Print statistics every 5 seconds
        self.print_timer = self.create_timer(5.0, self.print_statistics)
    
    def pose_callback(self, msg):
        """Process pose estimate"""
        pose = msg.pose.pose
        cov = msg.pose.covariance
        
        # Extract position and orientation
        x = pose.position.x
        y = pose.position.y
        z = pose.orientation.z
        w = pose.orientation.w
        yaw = 2 * math.atan2(z, w)
        
        # Extract covariance (diagonal elements)
        # [xx, xy, xz, xroll, xpitch, xyaw,
        #  yx, yy, yz, yroll, ypitch, yyaw,
        #  ...]
        cov_xx = cov[0]
        cov_yy = cov[7]
        cov_aa = cov[35]  # yaw covariance
        
        # Calculate position uncertainty (standard deviation)
        std_x = math.sqrt(cov_xx)
        std_y = math.sqrt(cov_yy)
        std_a = math.sqrt(cov_aa)
        
        self.last_pose = {
            'x': x,
            'y': y,
            'yaw': yaw,
            'std_x': std_x,
            'std_y': std_y,
            'std_a': std_a,
            'cov_xx': cov_xx,
            'cov_yy': cov_yy,
            'cov_aa': cov_aa,
        }
        
        self.pose_variance_history.append({
            'std_x': std_x,
            'std_y': std_y,
            'std_a': std_a,
        })
        
        # Keep only last 20 poses
        if len(self.pose_variance_history) > 20:
            self.pose_variance_history.pop(0)
    
    def particle_callback(self, msg):
        """Process particle cloud"""
        self.particle_count = len(msg.particles)
    
    def print_statistics(self):
        """Print localization statistics"""
        if self.last_pose is None:
            self.get_logger().warn('No pose data received yet...')
            return
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('AMCL LOCALIZATION STATISTICS')
        self.get_logger().info('=' * 60)
        
        self.get_logger().info(f'Particle Count: {self.particle_count}')
        self.get_logger().info(f'Estimated Position: ({self.last_pose["x"]:.3f}, {self.last_pose["y"]:.3f})')
        self.get_logger().info(f'Estimated Yaw: {math.degrees(self.last_pose["yaw"]):.1f}°')
        self.get_logger().info(f'Position Uncertainty (std): X={self.last_pose["std_x"]:.3f}m, Y={self.last_pose["std_y"]:.3f}m')
        self.get_logger().info(f'Yaw Uncertainty (std): {math.degrees(self.last_pose["std_a"]):.1f}°')
        
        # Calculate average uncertainty
        if len(self.pose_variance_history) > 0:
            avg_std_x = np.mean([p['std_x'] for p in self.pose_variance_history])
            avg_std_y = np.mean([p['std_y'] for p in self.pose_variance_history])
            avg_std_a = np.mean([p['std_a'] for p in self.pose_variance_history])
            
            self.get_logger().info(f'Average Uncertainty (last 20): X={avg_std_x:.3f}m, Y={avg_std_y:.3f}m, Yaw={math.degrees(avg_std_a):.1f}°')
        
        # Diagnostic warnings
        if self.particle_count < 1000:
            self.get_logger().warn('⚠️  LOW PARTICLE COUNT: Consider increasing max_particles')
        
        if self.last_pose["std_x"] > 0.1 or self.last_pose["std_y"] > 0.1:
            self.get_logger().warn('⚠️  HIGH POSITION UNCERTAINTY: Localization confidence is low')
            self.get_logger().warn('   - Check if initial pose is set correctly')
            self.get_logger().warn('   - Verify map matches the environment')
            self.get_logger().warn('   - Check scan quality')
        
        if math.degrees(self.last_pose["std_a"]) > 10:
            self.get_logger().warn('⚠️  HIGH YAW UNCERTAINTY: Orientation confidence is low')
        
        if self.last_pose["std_x"] < 0.05 and self.last_pose["std_y"] < 0.05 and math.degrees(self.last_pose["std_a"]) < 5:
            self.get_logger().info('✅ GOOD LOCALIZATION: Low uncertainty, high confidence')
        
        self.get_logger().info('=' * 60)


def main():
    rclpy.init()
    node = LocalizationDiagnostic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

