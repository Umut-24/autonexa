#!/usr/bin/env python3
"""
Print Robot Position - Continuously outputs robot's estimated position
Also triggers global localization so robot figures out where it is
"""

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from std_srvs.srv import Empty
import math
import time


class PositionPrinter(Node):
    def __init__(self):
        super().__init__('position_printer')
        
        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Global localization service client
        self.global_loc_client = self.create_client(Empty, '/reinitialize_global_localization')
        
        # Wait for AMCL to start
        self.get_logger().info('Waiting for AMCL global localization service...')
        
        # Timer to print position
        self.position_timer = self.create_timer(1.0, self.print_position)
        
        # Trigger global localization after a delay
        self.create_timer(3.0, self.trigger_global_localization)
        self.global_loc_triggered = False
        
        self.get_logger().info('Position Printer started')
        self.get_logger().info('Robot will automatically figure out its position...')
    
    def trigger_global_localization(self):
        """Trigger AMCL global localization - spreads particles across entire map"""
        if self.global_loc_triggered:
            return
            
        if self.global_loc_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('🔍 Triggering GLOBAL LOCALIZATION...')
            self.get_logger().info('   Robot is searching the entire map to find itself...')
            
            request = Empty.Request()
            future = self.global_loc_client.call_async(request)
            future.add_done_callback(self.global_loc_callback)
            self.global_loc_triggered = True
        else:
            self.get_logger().warn('Global localization service not available yet...')
    
    def global_loc_callback(self, future):
        try:
            future.result()
            self.get_logger().info('✅ Global localization triggered!')
            self.get_logger().info('   Move the robot slightly to help it converge...')
        except Exception as e:
            self.get_logger().error(f'Global localization failed: {e}')
    
    def print_position(self):
        """Print current estimated robot position"""
        try:
            # Get transform from map to base_link
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            # Extract position
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
            # Extract rotation (yaw/theta)
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            
            # Convert quaternion to yaw (theta)
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            theta = math.atan2(siny_cosp, cosy_cosp)
            theta_deg = math.degrees(theta)
            
            # Print position
            print(f'\r📍 Robot Position: X={x:+.3f}m, Y={y:+.3f}m, θ={theta_deg:+.1f}°', end='', flush=True)
            
        except Exception as e:
            print(f'\r⏳ Waiting for localization... (TF not ready)', end='', flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = PositionPrinter()
    
    print('\n' + '='*60)
    print('ROBOT POSITION TRACKER')
    print('='*60)
    print('The robot will automatically search the map to find itself.')
    print('Move the robot slightly to help it converge.')
    print('='*60 + '\n')
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n\nStopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

