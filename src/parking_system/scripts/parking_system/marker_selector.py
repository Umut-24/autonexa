#!/usr/bin/env python3
"""
ArUco Marker ID Selector Node
Allows selection of target ArUco marker IDs via topics or services
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from std_srvs.srv import SetBool


class MarkerSelector(Node):
    def __init__(self):
        super().__init__('marker_selector')
        
        # Publishers
        self.target_id_pub = self.create_publisher(Int32, '/target_marker_id', 10)
        self.status_pub = self.create_publisher(String, '/selector_status', 10)
        
        # Subscribers
        self.id_command_sub = self.create_subscription(
            Int32,
            '/select_marker_id',
            self.id_command_callback,
            10
        )
        
        # Services
        self.set_marker_srv = self.create_service(
            SetBool,
            '/set_marker_active',
            self.set_marker_callback
        )
        
        # Current state
        self.current_target_id = 0
        self.marker_active = False
        
        # Available markers (should match your physical setup)
        self.available_markers = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
        
        self.get_logger().info('Marker Selector initialized')
        self.status_pub.publish(String(data='READY: Marker selector active'))
    
    def id_command_callback(self, msg):
        """Handle direct ID selection"""
        marker_id = msg.data
        
        if marker_id not in self.available_markers:
            self.get_logger().warn(f'Marker ID {marker_id} not in available list')
            self.status_pub.publish(String(data=f'ERROR: Marker {marker_id} not available'))
            return
        
        self.current_target_id = marker_id
        self.marker_active = True
        
        # Publish the selection
        id_msg = Int32()
        id_msg.data = marker_id
        self.target_id_pub.publish(id_msg)
        
        self.get_logger().info(f'Selected marker ID: {marker_id}')
        self.status_pub.publish(String(data=f'SELECTED: Marker {marker_id} active'))
    
    def set_marker_callback(self, request, response):
        """Handle marker activation/deactivation"""
        if request.data:
            # Activate current target
            if self.current_target_id is not None:
                id_msg = Int32()
                id_msg.data = self.current_target_id
                self.target_id_pub.publish(id_msg)
                self.marker_active = True
                response.success = True
                response.message = f'Activated marker {self.current_target_id}'
                self.status_pub.publish(String(data=f'ACTIVATED: Marker {self.current_target_id}'))
            else:
                response.success = False
                response.message = 'No marker selected'
        else:
            # Deactivate
            self.marker_active = False
            response.success = True
            response.message = 'Deactivated marker tracking'
            self.status_pub.publish(String(data='DEACTIVATED: Marker tracking off'))
        
        return response
    
    def list_available_markers(self):
        """Log available markers"""
        self.get_logger().info(f'Available markers: {self.available_markers}')


def main(args=None):
    rclpy.init(args=args)
    node = MarkerSelector()
    
    # List available markers on startup
    node.list_available_markers()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()</content>
<parameter name="filePath">c:\Users\Anıl\OneDrive\Belgeler\GitHub\autonexa\src\parking_system\scripts\parking_system\marker_selector.py