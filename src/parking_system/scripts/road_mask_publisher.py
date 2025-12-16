#!/usr/bin/env python3
"""
Road Mask Publisher - Publishes the road mask as a nav_msgs/OccupancyGrid
This allows the costmap to use the road mask as a keepout filter

The road mask defines where navigation is ALLOWED:
- Areas you drew (roads) = FREE (0)
- Areas not drawn = OCCUPIED (100) - blocked for navigation
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
import yaml
import cv2
import numpy as np
import os


class RoadMaskPublisher(Node):
    def __init__(self):
        super().__init__('road_mask_publisher')
        
        # Declare parameters
        self.declare_parameter('mask_yaml', '/home/autonexa/intelligent_parking_ws/maps/mapppp_roads.yaml')
        
        mask_yaml_path = self.get_parameter('mask_yaml').value
        
        # Load mask
        self.occupancy_grid = self.load_mask(mask_yaml_path)
        
        if self.occupancy_grid is None:
            self.get_logger().error(f'Failed to load road mask from {mask_yaml_path}')
            return
        
        # Publisher with transient local QoS (like map_server)
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        
        self.publisher = self.create_publisher(OccupancyGrid, '/road_mask', qos)
        
        # Publish once and then periodically
        self.publisher.publish(self.occupancy_grid)
        self.get_logger().info(f'Published road mask: {self.occupancy_grid.info.width}x{self.occupancy_grid.info.height}')
        
        # Republish periodically in case of late subscribers
        self.timer = self.create_timer(5.0, self.publish_mask)
    
    def load_mask(self, yaml_path):
        """Load road mask from YAML file and convert to OccupancyGrid"""
        if not os.path.exists(yaml_path):
            self.get_logger().error(f'Mask YAML not found: {yaml_path}')
            return None
        
        try:
            with open(yaml_path, 'r') as f:
                mask_info = yaml.safe_load(f)
            
            mask_dir = os.path.dirname(yaml_path)
            image_path = os.path.join(mask_dir, mask_info['image'])
            
            # Load mask image
            mask_image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
            if mask_image is None:
                self.get_logger().error(f'Failed to load mask image: {image_path}')
                return None
            
            # Convert to occupancy grid format
            # In the saved mask: black (0) = road (navigable), white (255) = blocked
            # In OccupancyGrid: 0 = free, 100 = occupied, -1 = unknown
            height, width = mask_image.shape
            
            # Create occupancy data
            data = []
            for y in range(height - 1, -1, -1):  # Flip Y axis for ROS convention
                for x in range(width):
                    pixel = mask_image[y, x]
                    if pixel < 50:  # Black = road = free
                        data.append(0)
                    elif pixel > 200:  # White = blocked = occupied
                        data.append(100)
                    else:  # Gray = unknown/semi-blocked
                        data.append(50)
            
            # Create OccupancyGrid message
            grid = OccupancyGrid()
            grid.header.frame_id = 'map'
            grid.header.stamp = self.get_clock().now().to_msg()
            
            grid.info.resolution = float(mask_info.get('resolution', 0.02))
            grid.info.width = width
            grid.info.height = height
            
            origin = mask_info.get('origin', [0, 0, 0])
            grid.info.origin.position.x = float(origin[0])
            grid.info.origin.position.y = float(origin[1])
            grid.info.origin.position.z = 0.0
            grid.info.origin.orientation.w = 1.0
            
            grid.data = data
            
            self.get_logger().info(f'Loaded road mask: {width}x{height}, resolution: {grid.info.resolution}')
            return grid
            
        except Exception as e:
            self.get_logger().error(f'Error loading mask: {e}')
            return None
    
    def publish_mask(self):
        """Periodically publish the mask"""
        if self.occupancy_grid:
            self.occupancy_grid.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(self.occupancy_grid)


def main(args=None):
    rclpy.init(args=args)
    node = RoadMaskPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

