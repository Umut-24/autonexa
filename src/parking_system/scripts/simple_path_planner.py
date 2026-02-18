#!/usr/bin/env python3
"""
SIMPLE PATH PLANNER - Works with RViz
Subscribes to AMCL, publishes path to /plan for RViz visualization
NO GUI needed - use RViz!
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import String
import numpy as np
import heapq
import yaml
import cv2
import math


class SimplePathPlanner(Node):
    def __init__(self):
        super().__init__('simple_path_planner')
        
        # Parameters
        self.declare_parameter('map_yaml', '/home/autonexa/intelligent_parking_ws/maps/emre.yaml')
        self.declare_parameter('spots_file', '/home/autonexa/intelligent_parking_ws/maps/parking_spots.yaml')
        
        map_yaml = self.get_parameter('map_yaml').value
        spots_file = self.get_parameter('spots_file').value
        
        # Load map
        self.load_map(map_yaml)
        
        # Load parking spots
        self.load_spots(spots_file)
        
        # Current robot pose
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.has_pose = False
        
        # Current goal
        self.goal_spot = None
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        
        self.goal_sub = self.create_subscription(
            String, '/navigate_to_spot', self.goal_callback, 10)
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/plan', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Timer for continuous path publishing
        self.timer = self.create_timer(0.5, self.publish_path)
        
        self.get_logger().info('=== SIMPLE PATH PLANNER READY ===')
        self.get_logger().info(f'Loaded {len(self.spots)} parking spots')
        self.get_logger().info('Send goal: ros2 topic pub --once /navigate_to_spot std_msgs/String "data: spot_1"')
    
    def load_map(self, map_yaml):
        """Load map from YAML"""
        try:
            with open(map_yaml, 'r') as f:
                info = yaml.safe_load(f)
            
            map_dir = map_yaml.rsplit('/', 1)[0]
            img_path = f"{map_dir}/{info['image']}"
            
            self.map_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
            self.resolution = float(info['resolution'])
            self.origin = info['origin']
            self.origin_x = float(self.origin[0])
            self.origin_y = float(self.origin[1])
            self.height, self.width = self.map_img.shape
            
            # Create occupancy grid (0=free, 1=occupied)
            self.occupancy = (self.map_img < 200).astype(np.uint8)
            
            # Inflate obstacles slightly
            kernel = np.ones((3, 3), np.uint8)
            self.occupancy = cv2.dilate(self.occupancy, kernel, iterations=1)
            
            self.get_logger().info(f'Map loaded: {self.width}x{self.height}, res={self.resolution}')
        except Exception as e:
            self.get_logger().error(f'Failed to load map: {e}')
            self.occupancy = np.zeros((100, 100), dtype=np.uint8)
    
    def load_spots(self, spots_file):
        """Load parking spots"""
        self.spots = {}
        try:
            with open(spots_file, 'r') as f:
                data = yaml.safe_load(f) or {}
            self.spots = data.get('parking_spots', {})
        except Exception as e:
            self.get_logger().error(f'Failed to load spots: {e}')
    
    def pose_callback(self, msg):
        """Handle AMCL pose"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Get yaw from quaternion
        q = msg.pose.pose.orientation
        self.robot_theta = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self.has_pose = True
    
    def goal_callback(self, msg):
        """Handle goal request"""
        spot_id = msg.data.strip()
        
        if spot_id not in self.spots:
            self.get_logger().error(f'Unknown spot: {spot_id}. Available: {list(self.spots.keys())}')
            return
        
        self.goal_spot = spot_id
        spot = self.spots[spot_id]
        self.get_logger().info(f'Goal set: {spot_id} at ({spot["x"]:.2f}, {spot["y"]:.2f})')
        
        # Publish goal pose for RViz
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = float(spot['x'])
        goal_msg.pose.position.y = float(spot['y'])
        goal_msg.pose.orientation.w = 1.0
        self.goal_pub.publish(goal_msg)
    
    def world_to_pixel(self, wx, wy):
        """Convert world coords to pixel coords"""
        px = int((wx - self.origin_x) / self.resolution)
        py = int((wy - self.origin_y) / self.resolution)
        py = self.height - 1 - py  # Flip Y
        return (py, px)  # (row, col)
    
    def pixel_to_world(self, row, col):
        """Convert pixel coords to world coords"""
        wx = col * self.resolution + self.origin_x
        wy = (self.height - 1 - row) * self.resolution + self.origin_y
        return (wx, wy)
    
    def plan_astar(self, start, goal):
        """Simple A* path planning"""
        sr, sc = start
        gr, gc = goal
        
        # Validate
        h, w = self.occupancy.shape
        if not (0 <= sr < h and 0 <= sc < w and 0 <= gr < h and 0 <= gc < w):
            return None
        if self.occupancy[sr, sc] == 1 or self.occupancy[gr, gc] == 1:
            return None
        
        # A* search
        open_set = [(0, sr, sc, None)]
        closed = set()
        parent = {}
        
        while open_set:
            f, r, c, p = heapq.heappop(open_set)
            
            if (r, c) in closed:
                continue
            
            closed.add((r, c))
            parent[(r, c)] = p
            
            if r == gr and c == gc:
                # Reconstruct path
                path = []
                cur = (r, c)
                while cur:
                    path.append(cur)
                    cur = parent[cur]
                path.reverse()
                return path
            
            # Neighbors (8-connected)
            for dr in [-1, 0, 1]:
                for dc in [-1, 0, 1]:
                    if dr == 0 and dc == 0:
                        continue
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < h and 0 <= nc < w and (nr, nc) not in closed:
                        if self.occupancy[nr, nc] == 0:
                            g = f + (1.414 if dr != 0 and dc != 0 else 1.0)
                            h_val = math.sqrt((nr-gr)**2 + (nc-gc)**2)
                            heapq.heappush(open_set, (g + h_val, nr, nc, (r, c)))
        
        return None
    
    def publish_path(self):
        """Publish path to /plan"""
        if not self.has_pose or not self.goal_spot:
            return
        
        spot = self.spots.get(self.goal_spot)
        if not spot:
            return
        
        # Convert to pixels
        start = self.world_to_pixel(self.robot_x, self.robot_y)
        goal = self.world_to_pixel(spot['x'], spot['y'])
        
        # Plan path
        path_pixels = self.plan_astar(start, goal)
        
        if not path_pixels:
            self.get_logger().warn('No path found!')
            return
        
        # Convert to Path message
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Simplify path (every 5th point)
        for i, (r, c) in enumerate(path_pixels):
            if i % 5 == 0 or i == len(path_pixels) - 1:
                wx, wy = self.pixel_to_world(r, c)
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = wx
                pose.pose.position.y = wy
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Path published: {len(path_msg.poses)} waypoints')


def main():
    rclpy.init()
    node = SimplePathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

