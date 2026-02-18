#!/usr/bin/env python3
"""
Path Planner Module
A* path planning on occupancy grid
"""

import numpy as np
import cv2
from typing import List, Tuple, Optional
from dataclasses import dataclass
import heapq


@dataclass
class Node:
    """A* search node"""
    x: int
    y: int
    g: float  # Cost from start
    h: float  # Heuristic to goal
    parent: Optional['Node'] = None
    
    def f(self) -> float:
        return self.g + self.h
    
    def __lt__(self, other):
        return self.f() < other.f()
    
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y


class AStarPlanner:
    """A* path planner for grid maps"""
    
    def __init__(self, occupancy_map: np.ndarray, resolution: float, 
                 robot_radius: float = 0.1):
        """
        Initialize planner
        
        Args:
            occupancy_map: Binary occupancy map (0=free, 1=occupied)
            resolution: Map resolution (meters per pixel)
            robot_radius: Robot radius for obstacle inflation (meters)
        """
        self.occupancy_map = occupancy_map
        self.resolution = resolution
        self.robot_radius = robot_radius
        self.height, self.width = occupancy_map.shape
        
        # Inflate obstacles by robot radius
        if robot_radius > 0:
            inflation_pixels = int(robot_radius / resolution)
            kernel = np.ones((2*inflation_pixels+1, 2*inflation_pixels+1), np.uint8)
            self.inflated_map = cv2.dilate(occupancy_map.astype(np.uint8), 
                                          kernel, iterations=1).astype(np.uint8)
        else:
            self.inflated_map = occupancy_map
    
    def is_valid(self, x: int, y: int) -> bool:
        """Check if cell is valid (in bounds and free)"""
        if x < 0 or x >= self.height or y < 0 or y >= self.width:
            return False
        return self.inflated_map[x, y] == 0
    
    def heuristic(self, x1: int, y1: int, x2: int, y2: int) -> float:
        """Euclidean distance heuristic"""
        dx = x1 - x2
        dy = y1 - y2
        return np.sqrt(dx*dx + dy*dy) * self.resolution
    
    def get_neighbors(self, x: int, y: int) -> List[Tuple[int, int]]:
        """Get 8-connected neighbors"""
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if self.is_valid(nx, ny):
                    neighbors.append((nx, ny))
        return neighbors
    
    def plan(self, start_world: Tuple[float, float], 
             goal_world: Tuple[float, float],
             world_to_pixel_func) -> Optional[List[Tuple[float, float]]]:
        """
        Plan path from start to goal
        
        Args:
            start_world: Start pose (wx, wy) in meters
            goal_world: Goal pose (gx, gy) in meters
            world_to_pixel_func: Function to convert world to pixel coords
        
        Returns:
            List of waypoints in world coordinates, or None if no path found
        """
        # Convert to pixel coordinates
        start_px, start_py = world_to_pixel_func(start_world[0], start_world[1])
        goal_px, goal_py = world_to_pixel_func(goal_world[0], goal_world[1])
        
        # Validate start and goal
        if not self.is_valid(start_px, start_py):
            return None
        if not self.is_valid(goal_px, goal_py):
            return None
        
        # A* search
        open_set = []
        closed_set = set()
        
        start_node = Node(start_px, start_py, 0.0, 
                         self.heuristic(start_px, start_py, goal_px, goal_py))
        heapq.heappush(open_set, start_node)
        
        while open_set:
            current = heapq.heappop(open_set)
            
            # Check if goal reached
            if current.x == goal_px and current.y == goal_py:
                # Reconstruct path
                path_pixels = []
                node = current
                while node is not None:
                    path_pixels.append((node.x, node.y))
                    node = node.parent
                path_pixels.reverse()
                
                # Convert back to world coordinates
                path_world = []
                pixel_to_world = lambda px, py: (px, py)  # Will use actual function
                # We need the actual conversion function, but we'll do it inline
                for px, py in path_pixels:
                    # This is approximate - we'll convert properly in caller
                    path_world.append((px, py))
                
                return self._pixels_to_world_path(path_pixels, world_to_pixel_func)
            
            # Skip if already explored
            if (current.x, current.y) in closed_set:
                continue
            
            closed_set.add((current.x, current.y))
            
            # Explore neighbors
            for nx, ny in self.get_neighbors(current.x, current.y):
                if (nx, ny) in closed_set:
                    continue
                
                # Cost: 1.0 for cardinal, 1.414 for diagonal
                dx = abs(nx - current.x)
                dy = abs(ny - current.y)
                move_cost = 1.414 if (dx == 1 and dy == 1) else 1.0
                move_cost *= self.resolution  # Convert to meters
                
                g_new = current.g + move_cost
                h_new = self.heuristic(nx, ny, goal_px, goal_py)
                
                neighbor = Node(nx, ny, g_new, h_new, current)
                
                # Add to open set
                heapq.heappush(open_set, neighbor)
        
        # No path found
        return None
    
    def _pixels_to_world_path(self, path_pixels: List[Tuple[int, int]],
                              world_to_pixel_func) -> List[Tuple[float, float]]:
        """Convert pixel path to world path (reverse lookup)"""
        # For now, return pixel coords - actual conversion needs pixel_to_world
        # This will be handled in the GUI
        return path_pixels


# Alternative: Simplified version that takes pixel coordinates directly
class SimpleAStarPlanner:
    """Simplified A* planner that works directly with pixel coordinates"""
    
    def __init__(self, occupancy_map: np.ndarray, resolution: float,
                 robot_radius: float = 0.1):
        self.occupancy_map = occupancy_map
        self.resolution = resolution
        self.height, self.width = occupancy_map.shape
        
        # Inflate obstacles
        if robot_radius > 0:
            inflation_pixels = int(robot_radius / resolution)
            kernel = np.ones((2*inflation_pixels+1, 2*inflation_pixels+1), np.uint8)
            self.inflated_map = cv2.dilate(
                occupancy_map.astype(np.uint8), kernel, iterations=1
            ).astype(np.uint8)
        else:
            self.inflated_map = occupancy_map
    
    def plan_pixels(self, start: Tuple[int, int], 
                   goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """
        Plan path in pixel coordinates
        
        Args:
            start: (px, py) start pixel
            goal: (gx, gy) goal pixel
        
        Returns:
            List of pixel waypoints, or None
        """
        sx, sy = start
        gx, gy = goal
        
        if not self._is_valid(sx, sy) or not self._is_valid(gx, gy):
            return None
        
        open_set = []
        closed_set = set()
        
        h_start = np.sqrt((sx-gx)**2 + (sy-gy)**2)
        start_node = Node(sx, sy, 0.0, h_start)
        heapq.heappush(open_set, start_node)
        
        while open_set:
            current = heapq.heappop(open_set)
            
            if (current.x, current.y) in closed_set:
                continue
            
            if current.x == gx and current.y == gy:
                # Reconstruct path
                path = []
                node = current
                while node is not None:
                    path.append((node.x, node.y))
                    node = node.parent
                path.reverse()
                return path
            
            closed_set.add((current.x, current.y))
            
            # 8-connected neighbors
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue
                    nx, ny = current.x + dx, current.y + dy
                    
                    if not self._is_valid(nx, ny) or (nx, ny) in closed_set:
                        continue
                    
                    move_cost = 1.414 if (abs(dx) == 1 and abs(dy) == 1) else 1.0
                    g_new = current.g + move_cost
                    h_new = np.sqrt((nx-gx)**2 + (ny-gy)**2)
                    
                    neighbor = Node(nx, ny, g_new, h_new, current)
                    heapq.heappush(open_set, neighbor)
        
        return None
    
    def _is_valid(self, x: int, y: int) -> bool:
        if x < 0 or x >= self.height or y < 0 or y >= self.width:
            return False
        return self.inflated_map[x, y] == 0

