#!/usr/bin/env python3
"""
Map Loader Module
Loads PGM map + YAML metadata and provides coordinate transformations
"""

import yaml
import numpy as np
from PIL import Image
import cv2
from typing import Tuple, Optional


class MapLoader:
    """Loads and handles map data with coordinate transformations"""
    
    def __init__(self, map_yaml_path: str):
        """
        Initialize map loader
        
        Args:
            map_yaml_path: Path to map YAML file (e.g., emre.yaml)
        """
        self.map_yaml_path = map_yaml_path
        self.map_dir = map_yaml_path.rsplit('/', 1)[0] if '/' in map_yaml_path else '.'
        
        # Load YAML metadata
        with open(map_yaml_path, 'r') as f:
            self.map_info = yaml.safe_load(f)
        
        # Load map image
        map_image_path = f"{self.map_dir}/{self.map_info['image']}"
        self.map_image = cv2.imread(map_image_path, cv2.IMREAD_GRAYSCALE)
        
        if self.map_image is None:
            raise FileNotFoundError(f"Could not load map image: {map_image_path}")
        
        # Map properties
        self.resolution = float(self.map_info['resolution'])  # meters per pixel
        self.height, self.width = self.map_image.shape  # pixels
        self.origin = self.map_info.get('origin', [0.0, 0.0, 0.0])  # [x, y, theta]
        self.origin_x = float(self.origin[0])
        self.origin_y = float(self.origin[1])
        self.origin_theta = float(self.origin[2])
        
        # Occupancy thresholds
        self.occupied_thresh = float(self.map_info.get('occupied_thresh', 0.65))
        self.free_thresh = float(self.map_info.get('free_thresh', 0.196))
        self.negate = int(self.map_info.get('negate', 0))
        
        # Create binary occupancy map (for path planning)
        self.occupancy_map = self._create_occupancy_map()
    
    def _create_occupancy_map(self) -> np.ndarray:
        """Convert grayscale map to binary occupancy (0=free, 1=occupied)"""
        map_normalized = self.map_image.astype(np.float32) / 255.0
        
        if self.negate:
            map_normalized = 1.0 - map_normalized
        
        # Free = below free_thresh, Occupied = above occupied_thresh
        occupied = (map_normalized > self.occupied_thresh).astype(np.uint8)
        
        return occupied
    
    def world_to_pixel(self, wx: float, wy: float) -> Tuple[int, int]:
        """
        Convert world coordinates (meters) to pixel coordinates
        
        Args:
            wx: World X coordinate (meters)
            wy: World Y coordinate (meters)
        
        Returns:
            (px, py): Pixel coordinates (row, col)
        """
        # Account for origin offset
        px = int((wy - self.origin_y) / self.resolution)
        py = int((wx - self.origin_x) / self.resolution)
        
        # Y-axis is flipped in image coordinates
        px = self.height - 1 - px
        
        # Clamp to image bounds
        px = max(0, min(self.height - 1, px))
        py = max(0, min(self.width - 1, py))
        
        return (px, py)
    
    def pixel_to_world(self, px: int, py: int) -> Tuple[float, float]:
        """
        Convert pixel coordinates to world coordinates (meters)
        
        Args:
            px: Pixel row (0 to height-1)
            py: Pixel column (0 to width-1)
        
        Returns:
            (wx, wy): World coordinates (meters)
        """
        # Y-axis is flipped
        px_flipped = self.height - 1 - px
        
        # Convert to world coordinates
        wy = px_flipped * self.resolution + self.origin_y
        wx = py * self.resolution + self.origin_x
        
        return (wx, wy)
    
    def is_free(self, wx: float, wy: float, radius: float = 0.0) -> bool:
        """
        Check if a world coordinate is in free space
        
        Args:
            wx: World X (meters)
            wy: World Y (meters)
            radius: Safety radius around point (meters)
        
        Returns:
            True if free, False if occupied
        """
        px, py = self.world_to_pixel(wx, wy)
        
        # Check radius around point
        if radius > 0:
            radius_pixels = int(radius / self.resolution)
            for dx in range(-radius_pixels, radius_pixels + 1):
                for dy in range(-radius_pixels, radius_pixels + 1):
                    if dx*dx + dy*dy > radius_pixels*radius_pixels:
                        continue
                    px_check = px + dx
                    py_check = py + dy
                    if (px_check < 0 or px_check >= self.height or
                        py_check < 0 or py_check >= self.width):
                        return False
                    if self.occupancy_map[px_check, py_check] == 1:
                        return False
            return True
        else:
            # Single pixel check
            return self.occupancy_map[px, py] == 0
    
    def get_map_image(self) -> np.ndarray:
        """Get the map image (grayscale)"""
        return self.map_image.copy()
    
    def get_occupancy_map(self) -> np.ndarray:
        """Get binary occupancy map (0=free, 1=occupied)"""
        return self.occupancy_map.copy()
    
    def get_bounds(self) -> Tuple[float, float, float, float]:
        """
        Get world coordinate bounds
        
        Returns:
            (min_x, min_y, max_x, max_y) in meters
        """
        min_x = self.origin_x
        min_y = self.origin_y
        max_x = min_x + self.width * self.resolution
        max_y = min_y + self.height * self.resolution
        return (min_x, min_y, max_x, max_y)

