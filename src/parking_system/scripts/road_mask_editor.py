#!/usr/bin/env python3
"""
Road Mask Editor - Create and edit road masks for navigation

This tool allows you to define ALLOWED navigation paths on your map.
- Start with all areas BLOCKED (only walls visible)
- Draw roads/paths in BLUE where the robot CAN navigate
- Only the drawn roads will allow navigation

Usage:
  ros2 run parking_system road_mask_editor.py --map <map.yaml>

Controls:
  Left Mouse: Draw/Erase
  D: Draw mode (add roads)
  E: Erase mode (remove roads)
  +/-: Change brush size
  A: Auto-generate roads from map free space
  C: Clear all roads
  S: Save mask
  Q: Quit
"""

import cv2
import numpy as np
import yaml
import os
import sys
import argparse


class RoadMaskEditor:
    def __init__(self, map_yaml_path):
        self.map_yaml_path = map_yaml_path
        self.map_dir = os.path.dirname(map_yaml_path)
        
        # Load map metadata
        with open(map_yaml_path, 'r') as f:
            self.map_info = yaml.safe_load(f)
        
        # Load map image
        map_image_path = os.path.join(self.map_dir, self.map_info['image'])
        self.original_map = cv2.imread(map_image_path, cv2.IMREAD_GRAYSCALE)
        
        if self.original_map is None:
            raise FileNotFoundError(f"Could not load map image: {map_image_path}")
        
        self.height, self.width = self.original_map.shape
        
        # Road mask - start with NO roads (all black = forbidden)
        # White (255) = navigable road, Black (0) = forbidden
        self.road_mask = np.zeros_like(self.original_map)
        
        # Check if road mask already exists
        mask_path = self.get_mask_path()
        if os.path.exists(mask_path):
            print(f"Loading existing road mask from {mask_path}")
            self.road_mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
        
        # Drawing state
        self.drawing = False
        self.last_point = None
        self.brush_size = 8  # Thinner default for roads
        self.draw_mode = 'draw'  # 'draw' or 'erase'
        self.line_mode = True  # True = draw lines, False = fill brush
        
        # Display
        self.display = None
        self.update_display()
    
    def get_mask_path(self):
        """Get path for road mask file"""
        base_name = os.path.splitext(self.map_info['image'])[0]
        return os.path.join(self.map_dir, f"{base_name}_roads.pgm")
    
    def get_mask_yaml_path(self):
        """Get path for road mask YAML file"""
        base_name = os.path.splitext(os.path.basename(self.map_yaml_path))[0]
        return os.path.join(self.map_dir, f"{base_name}_roads.yaml")
    
    def update_display(self):
        """Update the display image with roads shown in blue"""
        # Create color display from original map
        self.display = cv2.cvtColor(self.original_map, cv2.COLOR_GRAY2BGR)
        
        # Create road overlay - roads shown in BLUE color
        # This makes them visible on both white and black areas
        road_overlay = np.zeros_like(self.display)
        
        # Where road_mask is white (255), draw blue roads
        road_pixels = self.road_mask > 128
        road_overlay[road_pixels] = [255, 150, 50]  # Blue-ish color (BGR)
        
        # Blend the road overlay with the map
        alpha = 0.6
        self.display = cv2.addWeighted(self.display, 1.0, road_overlay, alpha, 0)
        
        # Draw a border around road areas for clarity
        contours, _ = cv2.findContours(self.road_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(self.display, contours, -1, (255, 100, 0), 1)
        
        # Draw info bar at top
        info_bar = np.zeros((60, self.width, 3), dtype=np.uint8)
        info_bar[:] = (40, 40, 40)
        
        # Mode indicator
        mode_color = (0, 255, 0) if self.draw_mode == 'draw' else (0, 0, 255)
        mode_text = "DRAW ROADS" if self.draw_mode == 'draw' else "ERASE ROADS"
        cv2.putText(info_bar, mode_text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, mode_color, 2)
        
        # Road width (brush size)
        cv2.putText(info_bar, f"Road Width: {self.brush_size * 2}px", (200, 25), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        
        # Instructions
        cv2.putText(info_bar, "D=Draw  E=Erase  +/-=Width  A=Auto  C=Clear  S=Save  Q=Quit", 
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 150), 1)
        
        # Combine info bar with display
        self.display = np.vstack([info_bar, self.display])
    
    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse events for drawing"""
        # Adjust y for info bar offset
        y_adjusted = y - 60
        if y_adjusted < 0:
            return
        
        draw_value = 255 if self.draw_mode == 'draw' else 0
        
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.last_point = (x, y_adjusted)
            cv2.circle(self.road_mask, (x, y_adjusted), self.brush_size, draw_value, -1)
            self.update_display()
        
        elif event == cv2.EVENT_MOUSEMOVE:
            if self.drawing:
                current_point = (x, y_adjusted)
                if self.line_mode and self.last_point is not None:
                    # Draw a line from last point to current point
                    cv2.line(self.road_mask, self.last_point, current_point, draw_value, self.brush_size * 2)
                else:
                    cv2.circle(self.road_mask, current_point, self.brush_size, draw_value, -1)
                self.last_point = current_point
                self.update_display()
        
        elif event == cv2.EVENT_LBUTTONUP:
            self.drawing = False
            self.last_point = None
    
    def save_mask(self):
        """Save the road mask"""
        mask_path = self.get_mask_path()
        mask_yaml_path = self.get_mask_yaml_path()
        
        # Save mask image (inverted for Nav2 keepout filter)
        # In Nav2 keepout: black=free, white=blocked
        # Our mask: white=road (navigable), black=blocked
        # So we need to invert for Nav2: road becomes black (free), non-road becomes white (blocked)
        nav2_mask = 255 - self.road_mask
        cv2.imwrite(mask_path, nav2_mask)
        print(f"Saved road mask to: {mask_path}")
        
        # Also save original mask for reference
        original_mask_path = mask_path.replace('.pgm', '_original.pgm')
        cv2.imwrite(original_mask_path, self.road_mask)
        
        # Save mask YAML (same format as map for Nav2 costmap filter)
        mask_yaml = {
            'image': os.path.basename(mask_path),
            'resolution': self.map_info['resolution'],
            'origin': self.map_info['origin'],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.25,
            'mode': 'trinary'
        }
        
        with open(mask_yaml_path, 'w') as f:
            yaml.dump(mask_yaml, f, default_flow_style=False)
        print(f"Saved mask YAML to: {mask_yaml_path}")
        
        return mask_path, mask_yaml_path
    
    def auto_generate_from_map(self, threshold=200):
        """Auto-generate roads from map free space (white areas)"""
        # Copy free space from original map as roads
        self.road_mask = np.where(self.original_map > threshold, 255, 0).astype(np.uint8)
        
        # Erode slightly to keep away from walls
        kernel = np.ones((5, 5), np.uint8)
        self.road_mask = cv2.erode(self.road_mask, kernel, iterations=2)
        
        self.update_display()
        print("Auto-generated roads from map free space (you can erase unwanted areas)")
    
    def run(self):
        """Run the interactive editor"""
        window_name = "Road Mask Editor - Draw Navigation Paths"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, min(1200, self.width), min(900, self.height + 60))
        cv2.setMouseCallback(window_name, self.mouse_callback)
        
        print("\n" + "="*60)
        print("ROAD MASK EDITOR - Define Allowed Navigation Paths")
        print("="*60)
        print("\nThe robot will ONLY be able to navigate on the BLUE roads")
        print("you draw. All other areas will be blocked.\n")
        print("Controls:")
        print("  Left Mouse + Drag: Draw/Erase roads")
        print("  D: Switch to DRAW mode (add roads - blue)")
        print("  E: Switch to ERASE mode (remove roads)")
        print("  +/=: Increase brush size")
        print("  -: Decrease brush size")
        print("  A: Auto-generate roads from all white areas")
        print("  C: Clear all roads (start fresh)")
        print("  S: Save road mask")
        print("  Q: Quit")
        print("="*60 + "\n")
        
        while True:
            cv2.imshow(window_name, self.display)
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                break
            elif key == ord('s'):
                self.save_mask()
            elif key == ord('d'):
                self.draw_mode = 'draw'
                print("Mode: DRAW roads (blue)")
                self.update_display()
            elif key == ord('e'):
                self.draw_mode = 'erase'
                print("Mode: ERASE roads")
                self.update_display()
            elif key == ord('+') or key == ord('='):
                self.brush_size = min(100, self.brush_size + 5)
                print(f"Brush size: {self.brush_size}")
                self.update_display()
            elif key == ord('-'):
                self.brush_size = max(3, self.brush_size - 5)
                print(f"Brush size: {self.brush_size}")
                self.update_display()
            elif key == ord('a'):
                self.auto_generate_from_map()
                print("Auto-generated roads. Use E (erase) to remove unwanted areas.")
            elif key == ord('c'):
                self.road_mask = np.zeros_like(self.original_map)
                print("Cleared all roads")
                self.update_display()
        
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description='Road Mask Editor - Draw navigation paths')
    parser.add_argument('--map', '-m', required=True, help='Path to map YAML file')
    args = parser.parse_args()
    
    if not os.path.exists(args.map):
        print(f"Error: Map file not found: {args.map}")
        sys.exit(1)
    
    editor = RoadMaskEditor(args.map)
    editor.run()


if __name__ == '__main__':
    main()
