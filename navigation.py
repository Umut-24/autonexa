#!/usr/bin/env python3
import sys
import threading
import yaml
import numpy as np
import math
from datetime import datetime

# Set matplotlib backend before any Qt imports
import matplotlib
matplotlib.use('Qt5Agg')

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                              QHBoxLayout, QLabel, QLineEdit, QPushButton, 
                              QGroupBox, QTextEdit, QSplitter)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont

import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist


# =======================
# ROS2 AMCL Subscriber
# =======================
class AMCLListener(Node):
    def __init__(self):
        super().__init__('amcl_listener')
        self.pose = None
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.callback,
            10
        )

    def callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # Extract yaw from quaternion (ROS convention: yaw is rotation around z-axis)
        # Standard formula: yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
        yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z)
        )
        self.pose = (x, y, yaw)


# =======================
# A* Planner
# =======================
class AStarPlanner:
    def __init__(self, grid):
        self.grid = grid
        self.h, self.w = grid.shape

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def neighbors(self, node):
        dirs = [(1,0),(-1,0),(0,1),(0,-1)]
        for dx, dy in dirs:
            nx, ny = node[0]+dx, node[1]+dy
            if 0 <= nx < self.h and 0 <= ny < self.w:
                if self.grid[nx, ny] == 0:
                    yield (nx, ny)

    def plan(self, start, goal):
        import heapq
        from scipy import ndimage
        
        # Use cached obstacle map if available
        if not hasattr(self, '_cached_obstacle_map'):
            original_walls = (self.grid > 0.5).astype(np.uint8)
            struct = np.ones((3, 3), dtype=np.uint8)
            closed = ndimage.binary_closing(original_walls, structure=struct, iterations=2).astype(np.uint8)
            self._cached_obstacle_map = ndimage.binary_dilation(closed, structure=struct, iterations=3).astype(np.uint8)
            self._cached_walls = original_walls
        
        obstacle_map = self._cached_obstacle_map
        original_walls = self._cached_walls
        
        # 8-directional movement
        dirs_8 = [(1,0,1.0),(-1,0,1.0),(0,1,1.0),(0,-1,1.0),
                  (1,1,1.414),(1,-1,1.414),(-1,1,1.414),(-1,-1,1.414)]
        
        def heuristic(a, b):
            return abs(a[0]-b[0]) + abs(a[1]-b[1])  # Manhattan is faster than Euclidean
        
        def find_nearest_free(pos):
            if 0 <= pos[0] < self.h and 0 <= pos[1] < self.w and obstacle_map[pos[0], pos[1]] == 0:
                return pos
            for radius in range(1, 20):
                for dr in range(-radius, radius + 1, max(1, radius//2)):
                    for dc in range(-radius, radius + 1, max(1, radius//2)):
                        nr, nc = pos[0] + dr, pos[1] + dc
                        if 0 <= nr < self.h and 0 <= nc < self.w and obstacle_map[nr, nc] == 0:
                            return (nr, nc)
            return pos
        
        def astar(s, g):
            open_set = [(0, s)]
            came_from = {}
            cost_so_far = {s: 0}
            
            while open_set:
                _, current = heapq.heappop(open_set)
                
                if current == g:
                    path = []
                    node = g
                    while node in came_from:
                        path.append(node)
                        node = came_from[node]
                    path.append(s)
                    path.reverse()
                    return path
                
                for dx, dy, mc in dirs_8:
                    nx, ny = current[0] + dx, current[1] + dy
                    if 0 <= nx < self.h and 0 <= ny < self.w and obstacle_map[nx, ny] == 0:
                        nc = cost_so_far[current] + mc
                        if (nx, ny) not in cost_so_far or nc < cost_so_far[(nx, ny)]:
                            cost_so_far[(nx, ny)] = nc
                            heapq.heappush(open_set, (nc + heuristic((nx, ny), g), (nx, ny)))
                            came_from[(nx, ny)] = current
            return []
        
        def path_valid(path):
            if not path:
                return False
            for r, c in path:
                if 0 <= r < self.h and 0 <= c < self.w:
                    if original_walls[r, c] == 1:
                        return False
            return True
        
        # Try to find path
        adj_start = find_nearest_free(start)
        adj_goal = find_nearest_free(goal)
        path = astar(adj_start, adj_goal)
        
        if path and path_valid(path):
            return path
        
        # Fallback: try with more inflation (only if first attempt failed)
        if path and not path_valid(path):
            struct = np.ones((3, 3), dtype=np.uint8)
            obstacle_map = ndimage.binary_dilation(self._cached_obstacle_map, structure=struct, iterations=2).astype(np.uint8)
            adj_start = find_nearest_free(start)
            adj_goal = find_nearest_free(goal)
            path = astar(adj_start, adj_goal)
            if path and path_valid(path):
                return path
        
        return path if path else []


# =======================
# GUI
# =======================
class NavigationGUI(QMainWindow):
    def __init__(self, map_yaml):
        super().__init__()
        self.setWindowTitle("Parking Navigation")
        self.setMinimumSize(900, 600)

        self.map, self.resolution, self.origin = self.load_map(map_yaml)
        self.goal = None
        self.goal_world = None  # Goal in world coordinates
        self.robot_pose = None
        self.current_path = []
        
        # Navigation state
        self.navigation_active = False
        self.has_arrived = False
        self.arrival_threshold = 0.08  # 8cm threshold for arrival detection
        self.arrival_position = None  # Position when arrived
        self.test_results = []  # Store test results
        
        # Path caching
        self.cached_world_path = []
        
        # Setup main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        
        # Left side: Map visualization
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        
        self.fig, self.ax = plt.subplots(figsize=(6, 6), dpi=80)
        self.fig.tight_layout()
        self.canvas = FigureCanvas(self.fig)
        left_layout.addWidget(self.canvas)
        
        # Right side: Controls (simplified)
        right_widget = QWidget()
        right_widget.setMaximumWidth(280)
        right_layout = QVBoxLayout(right_widget)
        right_layout.setSpacing(5)
        
        # Goal Input
        right_layout.addWidget(QLabel("<b>Goal Position (m):</b>"))
        
        coord_layout = QHBoxLayout()
        coord_layout.addWidget(QLabel("X:"))
        self.x_input = QLineEdit()
        self.x_input.setMaximumWidth(80)
        coord_layout.addWidget(self.x_input)
        coord_layout.addWidget(QLabel("Y:"))
        self.y_input = QLineEdit()
        self.y_input.setMaximumWidth(80)
        coord_layout.addWidget(self.y_input)
        right_layout.addLayout(coord_layout)
        
        # Buttons (compact)
        self.set_goal_btn = QPushButton("Set Goal")
        self.set_goal_btn.clicked.connect(self.set_goal_from_input)
        right_layout.addWidget(self.set_goal_btn)
        
        btn_row = QHBoxLayout()
        self.start_nav_btn = QPushButton("Start")
        self.start_nav_btn.clicked.connect(self.start_navigation)
        self.start_nav_btn.setStyleSheet("background-color: #4CAF50; color: white;")
        btn_row.addWidget(self.start_nav_btn)
        
        self.stop_nav_btn = QPushButton("Stop")
        self.stop_nav_btn.clicked.connect(self.stop_navigation)
        self.stop_nav_btn.setStyleSheet("background-color: #f44336; color: white;")
        btn_row.addWidget(self.stop_nav_btn)
        
        self.clear_btn = QPushButton("Clear")
        self.clear_btn.clicked.connect(self.clear_goal)
        btn_row.addWidget(self.clear_btn)
        right_layout.addLayout(btn_row)
        
        # Status (compact)
        right_layout.addWidget(QLabel("<b>Status:</b>"))
        self.status_label = QLabel("IDLE")
        self.status_label.setFont(QFont('Arial', 11, QFont.Bold))
        right_layout.addWidget(self.status_label)
        
        self.robot_pos_label = QLabel("Robot: --")
        right_layout.addWidget(self.robot_pos_label)
        
        self.goal_pos_label = QLabel("Goal: --")
        right_layout.addWidget(self.goal_pos_label)
        
        self.distance_label = QLabel("Distance: --")
        right_layout.addWidget(self.distance_label)
        
        # Arrival
        right_layout.addWidget(QLabel("<b>Arrival:</b>"))
        self.arrival_status = QLabel("NOT ARRIVED")
        self.arrival_status.setFont(QFont('Arial', 10, QFont.Bold))
        right_layout.addWidget(self.arrival_status)
        
        self.error_label = QLabel("Error: --")
        right_layout.addWidget(self.error_label)
        self.error_x_label = QLabel("  X: --")
        right_layout.addWidget(self.error_x_label)
        self.error_y_label = QLabel("  Y: --")
        right_layout.addWidget(self.error_y_label)
        
        # Log (smaller)
        right_layout.addWidget(QLabel("<b>Log:</b>"))
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(120)
        right_layout.addWidget(self.log_text)
        
        self.save_log_btn = QPushButton("Save Results")
        self.save_log_btn.clicked.connect(self.save_test_results)
        right_layout.addWidget(self.save_log_btn)
        
        right_layout.addStretch()
        
        # Add to main layout
        main_layout.addWidget(left_widget, stretch=3)
        main_layout.addWidget(right_widget, stretch=1)

        # Set up coordinate system with extent to match ROS map coordinates
        height, width = self.map.shape
        x_min = self.origin[0]
        y_min = self.origin[1]
        x_max = x_min + height * self.resolution
        y_max = y_min + width * self.resolution
        
        self.ax.imshow(self.map, cmap='gray', extent=[x_min, x_max, y_min, y_max], origin='lower')
        self.ax.set_aspect('equal')
        self.canvas.mpl_connect('button_press_event', self.on_click)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(300)  # 300ms update interval (balanced)
        
        self.log_message("System initialized. Click on map or enter coordinates to set goal.")

    def load_map(self, yaml_file):
        import os
        with open(yaml_file, 'r') as f:
            data = yaml.safe_load(f)
        # Get the directory of the yaml file to resolve relative image paths
        yaml_dir = os.path.dirname(os.path.abspath(yaml_file))
        image_path = os.path.join(yaml_dir, data['image'])
        img = plt.imread(image_path)
        grid = (img < 0.5).astype(int)
        # Flip vertically: ROS maps have origin at bottom-left, but images have row 0 at top
        # After flipud, row 0 is at bottom (matching ROS convention)
        grid = np.flipud(grid)
        # No horizontal flip needed - map orientation should match RViz
        return grid, data['resolution'], data['origin']

    def world_to_map(self, x, y):
        # ROS map_server standard conversion:
        # After flipud, row 0 is at bottom (matching ROS convention)
        # Standard conversion: pixel_col = (world_y - origin_y) / resolution
        #                     pixel_row = (world_x - origin_x) / resolution
        height, width = self.map.shape
        pixel_col = (y - self.origin[1]) / self.resolution
        pixel_row = (x - self.origin[0]) / self.resolution
        # Clamp to valid range
        pixel_row = max(0, min(int(pixel_row), height - 1))
        pixel_col = max(0, min(int(pixel_col), width - 1))
        return int(pixel_row), int(pixel_col)

    def map_to_world(self, mx, my):
        # mx, my are pixel coordinates (row, col) after flipud
        # Convert to world coordinates using standard ROS map_server convention
        x = mx * self.resolution + self.origin[0]
        y = my * self.resolution + self.origin[1]
        return x, y

    def log_message(self, msg):
        """Add a timestamped message to the log"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f"[{timestamp}] {msg}")
        self.log_text.verticalScrollBar().setValue(self.log_text.verticalScrollBar().maximum())
    
    def set_goal_from_input(self):
        """Set goal from keyboard input"""
        try:
            x = float(self.x_input.text())
            y = float(self.y_input.text())
            self.set_goal_world(x, y)
            self.log_message(f"Goal set from input: ({x:.3f}, {y:.3f})")
        except ValueError:
            self.log_message("ERROR: Invalid coordinates. Please enter valid numbers.")
    
    def set_goal_world(self, world_x, world_y):
        """Set goal using world coordinates"""
        pixel_coords = self.world_to_map(world_x, world_y)
        self.goal = pixel_coords
        self.goal_world = (world_x, world_y)
        self.has_arrived = False
        self.arrival_position = None
        self.cached_world_path = []
        self.goal_pos_label.setText(f"Goal: ({world_x:.3f}, {world_y:.3f})")
        self.arrival_status.setText("Arrival: NOT ARRIVED")
        self.arrival_status.setStyleSheet("color: black;")
        self.error_label.setText("Position Error: --")
        self.error_x_label.setText("  X Error: --")
        self.error_y_label.setText("  Y Error: --")
    
    def start_navigation(self):
        """Start navigation to the goal"""
        if self.goal is None:
            self.log_message("ERROR: No goal set. Please set a goal first.")
            return
        
        self.navigation_active = True
        self.has_arrived = False
        self.status_label.setText("Status: NAVIGATING")
        self.status_label.setStyleSheet("color: blue;")
        self.log_message(f"Navigation started to ({self.goal_world[0]:.3f}, {self.goal_world[1]:.3f})")
    
    def stop_navigation(self):
        """Stop navigation"""
        self.navigation_active = False
        self.status_label.setText("Status: STOPPED")
        self.status_label.setStyleSheet("color: orange;")
        self.log_message("Navigation stopped by user.")
    
    def clear_goal(self):
        """Clear the current goal"""
        self.goal = None
        self.goal_world = None
        self.navigation_active = False
        self.has_arrived = False
        self.current_path = []
        self.cached_world_path = []
        self.status_label.setText("IDLE")
        self.status_label.setStyleSheet("color: black;")
        self.goal_pos_label.setText("Goal: --")
        self.distance_label.setText("Distance: --")
        self.arrival_status.setText("NOT ARRIVED")
        self.arrival_status.setStyleSheet("color: black;")
        self.error_label.setText("Error: --")
        self.error_x_label.setText("  X: --")
        self.error_y_label.setText("  Y: --")
        self.log_message("Goal cleared.")
    
    def check_arrival(self):
        """Check if robot has arrived at the goal"""
        if not self.goal_world or not self.robot_pose or not self.navigation_active:
            return False
        
        robot_x, robot_y = self.robot_pose[0], self.robot_pose[1]
        goal_x, goal_y = self.goal_world
        
        distance = math.sqrt((robot_x - goal_x)**2 + (robot_y - goal_y)**2)
        
        if distance <= self.arrival_threshold and not self.has_arrived:
            self.has_arrived = True
            self.arrival_position = (robot_x, robot_y)
            self.navigation_active = False
            
            # Calculate errors
            error_x = robot_x - goal_x
            error_y = robot_y - goal_y
            error_total = distance
            
            # Update UI
            self.status_label.setText("Status: ARRIVED!")
            self.status_label.setStyleSheet("color: green; font-size: 14px;")
            self.arrival_status.setText("🎉 YOU REACHED THE PARKING SPOT! 🎉")
            self.arrival_status.setStyleSheet("color: green; font-weight: bold;")
            
            self.error_label.setText(f"Position Error: {error_total*100:.2f} cm")
            self.error_x_label.setText(f"  X Error: {error_x*100:.2f} cm ({error_x:.4f} m)")
            self.error_y_label.setText(f"  Y Error: {error_y*100:.2f} cm ({error_y:.4f} m)")
            
            # Log result
            result = {
                'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                'goal': self.goal_world,
                'arrival': self.arrival_position,
                'error_x': error_x,
                'error_y': error_y,
                'error_total': error_total
            }
            self.test_results.append(result)
            
            self.log_message("=" * 40)
            self.log_message("🎉 ARRIVED AT PARKING SPOT!")
            self.log_message(f"Goal: ({goal_x:.4f}, {goal_y:.4f})")
            self.log_message(f"Actual: ({robot_x:.4f}, {robot_y:.4f})")
            self.log_message(f"Error: {error_total*100:.2f} cm")
            self.log_message(f"  X Error: {error_x*100:.2f} cm")
            self.log_message(f"  Y Error: {error_y*100:.2f} cm")
            self.log_message("=" * 40)
            
            return True
        
        return False
    
    def save_test_results(self):
        """Save test results to file"""
        if not self.test_results:
            self.log_message("No test results to save.")
            return
        
        filename = f"parking_test_results_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
        with open(filename, 'w') as f:
            f.write("=" * 60 + "\n")
            f.write("PARKING NAVIGATION TEST RESULTS\n")
            f.write("=" * 60 + "\n\n")
            
            total_error = 0
            for i, result in enumerate(self.test_results, 1):
                f.write(f"Test #{i}\n")
                f.write(f"  Timestamp: {result['timestamp']}\n")
                f.write(f"  Goal Position: ({result['goal'][0]:.4f}, {result['goal'][1]:.4f})\n")
                f.write(f"  Arrival Position: ({result['arrival'][0]:.4f}, {result['arrival'][1]:.4f})\n")
                f.write(f"  X Error: {result['error_x']*100:.2f} cm\n")
                f.write(f"  Y Error: {result['error_y']*100:.2f} cm\n")
                f.write(f"  Total Error: {result['error_total']*100:.2f} cm\n")
                f.write("\n")
                total_error += result['error_total']
            
            avg_error = total_error / len(self.test_results)
            f.write("=" * 60 + "\n")
            f.write(f"SUMMARY: {len(self.test_results)} tests\n")
            f.write(f"Average Error: {avg_error*100:.2f} cm\n")
            f.write("=" * 60 + "\n")
        
        self.log_message(f"Test results saved to: {filename}")

    def on_click(self, event):
        if event.xdata is None or event.ydata is None:
            return
        # event.xdata and event.ydata are now in world coordinates (due to extent)
        world_x = event.xdata
        world_y = event.ydata
        self.set_goal_world(world_x, world_y)
        # Update input fields
        self.x_input.setText(f"{world_x:.3f}")
        self.y_input.setText(f"{world_y:.3f}")
        self.log_message(f"Goal selected on map: ({world_x:.3f}, {world_y:.3f})")

    def update(self):
        self.ax.clear()
        
        # Cache extent calculations
        if not hasattr(self, '_extent'):
            height, width = self.map.shape
            x_min, y_min = self.origin[0], self.origin[1]
            x_max = x_min + height * self.resolution
            y_max = y_min + width * self.resolution
            self._extent = [x_min, x_max, y_min, y_max]
        
        self.ax.imshow(self.map, cmap='gray', extent=self._extent, origin='lower')
        self.ax.set_aspect('equal')

        if amcl_node.pose:
            self.robot_pose = amcl_node.pose
            x_world, y_world, yaw = self.robot_pose
            
            # Update labels
            self.robot_pos_label.setText(f"Robot: ({x_world:.3f}, {y_world:.3f})")
            
            # Plot robot
            color = 'lime' if self.has_arrived else 'red'
            self.ax.plot(x_world, y_world, 'o', color=color, markersize=8)
            
            # Simple direction line instead of arrow (faster)
            dx = 0.12 * math.cos(yaw)
            dy = 0.12 * math.sin(yaw)
            self.ax.plot([x_world, x_world+dx], [y_world, y_world+dy], color=color, linewidth=2)
            
            # Check arrival
            self.check_arrival()
            
            # Update distance
            if self.goal_world:
                dist = math.sqrt((x_world - self.goal_world[0])**2 + (y_world - self.goal_world[1])**2)
                self.distance_label.setText(f"Distance: {dist*100:.1f} cm")

        if self.goal and self.robot_pose:
            # Always recalculate path when robot moves (unless arrived)
            if not self.has_arrived:
                planner = AStarPlanner(self.map)
                start = self.world_to_map(self.robot_pose[0], self.robot_pose[1])
                self.current_path = planner.plan(start, self.goal)
                if self.current_path:
                    self.cached_world_path = [self.map_to_world(r, c) for r, c in self.current_path]
            
            # Draw path
            if self.cached_world_path:
                px = [p[0] for p in self.cached_world_path]
                py = [p[1] for p in self.cached_world_path]
                color = 'b' if self.navigation_active else ('g' if self.has_arrived else 'c')
                self.ax.plot(px, py, color+'-', linewidth=2)

            # Draw goal
            if self.goal_world:
                marker = '*' if self.has_arrived else 'x'
                color = 'gold' if self.has_arrived else 'green'
                self.ax.plot(self.goal_world[0], self.goal_world[1], marker, color=color, markersize=12)
        
        self.canvas.draw_idle()  # More efficient than draw()


# =======================
# Main
# =======================
def ros_spin():
    rclpy.spin(amcl_node)

if __name__ == '__main__':
    rclpy.init()
    amcl_node = AMCLListener()

    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    app = QApplication(sys.argv)
    gui = NavigationGUI("maps/3012map.yaml")
    gui.show()
    sys.exit(app.exec_())

