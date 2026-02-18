#!/usr/bin/env python3
"""
Navigation GUI Application
PyQt5-based GUI for custom navigation visualization
"""

import sys
import math
import numpy as np
from typing import Optional, Tuple, List
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QLabel, QComboBox,
                             QMessageBox)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QObject, QPointF
from PyQt5.QtGui import QImage, QPixmap, QPainter, QPen, QColor, QPolygonF
import rclpy
from rclpy.executors import SingleThreadedExecutor

try:
    from .map_loader import MapLoader
    from .path_planner import SimpleAStarPlanner
    from .parking_spots import ParkingSpotsManager
    from .amcl_subscriber import AMCLSubscriber
except ImportError:
    # Fallback for direct execution
    from map_loader import MapLoader
    from path_planner import SimpleAStarPlanner
    from parking_spots import ParkingSpotsManager
    from amcl_subscriber import AMCLSubscriber


class ROSThread(QObject):
    """ROS spin thread for GUI"""
    pose_updated = pyqtSignal(float, float, float)  # x, y, theta
    
    def __init__(self, amcl_sub: AMCLSubscriber):
        super().__init__()
        self.amcl_sub = amcl_sub
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(amcl_sub)
    
    def spin_once(self):
        """Spin ROS once (called from QTimer)"""
        self.executor.spin_once(timeout_sec=0.0)
        pose = self.amcl_sub.get_pose()
        if pose:
            self.pose_updated.emit(pose[0], pose[1], pose[2])


class MapDisplayWidget(QWidget):
    """Custom widget to display map, robot, and path"""
    
    def __init__(self, map_loader: MapLoader):
        super().__init__()
        self.map_loader = map_loader
        self.map_image = map_loader.get_map_image()
        self.setMinimumSize(800, 600)
        
        # State
        self.robot_pose: Optional[Tuple[float, float, float]] = None  # (x, y, theta)
        self.goal_pose: Optional[Tuple[float, float]] = None  # (x, y)
        self.current_path: List[Tuple[float, float]] = []  # List of (x, y) waypoints
        
        # Convert map image to QPixmap
        height, width = self.map_image.shape
        qimage = QImage(self.map_image.data, width, height, width, QImage.Format_Grayscale8)
        self.map_pixmap = QPixmap.fromImage(qimage)
        self.setFixedSize(width, height)
    
    def set_robot_pose(self, x: float, y: float, theta: float):
        """Update robot pose"""
        self.robot_pose = (x, y, theta)
        self.update()  # Trigger repaint
    
    def set_goal_pose(self, x: float, y: float):
        """Set goal position"""
        self.goal_pose = (x, y)
        self.update()
    
    def set_path(self, path: List[Tuple[float, float]]):
        """Update path to display"""
        self.current_path = path
        self.update()
    
    def world_to_pixel(self, wx: float, wy: float) -> Tuple[int, int]:
        """Convert world to pixel coordinates"""
        return self.map_loader.world_to_pixel(wx, wy)
    
    def paintEvent(self, event):
        """Paint the map, robot, goal, and path"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Draw map
        painter.drawPixmap(0, 0, self.map_pixmap)
        
        # Draw path (green line)
        if len(self.current_path) > 1:
            painter.setPen(QPen(QColor(0, 255, 0), 3))  # Green, 3px wide
            path_points = [QPointF(*self.world_to_pixel(wx, wy)) for wx, wy in self.current_path]
            for i in range(len(path_points) - 1):
                painter.drawLine(path_points[i], path_points[i + 1])
        
        # Draw goal (red circle)
        if self.goal_pose:
            gx, gy = self.goal_pose
            px, py = self.world_to_pixel(gx, gy)
            painter.setPen(QPen(QColor(255, 0, 0), 2))
            painter.setBrush(QColor(255, 0, 0, 128))  # Semi-transparent red
            painter.drawEllipse(px - 10, py - 10, 20, 20)
        
        # Draw robot (blue arrow)
        if self.robot_pose:
            rx, ry, rtheta = self.robot_pose
            px, py = self.world_to_pixel(rx, ry)
            
            # Draw robot as arrow
            painter.setPen(QPen(QColor(0, 0, 255), 2))
            painter.setBrush(QColor(0, 0, 255, 128))
            
            # Arrow pointing in direction of theta
            arrow_length = 20
            arrow_width = 10
            
            # Calculate arrow points
            end_x = px + arrow_length * math.cos(rtheta)
            end_y = py + arrow_length * math.sin(rtheta)
            
            # Arrow head points
            perp_angle = rtheta + math.pi / 2
            head1_x = end_x - arrow_width * math.cos(perp_angle)
            head1_y = end_y - arrow_width * math.sin(perp_angle)
            head2_x = end_x + arrow_width * math.cos(perp_angle)
            head2_y = end_y + arrow_width * math.sin(perp_angle)
            
            # Draw arrow
            arrow = QPolygonF([
                QPointF(px, py),
                QPointF(end_x, end_y),
                QPointF(head1_x, head1_y),
                QPointF(head2_x, head2_y),
                QPointF(end_x, end_y)
            ])
            painter.drawPolygon(arrow)
            
            # Draw robot center circle
            painter.drawEllipse(px - 5, py - 5, 10, 10)


class NavigationGUI(QMainWindow):
    """Main navigation GUI window"""
    
    def __init__(self, map_yaml: str, spots_file: str):
        super().__init__()
        self.setWindowTitle("Custom Navigation GUI")
        
        # Initialize components
        self.map_loader = MapLoader(map_yaml)
        self.spots_manager = ParkingSpotsManager(spots_file)
        self.planner = SimpleAStarPlanner(
            self.map_loader.get_occupancy_map(),
            self.map_loader.resolution,
            robot_radius=0.1
        )
        
        # Initialize ROS
        rclpy.init()
        self.amcl_sub = AMCLSubscriber()
        self.ros_thread = ROSThread(self.amcl_sub)
        self.ros_thread.pose_updated.connect(self.on_pose_updated)
        
        # Setup GUI
        self.setup_ui()
        
        # Timer for ROS spinning
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.ros_thread.spin_once)
        self.ros_timer.start(50)  # 20 Hz
        
        # Current goal
        self.current_goal_id: Optional[str] = None
    
    def setup_ui(self):
        """Setup user interface"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        layout = QVBoxLayout(central_widget)
        
        # Control panel
        control_layout = QHBoxLayout()
        
        # Parking spot selection
        control_layout.addWidget(QLabel("Parking Spot:"))
        self.spot_combo = QComboBox()
        self.spot_combo.addItems(["None"] + self.spots_manager.list_spot_ids())
        self.spot_combo.currentTextChanged.connect(self.on_spot_selected)
        control_layout.addWidget(self.spot_combo)
        
        # Status label
        self.status_label = QLabel("Status: Waiting for AMCL pose...")
        control_layout.addWidget(self.status_label)
        
        control_layout.addStretch()
        
        # Clear button
        clear_btn = QPushButton("Clear Path")
        clear_btn.clicked.connect(self.clear_path)
        control_layout.addWidget(clear_btn)
        
        layout.addLayout(control_layout)
        
        # Map display
        self.map_display = MapDisplayWidget(self.map_loader)
        layout.addWidget(self.map_display)
        
        # Resize window
        self.resize(900, 700)
    
    def on_pose_updated(self, x: float, y: float, theta: float):
        """Called when AMCL pose updates"""
        self.map_display.set_robot_pose(x, y, theta)
        
        # Replan if we have a goal
        if self.current_goal_id:
            self.replan_path(x, y)
    
    def on_spot_selected(self, spot_id: str):
        """Called when parking spot is selected"""
        if spot_id == "None":
            self.current_goal_id = None
            self.map_display.set_goal_pose(None, None)
            self.map_display.set_path([])
            return
        
        spot = self.spots_manager.get_spot(spot_id)
        if not spot:
            QMessageBox.warning(self, "Error", f"Spot {spot_id} not found")
            return
        
        self.current_goal_id = spot_id
        goal_x, goal_y = spot['x'], spot['y']
        self.map_display.set_goal_pose(goal_x, goal_y)
        
        # Plan initial path
        pose = self.amcl_sub.get_pose()
        if pose:
            self.replan_path(pose[0], pose[1])
        else:
            self.status_label.setText(f"Status: Waiting for AMCL pose to plan to {spot_id}...")
    
    def replan_path(self, robot_x: float, robot_y: float):
        """Replan path from current pose to goal"""
        if not self.current_goal_id:
            return
        
        goal_pos = self.spots_manager.get_spot_position(self.current_goal_id)
        if not goal_pos:
            return
        
        goal_x, goal_y = goal_pos
        
        # Convert to pixel coordinates
        start_px, start_py = self.map_loader.world_to_pixel(robot_x, robot_y)
        goal_px, goal_py = self.map_loader.world_to_pixel(goal_x, goal_y)
        
        # Plan path
        path_pixels = self.planner.plan_pixels((start_px, start_py), (goal_px, goal_py))
        
        if path_pixels:
            # Convert pixel path to world coordinates
            path_world = []
            for px, py in path_pixels:
                wx, wy = self.map_loader.pixel_to_world(px, py)
                path_world.append((wx, wy))
            
            self.map_display.set_path(path_world)
            self.status_label.setText(f"Status: Path planned to {self.current_goal_id} ({len(path_world)} waypoints)")
        else:
            self.map_display.set_path([])
            self.status_label.setText(f"Status: No path found to {self.current_goal_id}")
    
    def clear_path(self):
        """Clear current path and goal"""
        self.current_goal_id = None
        self.spot_combo.setCurrentIndex(0)
        self.map_display.set_goal_pose(None, None)
        self.map_display.set_path([])
        self.status_label.setText("Status: Path cleared")
    
    def closeEvent(self, event):
        """Cleanup on close"""
        self.ros_timer.stop()
        rclpy.shutdown()
        event.accept()


def main():
    """Main entry point"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Custom Navigation GUI')
    parser.add_argument('--map', required=True, help='Path to map YAML file')
    parser.add_argument('--spots', required=True, help='Path to parking_spots.yaml')
    args = parser.parse_args()
    
    app = QApplication(sys.argv)
    
    try:
        gui = NavigationGUI(args.map, args.spots)
        gui.show()
        sys.exit(app.exec_())
    except Exception as e:
        QMessageBox.critical(None, "Error", f"Failed to start GUI: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()

