# ROS2 Sensor Fusion Template for AutoNexa Parking System
# Copy this to Raspberry Pi 5 when ready to integrate ROS2

## Installation Instructions

```bash
# Install ROS2 Humble on Raspberry Pi 5
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-core ros-humble-common-msgs ros-humble-geometry2 ros-humble-nav2-bringup

# Create workspace
mkdir -p ~/autonex_ws/src
cd ~/autonex_ws
colcon build --packages-select autonex_bringup

# Source setup
source ~/autonex_ws/install/setup.bash
```

## Package Structure

```
autonex_ws/
├── src/
│   └── autonex_bringup/
│       ├── launch/
│       │   └── bringup.launch.py
│       ├── autonex_bringup/
│       │   ├── __init__.py
│       │   ├── camera_aruco_node.py
│       │   ├── lidar_processor_node.py
│       │   ├── sensor_fusion_node.py
│       │   ├── path_planner_node.py
│       │   └── web_server_node.py
│       ├── config/
│       │   ├── camera_params.yaml
│       │   ├── lidar_params.yaml
│       │   └── fusion_params.yaml
│       ├── package.xml
│       └── setup.py
```

## Node Descriptions

### 1. camera_aruco_node.py
Publishes detected ArUco markers

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Int32MultiArray
import cv2
import cv2.aruco as aruco
import numpy as np

class CameraArucoNode(Node):
    def __init__(self):
        super().__init__('camera_aruco_node')
        
        # Declare parameters
        self.declare_parameter('marker_size_cm', 10.0)
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_width', 1280)
        self.declare_parameter('frame_height', 720)
        
        # Publishers
        self.marker_pose_pub = self.create_publisher(
            PoseStamped, '/camera/marker_pose', 10
        )
        self.marker_ids_pub = self.create_publisher(
            Int32MultiArray, '/camera/marker_ids', 10
        )
        
        # Camera setup
        self.cap = cv2.VideoCapture(
            self.get_parameter('camera_index').value
        )
        self.marker_size = self.get_parameter('marker_size_cm').value
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.detector_params = aruco.DetectorParameters()
        
        # Timer for processing
        self.timer = self.create_timer(0.033, self.process_frame)  # ~30 Hz
        self.get_logger().info('CameraArucoNode started')
    
    def process_frame(self):
        success, frame = self.cap.read()
        if not success:
            return
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, 
                                              parameters=self.detector_params)
        
        if ids is not None:
            # Publish marker IDs
            ids_msg = Int32MultiArray()
            ids_msg.data = ids.flatten().tolist()
            self.marker_ids_pub.publish(ids_msg)
            
            # For each marker, estimate pose and publish
            for i, marker_id in enumerate(ids.flatten()):
                pose_stamped = self.estimate_marker_pose(
                    corners[i], marker_id, frame.shape
                )
                if pose_stamped:
                    self.marker_pose_pub.publish(pose_stamped)
    
    def estimate_marker_pose(self, corner, marker_id, frame_shape):
        """Estimate 3D pose of marker using PnP"""
        # Simplified - in production use camera matrix calibration
        h, w = frame_shape[:2]
        focal_length = w
        cx, cy = w / 2, h / 2
        cam_mtx = np.array([
            [focal_length, 0, cx],
            [0, focal_length, cy],
            [0, 0, 1]
        ], dtype=np.float32)
        dist = np.zeros((4, 1))
        
        obj_points = np.array([
            [-self.marker_size / 2, self.marker_size / 2, 0],
            [self.marker_size / 2, self.marker_size / 2, 0],
            [self.marker_size / 2, -self.marker_size / 2, 0],
            [-self.marker_size / 2, -self.marker_size / 2, 0]
        ], dtype=np.float32)
        
        _, rvec, tvec = cv2.solvePnP(obj_points, corner, cam_mtx, dist)
        
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'camera_link'
        pose.pose.position.x = float(tvec[0][0]) / 100  # Convert to meters
        pose.pose.position.y = float(tvec[1][0]) / 100
        pose.pose.position.z = float(tvec[2][0]) / 100
        # TODO: Set orientation from rvec
        
        return pose
    
    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. lidar_processor_node.py
Processes 2D LiDAR data into occupancy grid

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
import numpy as np

class LidarProcessorNode(Node):
    def __init__(self):
        super().__init__('lidar_processor_node')
        
        # Declare parameters
        self.declare_parameter('testbed_width_m', 2.0)
        self.declare_parameter('testbed_height_m', 2.0)
        self.declare_parameter('grid_resolution', 0.02)  # 2cm cells
        
        # Subscriber to LiDAR
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10
        )
        
        # Publisher for occupancy grid
        self.grid_pub = self.create_publisher(
            OccupancyGrid, '/map', 10
        )
        
        # Initialize occupancy grid
        self.grid_width = int(2.0 / 0.02)  # 100x100 at 2cm resolution
        self.grid_height = int(2.0 / 0.02)
        self.occupancy_grid = np.zeros(
            (self.grid_height, self.grid_width), dtype=np.int8
        )
        
        self.get_logger().info('LidarProcessorNode started')
    
    def lidar_callback(self, msg):
        """Process LiDAR scan and update occupancy grid"""
        # Initialize grid with unknown (-1)
        self.occupancy_grid[:] = -1
        
        # Convert scan to grid
        grid_center_x = self.grid_width // 2
        grid_center_y = self.grid_height // 2
        
        for i, range_val in enumerate(msg.ranges):
            if range_val < msg.range_min or range_val > msg.range_max:
                continue
            
            # Convert range/angle to x/y
            angle = msg.angle_min + i * msg.angle_increment
            x = range_val * np.cos(angle)
            y = range_val * np.sin(angle)
            
            # Convert to grid coordinates
            grid_x = int(grid_center_x + x / 0.02)
            grid_y = int(grid_center_y + y / 0.02)
            
            # Mark as occupied
            if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
                self.occupancy_grid[grid_y, grid_x] = 100
        
        # Publish grid
        self.publish_occupancy_grid()
    
    def publish_occupancy_grid(self):
        grid_msg = OccupancyGrid()
        grid_msg.header.frame_id = 'map'
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Metadata
        grid_msg.info.resolution = 0.02
        grid_msg.info.width = self.grid_width
        grid_msg.info.height = self.grid_height
        grid_msg.info.origin.position.x = -1.0
        grid_msg.info.origin.position.y = -1.0
        
        # Data (convert 2D array to 1D)
        grid_msg.data = self.occupancy_grid.flatten().tolist()
        
        self.grid_pub.publish(grid_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. sensor_fusion_node.py
EKF fusion of odometry, camera, and LiDAR

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
from filterpy.kalman import ExtendedKalmanFilter as EKF

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
        
        # Initialize EKF
        self.ekf = EKF(dim_x=3, dim_z=3)  # State: x, y, theta
        self.ekf.x = np.array([[1.0], [1.0], [0.0]])  # Initial state
        self.ekf.P *= 1000  # Covariance
        self.ekf.R = np.eye(3) * 0.1  # Measurement noise
        self.ekf.Q = np.eye(3) * 0.01  # Process noise
        
        # Subscriptions
        self.create_subscription(
            TwistStamped, '/cmd_vel', self.odometry_callback, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped, '/camera/pose_estimate',
            self.camera_callback, 10
        )
        
        # Publisher for fused pose
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/robot_pose', 10
        )
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info('SensorFusionNode started')
    
    def odometry_callback(self, msg):
        # Predict step
        dt = 0.01  # 100 Hz assumed
        self.ekf.predict()
    
    def camera_callback(self, msg):
        # Update step with camera measurements
        z = np.array([
            [msg.pose.pose.position.x],
            [msg.pose.pose.position.y],
            [msg.pose.pose.orientation.z]
        ])
        self.ekf.update(z, self.camera_hx, self.camera_jacobian)
        self.publish_fused_pose()
    
    def camera_hx(self, x):
        return x
    
    def camera_jacobian(self, x):
        return np.eye(3)
    
    def publish_fused_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.pose.pose.position.x = float(self.ekf.x[0][0])
        msg.pose.pose.position.y = float(self.ekf.x[1][0])
        msg.pose.pose.orientation.z = float(self.ekf.x[2][0])
        
        self.pose_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4. web_server_node.py
HTTP bridge for mobile app

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from flask import Flask, jsonify, send_file
import threading
import numpy as np
import cv2
import io

class WebServerNode(Node):
    def __init__(self):
        super().__init__('web_server_node')
        
        self.robot_pose = None
        self.occupancy_grid = None
        self.parking_spots = []
        
        # Subscriptions
        self.create_subscription(
            PoseStamped, '/robot_pose', self.pose_callback, 10
        )
        self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )
        
        # Start Flask server in thread
        self.app = Flask(__name__)
        self.setup_routes()
        
        server_thread = threading.Thread(
            target=lambda: self.app.run(host='0.0.0.0', port=5000),
            daemon=True
        )
        server_thread.start()
        
        self.get_logger().info('WebServerNode started on http://0.0.0.0:5000')
    
    def pose_callback(self, msg):
        self.robot_pose = msg
    
    def map_callback(self, msg):
        self.occupancy_grid = msg
    
    def setup_routes(self):
        @self.app.route('/robot_pose')
        def get_robot_pose():
            if self.robot_pose is None:
                return {"error": "No pose data"}, 503
            
            return jsonify({
                'x_cm': self.robot_pose.pose.position.x * 100,
                'y_cm': self.robot_pose.pose.position.y * 100,
                'theta_deg': self.robot_pose.pose.orientation.z,
                'timestamp': self.robot_pose.header.stamp.sec,
            })
        
        @self.app.route('/map_image')
        def get_map_image():
            if self.occupancy_grid is None:
                return {"error": "No map data"}, 503
            
            # Convert OccupancyGrid to PNG
            grid_array = np.array(self.occupancy_grid.data, dtype=np.uint8).reshape(
                (self.occupancy_grid.info.height, self.occupancy_grid.info.width)
            )
            
            # Scale to visualization range
            vis_array = (255 - (grid_array * 2.55)).astype(np.uint8)
            
            _, buffer = cv2.imencode('.png', vis_array)
            return send_file(io.BytesIO(buffer.tobytes()), mimetype='image/png')

def main(args=None):
    rclpy.init(args=args)
    node = WebServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Launch File

```python
# bringup.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autonex_bringup',
            executable='camera_aruco_node',
            name='camera_aruco',
            parameters=[
                {'marker_size_cm': 10.0},
                {'camera_index': 0},
            ]
        ),
        Node(
            package='autonex_bringup',
            executable='lidar_processor_node',
            name='lidar_processor',
        ),
        Node(
            package='autonex_bringup',
            executable='sensor_fusion_node',
            name='sensor_fusion',
        ),
        Node(
            package='autonex_bringup',
            executable='web_server_node',
            name='web_server',
        ),
    ])
```

## Running ROS2 Stack

```bash
cd ~/autonex_ws
colcon build
source install/setup.bash

# Terminal 1: Launch all nodes
ros2 launch autonex_bringup bringup.launch.py

# Terminal 2: Monitor topics
ros2 topic list
ros2 topic echo /robot_pose
ros2 topic echo /map

# Terminal 3: Mobile app connects to http://localhost:5000
```

## Notes

- Placeholder: Actual LiDAR driver depends on hardware (RPLiDAR, VL53L0X, etc.)
- Camera calibration: Run calibration procedure to get accurate camera matrix
- EKF tuning: Adjust Q and R matrices based on sensor characteristics
- Performance: Monitor CPU usage on Pi5; may need to reduce publish rates

For questions, refer to ARCHITECTURE_RECOMMENDATIONS.md and INTEGRATION_GUIDE.md
