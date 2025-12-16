#!/usr/bin/env python3
"""
Spot Recorder - Record parking spot positions using LiDAR/robot position
Move the LiDAR by hand to the desired spot, then call the service to record it

Usage:
  1. Launch navigation or localization first
  2. Move the LiDAR to the parking spot position
  3. Call: ros2 service call /record_spot parking_system/srv/RecordSpot "{spot_id: 'spot_1', description: 'My first spot'}"
  
  Or use the interactive terminal mode:
  ros2 run parking_system spot_recorder.py --interactive
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_srvs.srv import Trigger
from std_msgs.msg import String
import yaml
import os
import math
import sys
import threading


class SpotRecorder(Node):
    def __init__(self, spots_file=None, interactive=False):
        super().__init__('spot_recorder')
        
        # Default spots file
        self.spots_file = spots_file or '/home/autonexa/intelligent_parking_ws/maps/parking_spots.yaml'
        
        # Current robot pose (from AMCL or localization)
        self.current_pose = None
        
        # Subscribe to AMCL pose
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        
        # Also subscribe to alternative pose topics
        self.pose_sub2 = self.create_subscription(
            PoseStamped,
            '/slam_toolbox/pose',
            self.slam_pose_callback,
            10
        )
        
        # Service to record current position
        self.record_service = self.create_service(
            Trigger,
            'record_spot_trigger',
            self.record_spot_service
        )
        
        # Subscriber for spot recording commands (alternative to service)
        self.record_sub = self.create_subscription(
            String,
            '/record_spot_cmd',
            self.record_cmd_callback,
            10
        )
        
        # Publisher for feedback
        self.status_pub = self.create_publisher(String, '/spot_recorder/status', 10)
        
        # Load existing spots
        self.spots = self.load_spots()
        
        self.get_logger().info(f'Spot Recorder initialized')
        self.get_logger().info(f'Spots file: {self.spots_file}')
        self.get_logger().info(f'Loaded {len(self.spots.get("parking_spots", {}))} existing spots')
        
        # Interactive mode
        if interactive:
            self.start_interactive_mode()
    
    def load_spots(self):
        """Load existing spots from YAML file"""
        if os.path.exists(self.spots_file):
            try:
                with open(self.spots_file, 'r') as f:
                    data = yaml.safe_load(f) or {}
                    if 'parking_spots' not in data:
                        data['parking_spots'] = {}
                    if 'waypoints' not in data:
                        data['waypoints'] = {}
                    if 'roads' not in data:
                        data['roads'] = []
                    return data
            except Exception as e:
                self.get_logger().error(f'Failed to load spots: {e}')
        return {'parking_spots': {}, 'waypoints': {}, 'roads': []}
    
    def save_spots(self):
        """Save spots to YAML file"""
        try:
            # Add header comment
            header = """# Parking Spots Definition File
# Format: spot_id: {x, y, yaw (radians), description}
# These positions are in the MAP frame
# Use the spot_recorder tool to add new spots

"""
            with open(self.spots_file, 'w') as f:
                f.write(header)
                yaml.dump(self.spots, f, default_flow_style=False, sort_keys=False)
            self.get_logger().info(f'Saved spots to {self.spots_file}')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to save spots: {e}')
            return False
    
    def pose_callback(self, msg):
        """Handle AMCL pose updates"""
        self.current_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'yaw': self.quaternion_to_yaw(msg.pose.pose.orientation)
        }
    
    def slam_pose_callback(self, msg):
        """Handle SLAM pose updates"""
        if self.current_pose is None:  # Only use if AMCL not available
            self.current_pose = {
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'yaw': self.quaternion_to_yaw(msg.pose.orientation)
            }
    
    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def record_spot(self, spot_id, description=""):
        """Record current position as a parking spot"""
        if self.current_pose is None:
            return False, "No pose available. Is localization running?"
        
        # Add spot to dictionary
        self.spots['parking_spots'][spot_id] = {
            'x': round(self.current_pose['x'], 3),
            'y': round(self.current_pose['y'], 3),
            'yaw': round(self.current_pose['yaw'], 3),
            'description': description
        }
        
        # Save to file
        if self.save_spots():
            msg = f"Recorded spot '{spot_id}' at ({self.current_pose['x']:.3f}, {self.current_pose['y']:.3f})"
            self.get_logger().info(msg)
            
            status = String()
            status.data = f"RECORDED: {spot_id}"
            self.status_pub.publish(status)
            
            return True, msg
        else:
            return False, "Failed to save spots file"
    
    def record_waypoint(self, waypoint_id, description=""):
        """Record current position as a waypoint"""
        if self.current_pose is None:
            return False, "No pose available. Is localization running?"
        
        self.spots['waypoints'][waypoint_id] = {
            'x': round(self.current_pose['x'], 3),
            'y': round(self.current_pose['y'], 3),
            'description': description
        }
        
        if self.save_spots():
            msg = f"Recorded waypoint '{waypoint_id}' at ({self.current_pose['x']:.3f}, {self.current_pose['y']:.3f})"
            self.get_logger().info(msg)
            return True, msg
        return False, "Failed to save"
    
    def add_road(self, from_id, to_id):
        """Add a road connection between two waypoints/spots"""
        road = {'from': from_id, 'to': to_id}
        if road not in self.spots['roads']:
            self.spots['roads'].append(road)
            self.save_spots()
            return True, f"Added road: {from_id} -> {to_id}"
        return False, "Road already exists"
    
    def record_spot_service(self, request, response):
        """Service callback to record a spot"""
        # Generate automatic spot ID
        existing = list(self.spots.get('parking_spots', {}).keys())
        spot_num = 1
        while f'spot_{spot_num}' in existing:
            spot_num += 1
        spot_id = f'spot_{spot_num}'
        
        success, message = self.record_spot(spot_id)
        response.success = success
        response.message = message
        return response
    
    def record_cmd_callback(self, msg):
        """Handle recording commands via topic"""
        # Format: "spot_id:description" or just "spot_id"
        parts = msg.data.split(':', 1)
        spot_id = parts[0].strip()
        description = parts[1].strip() if len(parts) > 1 else ""
        
        success, message = self.record_spot(spot_id, description)
        self.get_logger().info(message)
    
    def list_spots(self):
        """List all recorded spots"""
        spots = self.spots.get('parking_spots', {})
        if not spots:
            return "No spots recorded yet"
        
        result = "=== Parking Spots ===\n"
        for spot_id, data in spots.items():
            result += f"  {spot_id}: ({data['x']:.3f}, {data['y']:.3f}) yaw={data['yaw']:.3f}"
            if data.get('description'):
                result += f" - {data['description']}"
            result += "\n"
        return result
    
    def delete_spot(self, spot_id):
        """Delete a parking spot"""
        if spot_id in self.spots.get('parking_spots', {}):
            del self.spots['parking_spots'][spot_id]
            self.save_spots()
            return True, f"Deleted spot '{spot_id}'"
        return False, f"Spot '{spot_id}' not found"
    
    def start_interactive_mode(self):
        """Start interactive terminal mode"""
        def interactive_loop():
            print("\n" + "="*50)
            print("SPOT RECORDER - Interactive Mode")
            print("="*50)
            print("Commands:")
            print("  r <id> [desc] - Record current position as spot")
            print("  w <id> [desc] - Record current position as waypoint")
            print("  road <from> <to> - Add road connection")
            print("  list          - List all spots")
            print("  delete <id>   - Delete a spot")
            print("  pos           - Show current position")
            print("  q             - Quit")
            print("="*50 + "\n")
            
            while rclpy.ok():
                try:
                    cmd = input(">>> ").strip()
                    if not cmd:
                        continue
                    
                    parts = cmd.split()
                    action = parts[0].lower()
                    
                    if action == 'q' or action == 'quit':
                        print("Exiting...")
                        rclpy.shutdown()
                        break
                    
                    elif action == 'r' or action == 'record':
                        if len(parts) < 2:
                            print("Usage: r <spot_id> [description]")
                            continue
                        spot_id = parts[1]
                        desc = ' '.join(parts[2:]) if len(parts) > 2 else ""
                        success, msg = self.record_spot(spot_id, desc)
                        print(msg)
                    
                    elif action == 'w' or action == 'waypoint':
                        if len(parts) < 2:
                            print("Usage: w <waypoint_id> [description]")
                            continue
                        wp_id = parts[1]
                        desc = ' '.join(parts[2:]) if len(parts) > 2 else ""
                        success, msg = self.record_waypoint(wp_id, desc)
                        print(msg)
                    
                    elif action == 'road':
                        if len(parts) < 3:
                            print("Usage: road <from_id> <to_id>")
                            continue
                        success, msg = self.add_road(parts[1], parts[2])
                        print(msg)
                    
                    elif action == 'list' or action == 'l':
                        print(self.list_spots())
                    
                    elif action == 'delete' or action == 'del':
                        if len(parts) < 2:
                            print("Usage: delete <spot_id>")
                            continue
                        success, msg = self.delete_spot(parts[1])
                        print(msg)
                    
                    elif action == 'pos' or action == 'position':
                        if self.current_pose:
                            print(f"Current position: x={self.current_pose['x']:.3f}, y={self.current_pose['y']:.3f}, yaw={self.current_pose['yaw']:.3f}")
                        else:
                            print("No position available. Is localization running?")
                    
                    else:
                        print(f"Unknown command: {action}")
                
                except EOFError:
                    break
                except Exception as e:
                    print(f"Error: {e}")
        
        # Start interactive thread
        thread = threading.Thread(target=interactive_loop, daemon=True)
        thread.start()


def main(args=None):
    rclpy.init(args=args)
    
    # Check for interactive mode
    interactive = '--interactive' in sys.argv or '-i' in sys.argv
    
    node = SpotRecorder(interactive=interactive)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

