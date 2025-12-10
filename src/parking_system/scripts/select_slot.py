#!/usr/bin/env python3
"""
Simple script to select a parking slot by number
Usage: select_slot.py <slot_number>
Example: select_slot.py 1
"""

import sys
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped
import math


def main():
    if len(sys.argv) < 2:
        print("Usage: select_slot.py <slot_number>")
        print("Available slots: 1, 2, 3, 4, 5")
        sys.exit(1)
    
    slot_num = int(sys.argv[1])
    
    if slot_num < 1 or slot_num > 5:
        print(f"Error: Slot number must be between 1 and 5, got {slot_num}")
        sys.exit(1)
    
    rclpy.init()
    
    # Create a simple node to publish goal
    node = Node('slot_selector')
    
    # Parking slots
    slots = {
        1: {'x': 1.0, 'y': 1.0, 'yaw': 0.0},
        2: {'x': 1.5, 'y': 1.0, 'yaw': 0.0},
        3: {'x': 2.0, 'y': 1.0, 'yaw': 0.0},
        4: {'x': 1.0, 'y': 1.5, 'yaw': 1.57},
        5: {'x': 1.5, 'y': 1.5, 'yaw': 1.57},
    }
    
    # Create goal pose
    goal_pub = node.create_publisher(PoseStamped, '/goal_pose', 10)
    
    slot = slots[slot_num]
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = node.get_clock().now().to_msg()
    goal_pose.pose.position.x = slot['x']
    goal_pose.pose.position.y = slot['y']
    goal_pose.pose.position.z = 0.0
    
    # Convert yaw to quaternion
    yaw = slot['yaw']
    goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
    goal_pose.pose.orientation.w = math.cos(yaw / 2.0)
    
    # Wait for publisher to be ready
    import time
    time.sleep(0.5)
    
    goal_pub.publish(goal_pose)
    print(f"Published goal to slot_{slot_num} at ({slot['x']}, {slot['y']})")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

