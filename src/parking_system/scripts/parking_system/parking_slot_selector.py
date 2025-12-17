#!/usr/bin/env python3
"""
Parking Slot Selector Node
Allows selecting parking slots and commanding the robot to navigate to them
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose
from std_srvs.srv import SetBool
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
import math


class ParkingSlotSelector(Node):
    def __init__(self):
        super().__init__('parking_slot_selector')
        
        # Parking slots definition (in map frame)
        # Users can define parking slots here or load from config
        self.parking_slots = {
            'slot_1': {'x': 1.0, 'y': 1.0, 'yaw': 0.0},
            'slot_2': {'x': 1.5, 'y': 1.0, 'yaw': 0.0},
            'slot_3': {'x': 2.0, 'y': 1.0, 'yaw': 0.0},
            'slot_4': {'x': 1.0, 'y': 1.5, 'yaw': 1.57},
            'slot_5': {'x': 1.5, 'y': 1.5, 'yaw': 1.57},
        }
        
        # Service to select a parking slot
        self.select_slot_service = self.create_service(
            SetBool,
            'select_parking_slot',
            self.select_slot_callback
        )
        
        # Publisher for goal pose
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Action client for navigation (Nav2 BT Navigator)
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Status publisher
        self.status_pub = self.create_publisher(String, '/parking_status', 10)
        
        self.current_slot = None
        
        self.get_logger().info('Parking Slot Selector initialized')
        self.get_logger().info(f'Available slots: {list(self.parking_slots.keys())}')
        
    def yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion"""
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return [0.0, 0.0, qz, qw]
    
    def select_slot_callback(self, request, response):
        """Handle parking slot selection requests"""
        # For now, we'll use SetBool where data field contains slot name
        # In a more complete implementation, use a custom service
        
        slot_name = 'slot_1'  # Default, can be extended
        
        if request.data:  # If True, use slot_1, else slot_2
            slot_name = 'slot_1'
        else:
            slot_name = 'slot_2'
            
        if slot_name in self.parking_slots:
            slot = self.parking_slots[slot_name]
            goal_pose = self.create_goal_pose(slot)
            
            # Publish goal pose
            self.goal_pub.publish(goal_pose)
            
            # Send Nav2 goal
            self.send_nav_goal(goal_pose)
            
            self.current_slot = slot_name
            response.success = True
            response.message = f'Navigating to {slot_name}'
            
            status_msg = String()
            status_msg.data = f'SELECTED: {slot_name}'
            self.status_pub.publish(status_msg)
            
            self.get_logger().info(f'Selected parking slot: {slot_name} at ({slot["x"]}, {slot["y"]})')
        else:
            response.success = False
            response.message = f'Invalid slot: {slot_name}'
            
        return response
    
    def create_goal_pose(self, slot):
        """Create a PoseStamped message for the parking slot"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = slot['x']
        pose.pose.position.y = slot['y']
        pose.pose.position.z = 0.0
        
        quat = self.yaw_to_quaternion(slot['yaw'])
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        
        return pose
    
    def send_nav_goal(self, goal_pose: PoseStamped):
        """Send a navigation goal to Nav2."""
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server (navigate_to_pose) not available')
            self.status_pub.publish(String(data='ERROR: Nav2 navigate_to_pose unavailable'))
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info(
            f'Sending Nav2 goal to ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})'
        )
        self.status_pub.publish(String(data='NAV_GOAL_SENT'))

        future = self.nav_to_pose_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 goal rejected')
            self.status_pub.publish(String(data='ERROR: NAV_GOAL_REJECTED'))
            return

        self.get_logger().info('Nav2 goal accepted')
        self.status_pub.publish(String(data='NAV_GOAL_ACCEPTED'))

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        status = future.result().status
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Nav2 goal succeeded')
            self.status_pub.publish(String(data='NAV_SUCCEEDED'))
        elif status == 5:  # CANCELED
            self.get_logger().warn('Nav2 goal canceled')
            self.status_pub.publish(String(data='NAV_CANCELED'))
        elif status == 6:  # ABORTED
            self.get_logger().error('Nav2 goal aborted')
            self.status_pub.publish(String(data='NAV_ABORTED'))
        else:
            self.get_logger().warn(f'Nav2 goal finished with status {status}')
            self.status_pub.publish(String(data=f'NAV_STATUS: {status}'))


def main(args=None):
    rclpy.init(args=args)
    node = ParkingSlotSelector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

