#!/usr/bin/env python3
"""
Parking Slot Selector Node
Allows selecting parking slots and commanding the robot to navigate to them
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.srv import GetPlan
from std_srvs.srv import SetBool
from std_msgs.msg import String
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
        
        # Service client for path planning (Nav2 planner server)
        self.nav2_client = self.create_client(GetPlan, '/planner_server/compute_path_to_pose')
        
        # Try to wait for service, but don't block forever
        self.get_logger().info('Waiting for Nav2 planning service...')
        if not self.nav2_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('Nav2 planning service not available yet. Will retry when needed.')
        
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
            
            # Request path plan
            self.request_path_plan(goal_pose)
            
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
    
    def request_path_plan(self, goal_pose):
        """Request a path plan from Nav2"""
        # Wait for service if not available
        if not self.nav2_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Nav2 planning service still not available')
            return
        
        request = GetPlan.Request()
        request.start.header.frame_id = 'map'
        request.start.header.stamp = self.get_clock().now().to_msg()
        # Start pose will be filled by nav2 from current robot pose
        request.goal = goal_pose
        request.tolerance = 0.05  # 5cm tolerance as per requirements
        
        future = self.nav2_client.call_async(request)
        future.add_done_callback(self.plan_response_callback)
    
    def plan_response_callback(self, future):
        """Handle path plan response"""
        try:
            response = future.result()
            if response.plan.poses:
                self.get_logger().info(f'Path planned with {len(response.plan.poses)} waypoints')
                status_msg = String()
                status_msg.data = f'PATH_PLANNED: {len(response.plan.poses)} waypoints'
                self.status_pub.publish(status_msg)
            else:
                self.get_logger().warn('No path found to goal')
                status_msg = String()
                status_msg.data = 'PATH_FAILED'
                self.status_pub.publish(status_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to get path plan: {str(e)}')


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

