#!/usr/bin/env python3
"""
Test script to select parking slots via service call
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class ParkingSlotTester(Node):
    def __init__(self):
        super().__init__('parking_slot_tester')
        
        self.client = self.create_client(SetBool, 'select_parking_slot')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
    
    def select_slot(self, slot_index):
        """Select a parking slot (0 for slot_1, 1 for slot_2, etc.)"""
        request = SetBool.Request()
        request.data = (slot_index == 0)  # True for slot_1, False for slot_2
        
        self.get_logger().info(f'Sending request to select parking slot {slot_index + 1}...')
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Response: Success={response.success}, Message={response.message}')
            return response.success
        else:
            self.get_logger().error('Service call failed')
            return False


def main(args=None):
    rclpy.init(args=args)
    
    tester = ParkingSlotTester()
    
    import sys
    if len(sys.argv) > 1:
        slot_index = int(sys.argv[1]) - 1  # Convert to 0-indexed
        tester.select_slot(slot_index)
    else:
        print("Usage: test_parking_slot_selection.py <slot_number>")
        print("Example: test_parking_slot_selection.py 1  (selects slot_1)")
    
    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

