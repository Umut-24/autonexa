#!/usr/bin/env python3
"""
Nav2 Activator - Automatically activates all Nav2 lifecycle nodes
This runs as a background node and keeps trying to activate nodes until successful
"""

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition
import time


class Nav2Activator(Node):
    def __init__(self):
        super().__init__('nav2_activator')
        
        self.nodes_to_activate = [
            'map_server',
            'amcl', 
            'controller_server',
            'planner_server',
            'smoother_server',
            'behavior_server',
            'bt_navigator',
            'waypoint_follower',
            'velocity_smoother',
            'collision_monitor'
        ]
        
        self.activated_nodes = set()
        
        self.get_logger().info('Nav2 Activator started - will activate nodes automatically')
        
        # Try to activate nodes every 2 seconds
        self.timer = self.create_timer(2.0, self.try_activate_nodes)
        
        # Initial delay to let nodes start
        time.sleep(3.0)
    
    def try_activate_nodes(self):
        """Try to activate all nodes that aren't activated yet"""
        all_activated = True
        
        for node_name in self.nodes_to_activate:
            if node_name in self.activated_nodes:
                continue
            
            all_activated = False
            
            # Check current state
            state = self.get_node_state(node_name)
            
            if state is None:
                # Node not found yet
                continue
            
            if state == 'active':
                self.activated_nodes.add(node_name)
                self.get_logger().info(f'✓ {node_name} is already active')
                continue
            
            if state == 'unconfigured':
                # Need to configure first
                if self.change_node_state(node_name, Transition.TRANSITION_CONFIGURE):
                    self.get_logger().info(f'→ Configured {node_name}')
                    time.sleep(0.5)
                    # Now activate
                    if self.change_node_state(node_name, Transition.TRANSITION_ACTIVATE):
                        self.get_logger().info(f'✓ Activated {node_name}')
                        self.activated_nodes.add(node_name)
            
            elif state == 'inactive':
                # Just need to activate
                if self.change_node_state(node_name, Transition.TRANSITION_ACTIVATE):
                    self.get_logger().info(f'✓ Activated {node_name}')
                    self.activated_nodes.add(node_name)
        
        if all_activated or len(self.activated_nodes) == len(self.nodes_to_activate):
            self.get_logger().info('All Nav2 nodes activated! Stopping activator.')
            self.timer.cancel()
    
    def get_node_state(self, node_name):
        """Get the current lifecycle state of a node"""
        try:
            client = self.create_client(GetState, f'/{node_name}/get_state')
            if not client.wait_for_service(timeout_sec=0.5):
                return None
            
            request = GetState.Request()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            
            if future.result() is not None:
                state_id = future.result().current_state.id
                state_names = {1: 'unconfigured', 2: 'inactive', 3: 'active', 4: 'finalized'}
                return state_names.get(state_id, 'unknown')
            return None
        except Exception:
            return None
    
    def change_node_state(self, node_name, transition_id):
        """Change the lifecycle state of a node"""
        try:
            client = self.create_client(ChangeState, f'/{node_name}/change_state')
            if not client.wait_for_service(timeout_sec=0.5):
                return False
            
            request = ChangeState.Request()
            request.transition.id = transition_id
            
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.result() is not None:
                return future.result().success
            return False
        except Exception:
            return False


def main(args=None):
    rclpy.init(args=args)
    node = Nav2Activator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

