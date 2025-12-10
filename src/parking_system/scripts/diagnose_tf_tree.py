#!/usr/bin/env python3
"""
Diagnostic script to check TF tree connectivity and laser_scan_matcher status
"""

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import time


class TFTreeDiagnostic(Node):
    def __init__(self):
        super().__init__('tf_tree_diagnostic')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('TF TREE DIAGNOSTIC')
        self.get_logger().info('=' * 60)
        
        # Wait a bit for TF to initialize
        time.sleep(2.0)
        
        self.check_tf_chain()
        self.check_topics()
        self.check_nodes()
    
    def check_tf_chain(self):
        """Check each link in the TF chain"""
        self.get_logger().info('\n--- TF CHAIN CHECK ---')
        
        transforms_to_check = [
            ('map', 'odom', 'LINK #1: SLAM Toolbox'),
            ('odom', 'base_link', 'LINK #2: Laser Scan Matcher'),
            ('base_link', 'laser_link', 'LINK #3: URDF/robot_state_publisher'),
        ]
        
        for parent, child, description in transforms_to_check:
            try:
                transform = self.tf_buffer.lookup_transform(
                    parent, child, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                trans = transform.transform.translation
                rot = transform.transform.rotation
                
                # Check if identity
                is_identity = (
                    abs(trans.x) < 0.001 and abs(trans.y) < 0.001 and abs(trans.z) < 0.001 and
                    abs(rot.x) < 0.001 and abs(rot.y) < 0.001 and 
                    abs(rot.z) < 0.001 and abs(rot.w - 1.0) < 0.001
                )
                
                status = "✅ OK" if not is_identity or (parent == 'map' and child == 'odom') else "⚠️  IDENTITY"
                self.get_logger().info(f'{status} {description}')
                self.get_logger().info(f'   Transform: [{trans.x:.3f}, {trans.y:.3f}, {trans.z:.3f}]')
                
            except Exception as e:
                self.get_logger().error(f'❌ MISSING {description}')
                self.get_logger().error(f'   Error: {str(e)}')
    
    def check_topics(self):
        """Check if required topics exist"""
        self.get_logger().info('\n--- TOPIC CHECK ---')
        
        topics_to_check = ['/scan', '/scan_filtered', '/odom', '/tf', '/tf_static']
        
        for topic in topics_to_check:
            try:
                topic_info = self.get_topic_names_and_types()
                topic_exists = any(topic in str(t) for t in topic_info)
                
                if topic_exists:
                    # Try to get message count
                    try:
                        import subprocess
                        result = subprocess.run(
                            ['ros2', 'topic', 'hz', topic, '--window', '5'],
                            capture_output=True, timeout=3, text=True
                        )
                        if 'average rate' in result.stdout:
                            rate = result.stdout.split('average rate:')[1].split()[0]
                            self.get_logger().info(f'✅ {topic} - Publishing at {rate} Hz')
                        else:
                            self.get_logger().info(f'✅ {topic} - Exists but no recent messages')
                    except:
                        self.get_logger().info(f'✅ {topic} - Exists')
                else:
                    self.get_logger().error(f'❌ {topic} - NOT FOUND')
            except Exception as e:
                self.get_logger().warn(f'⚠️  {topic} - Check failed: {str(e)}')
    
    def check_nodes(self):
        """Check if required nodes are running"""
        self.get_logger().info('\n--- NODE CHECK ---')
        
        nodes_to_check = [
            'laser_scan_matcher',
            'sllidar_node',
            'slam_toolbox',
            'robot_state_publisher',
        ]
        
        for node_name in nodes_to_check:
            try:
                node_list = self.get_node_names()
                node_exists = any(node_name in node for node in node_list)
                
                if node_exists:
                    self.get_logger().info(f'✅ {node_name} - Running')
                else:
                    self.get_logger().error(f'❌ {node_name} - NOT RUNNING')
            except Exception as e:
                self.get_logger().warn(f'⚠️  {node_name} - Check failed: {str(e)}')
        
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('Diagnostic complete. Check the results above.')
        self.get_logger().info('=' * 60)


def main(args=None):
    rclpy.init(args=args)
    node = TFTreeDiagnostic()
    
    # Run diagnostic and exit
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

