#!/usr/bin/env python3
"""
Diagnostic script to check LIDAR scan quality
Monitors scan topic and reports statistics
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import statistics


class ScanQualityDiagnostic(Node):
    def __init__(self):
        super().__init__('scan_quality_diagnostic')
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.filtered_scan_sub = self.create_subscription(
            LaserScan,
            '/scan_filtered',
            self.filtered_scan_callback,
            10
        )
        
        self.scan_count = 0
        self.filtered_scan_count = 0
        self.scan_stats = []
        self.filtered_scan_stats = []
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Scan Quality Diagnostic Started')
        self.get_logger().info('Monitoring /scan and /scan_filtered topics')
        self.get_logger().info('=' * 60)
        
        # Print statistics every 10 scans
        self.print_timer = self.create_timer(5.0, self.print_statistics)
    
    def scan_callback(self, msg):
        """Process raw scan"""
        self.scan_count += 1
        
        # Convert ranges to numpy array
        ranges = np.array(msg.ranges)
        
        # Filter out invalid readings (inf, nan, 0)
        valid_ranges = ranges[(ranges > msg.range_min) & (ranges < msg.range_max) & np.isfinite(ranges)]
        
        if len(valid_ranges) > 0:
            stats = {
                'count': self.scan_count,
                'valid_points': len(valid_ranges),
                'total_points': len(ranges),
                'min_range': float(np.min(valid_ranges)),
                'max_range': float(np.max(valid_ranges)),
                'mean_range': float(np.mean(valid_ranges)),
                'std_range': float(np.std(valid_ranges)),
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'angle_increment': msg.angle_increment,
                'time_increment': msg.time_increment,
                'scan_time': msg.scan_time,
            }
            self.scan_stats.append(stats)
            
            # Keep only last 20 scans
            if len(self.scan_stats) > 20:
                self.scan_stats.pop(0)
    
    def filtered_scan_callback(self, msg):
        """Process filtered scan"""
        self.filtered_scan_count += 1
        
        # Convert ranges to numpy array
        ranges = np.array(msg.ranges)
        
        # Filter out invalid readings
        valid_ranges = ranges[(ranges > msg.range_min) & (ranges < msg.range_max) & np.isfinite(ranges)]
        
        if len(valid_ranges) > 0:
            stats = {
                'count': self.filtered_scan_count,
                'valid_points': len(valid_ranges),
                'total_points': len(ranges),
                'min_range': float(np.min(valid_ranges)),
                'max_range': float(np.max(valid_ranges)),
                'mean_range': float(np.mean(valid_ranges)),
                'std_range': float(np.std(valid_ranges)),
            }
            self.filtered_scan_stats.append(stats)
            
            # Keep only last 20 scans
            if len(self.filtered_scan_stats) > 20:
                self.filtered_scan_stats.pop(0)
    
    def print_statistics(self):
        """Print scan statistics"""
        if len(self.scan_stats) == 0:
            self.get_logger().warn('No scan data received yet...')
            return
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('RAW SCAN STATISTICS (last 20 scans)')
        self.get_logger().info('=' * 60)
        
        latest = self.scan_stats[-1]
        self.get_logger().info(f'Scan Count: {latest["count"]}')
        self.get_logger().info(f'Valid Points: {latest["valid_points"]}/{latest["total_points"]} ({100*latest["valid_points"]/latest["total_points"]:.1f}%)')
        self.get_logger().info(f'Range: {latest["min_range"]:.3f}m - {latest["max_range"]:.3f}m')
        self.get_logger().info(f'Mean Range: {latest["mean_range"]:.3f}m')
        self.get_logger().info(f'Std Dev: {latest["std_range"]:.3f}m')
        self.get_logger().info(f'Angle Range: {np.degrees(latest["angle_min"]):.1f}° to {np.degrees(latest["angle_max"]):.1f}°')
        self.get_logger().info(f'Angle Increment: {np.degrees(latest["angle_increment"]):.3f}°')
        self.get_logger().info(f'Scan Time: {latest["scan_time"]:.3f}s')
        self.get_logger().info(f'Time Increment: {latest["time_increment"]:.3f}s')
        
        if latest["std_range"] > 0.5:
            self.get_logger().warn('⚠️  HIGH STD DEV: Scan has high variance (noisy)')
        if latest["valid_points"] / latest["total_points"] < 0.5:
            self.get_logger().warn('⚠️  LOW VALID POINTS: Many invalid readings')
        
        if len(self.filtered_scan_stats) > 0:
            self.get_logger().info('=' * 60)
            self.get_logger().info('FILTERED SCAN STATISTICS (last 20 scans)')
            self.get_logger().info('=' * 60)
            
            latest_filtered = self.filtered_scan_stats[-1]
            self.get_logger().info(f'Filtered Scan Count: {latest_filtered["count"]}')
            self.get_logger().info(f'Valid Points: {latest_filtered["valid_points"]}/{latest_filtered["total_points"]} ({100*latest_filtered["valid_points"]/latest_filtered["total_points"]:.1f}%)')
            self.get_logger().info(f'Range: {latest_filtered["min_range"]:.3f}m - {latest_filtered["max_range"]:.3f}m')
            self.get_logger().info(f'Mean Range: {latest_filtered["mean_range"]:.3f}m')
            self.get_logger().info(f'Std Dev: {latest_filtered["std_range"]:.3f}m')
            
            # Compare raw vs filtered
            reduction = latest["std_range"] - latest_filtered["std_range"]
            if reduction > 0:
                self.get_logger().info(f'✅ Noise Reduction: {reduction:.3f}m ({(100*reduction/latest["std_range"]):.1f}% improvement)')
            else:
                self.get_logger().warn('⚠️  Filter may not be working properly')
        
        self.get_logger().info('=' * 60)


def main():
    rclpy.init()
    node = ScanQualityDiagnostic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

