#!/usr/bin/env python3
"""
Simple road-constrained navigation without Nav2.
Features:
- Uses existing map + AMCL TF (map -> odom -> base_link)
- Uses road mask (/road_mask) to constrain navigation
- A* planner on combined static map + mask
- Publishes Path on /simple_plan
- Pure-pursuit follower publishes /cmd_vel
- Stops if obstacle closer than safety_radius from LiDAR

Launch with simple_navigation.launch.py.
"""

import math
import os
import yaml
import heapq
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.duration import Duration

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import tf2_ros
from tf2_ros import LookupException, ExtrapolationException, TransformException


def quaternion_from_yaw(yaw: float):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class SimpleRoadNavigator(Node):
    def __init__(self):
        super().__init__("simple_road_navigator")

        # Params
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("road_mask_topic", "/road_mask")
        self.declare_parameter("spots_file", "/home/autonexa/intelligent_parking_ws/maps/parking_spots.yaml")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("lookahead_dist", 0.35)
        self.declare_parameter("linear_speed", 0.18)
        self.declare_parameter("max_angular_speed", 0.8)
        self.declare_parameter("goal_tolerance", 0.08)
        self.declare_parameter("safety_radius", 0.30)

        # State
        self.map: Optional[OccupancyGrid] = None
        self.mask: Optional[OccupancyGrid] = None
        self.spots = self.load_spots(self.get_parameter("spots_file").value)
        self.current_path: List[Tuple[float, float]] = []
        self.goal_id: Optional[str] = None
        self.goal_pose: Optional[Tuple[float, float, float]] = None

        # TF buffer/listener
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # QoS
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        scan_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subs
        self.create_subscription(OccupancyGrid, self.get_parameter("map_topic").value, self.map_cb, map_qos)
        self.create_subscription(OccupancyGrid, self.get_parameter("road_mask_topic").value, self.mask_cb, map_qos)
        self.create_subscription(LaserScan, "/scan", self.scan_cb, scan_qos)
        self.create_subscription(String, "/navigate_to_spot", self.navigate_cmd_cb, 10)

        # Pubs
        self.plan_pub = self.create_publisher(Path, "/simple_plan", 1)
        self.cmd_pub = self.create_publisher(Twist, self.get_parameter("cmd_vel_topic").value, 1)
        self.status_pub = self.create_publisher(String, "/navigation_status", 10)

        # Timers
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        self.replan_timer = self.create_timer(1.0, self.try_plan)       # 1 Hz

        self.get_logger().info(f"Simple navigator ready. Spots: {list(self.spots.keys())}")

    def load_spots(self, path: str):
        if not os.path.exists(path):
            self.get_logger().warn(f"Spots file not found: {path}")
            return {}
        try:
            with open(path, "r") as f:
                data = yaml.safe_load(f) or {}
            return data.get("parking_spots", {})
        except Exception as e:
            self.get_logger().error(f"Failed to load spots: {e}")
            return {}

    # Callbacks
    def map_cb(self, msg: OccupancyGrid):
        self.map = msg

    def mask_cb(self, msg: OccupancyGrid):
        self.mask = msg

    def scan_cb(self, msg: LaserScan):
        # obstacle check in control loop; store latest
        self.last_scan = msg

    def navigate_cmd_cb(self, msg: String):
        spot_id = msg.data.strip()
        if spot_id not in self.spots:
            self.get_logger().warn(f"Unknown spot {spot_id}")
            return
        spot = self.spots[spot_id]
        self.goal_id = spot_id
        self.goal_pose = (float(spot["x"]), float(spot["y"]), float(spot.get("yaw", 0.0)))
        self.get_logger().info(f"Received goal: {spot_id} -> {self.goal_pose}")
        self.status_pub.publish(String(data=f"NAVIGATING: {spot_id}"))
        self.try_plan()

    # Planning
    def try_plan(self):
        if not (self.map and self.goal_pose):
            return
        start = self.lookup_pose()
        if start is None:
            return
        sx, sy, syaw = start
        gx, gy, gyaw = self.goal_pose
        path_cells = self.astar((sx, sy), (gx, gy))
        if not path_cells:
            self.get_logger().warn("Failed to plan path")
            self.status_pub.publish(String(data="REJECTED: no path"))
            return
        # Convert to world coords
        path_points = [self.cell_to_world(ix, iy) for ix, iy in path_cells]
        self.current_path = path_points
        self.publish_path(path_points)
        self.get_logger().info(f"Planned path with {len(path_points)} points")

    def publish_path(self, pts: List[Tuple[float, float]]):
        if not self.map or not pts:
            return
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in pts:
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.plan_pub.publish(path)

    # Control loop
    def control_loop(self):
        if not self.current_path or self.goal_pose is None:
            return
        pose = self.lookup_pose()
        if pose is None:
            return
        x, y, yaw = pose
        gx, gy, gyaw = self.goal_pose

        # Goal check
        if math.hypot(gx - x, gy - y) < self.get_parameter("goal_tolerance").value:
            self.stop_robot()
            self.status_pub.publish(String(data=f"ARRIVED: {self.goal_id or ''}"))
            self.get_logger().info("Arrived at goal")
            self.current_path = []
            return

        # Obstacle safety
        if hasattr(self, "last_scan") and self.is_obstacle_close(self.last_scan):
            self.stop_robot()
            self.status_pub.publish(String(data="STOPPED: obstacle"))
            return

        # Pure pursuit
        lookahead = self.get_parameter("lookahead_dist").value
        target = self.find_lookahead((x, y), lookahead)
        if target is None:
            self.stop_robot()
            return
        tx, ty = target
        angle_to_target = math.atan2(ty - y, tx - x)
        heading_error = self.normalize_angle(angle_to_target - yaw)

        cmd = Twist()
        cmd.linear.x = self.get_parameter("linear_speed").value
        cmd.angular.z = max(-self.get_parameter("max_angular_speed").value,
                            min(self.get_parameter("max_angular_speed").value,
                                2.0 * heading_error))
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def is_obstacle_close(self, scan: LaserScan) -> bool:
        min_range = min(scan.ranges) if scan.ranges else 10.0
        return min_range < self.get_parameter("safety_radius").value

    def find_lookahead(self, pose: Tuple[float, float], lookahead: float):
        if not self.current_path:
            return None
        px, py = pose
        # find first point farther than lookahead
        for cx, cy in self.current_path:
            if math.hypot(cx - px, cy - py) > lookahead:
                return (cx, cy)
        return self.current_path[-1]

    # A* on combined map+mask
    def astar(self, start_xy: Tuple[float, float], goal_xy: Tuple[float, float]):
        start_cell = self.world_to_cell(*start_xy)
        goal_cell = self.world_to_cell(*goal_xy)
        if not start_cell or not goal_cell:
            return None
        grid = self.get_combined_grid()
        if not grid:
            return None
        w, h, data = grid

        def idx(x, y): return y * w + x
        def is_free(x, y):
            if x < 0 or y < 0 or x >= w or y >= h:
                return False
            return data[idx(x, y)] == 0

        sx, sy = start_cell
        gx, gy = goal_cell
        if not (is_free(sx, sy) and is_free(gx, gy)):
            return None

        open_set = []
        heapq.heappush(open_set, (0, (sx, sy)))
        came_from = {}
        g_score = { (sx, sy): 0 }
        moves = [(-1,0),(1,0),(0,-1),(0,1)]

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == (gx, gy):
                # reconstruct
                path = []
                c = current
                while c in came_from:
                    path.append(c)
                    c = came_from[c]
                path.append((sx, sy))
                path.reverse()
                return path
            for dx, dy in moves:
                nx, ny = current[0]+dx, current[1]+dy
                if not is_free(nx, ny):
                    continue
                tentative = g_score[current] + 1
                if tentative < g_score.get((nx, ny), 1e9):
                    came_from[(nx, ny)] = current
                    g_score[(nx, ny)] = tentative
                    h = abs(nx-gx)+abs(ny-gy)
                    heapq.heappush(open_set, (tentative + h, (nx, ny)))
        return None

    def get_combined_grid(self):
        if not self.map:
            return None
        w = self.map.info.width
        h = self.map.info.height
        data = list(self.map.data)
        # Combine with mask: non-road (>=50) becomes occupied
        if self.mask and self.mask.info.width == w and self.mask.info.height == h:
            mdata = self.mask.data
            for i in range(len(data)):
                if mdata[i] >= 50:
                    data[i] = 100
        # Threshold map occupancy
        for i, v in enumerate(data):
            if v < 0:
                data[i] = 0
            elif v >= 50:
                data[i] = 100
            else:
                data[i] = 0
        return (w, h, data)

    def world_to_cell(self, x: float, y: float):
        if not self.map:
            return None
        origin = self.map.info.origin.position
        res = self.map.info.resolution
        cx = int((x - origin.x) / res)
        cy = int((y - origin.y) / res)
        return (cx, cy)

    def cell_to_world(self, cx: int, cy: int):
        origin = self.map.info.origin.position
        res = self.map.info.resolution
        return (origin.x + (cx + 0.5) * res,
                origin.y + (cy + 0.5) * res)

    def lookup_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            trans = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            tx = trans.transform.translation.x
            ty = trans.transform.translation.y
            q = trans.transform.rotation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            return (tx, ty, yaw)
        except (LookupException, ExtrapolationException, TransformException):
            return None

    @staticmethod
    def normalize_angle(a):
        while a > math.pi:
            a -= 2*math.pi
        while a < -math.pi:
            a += 2*math.pi
        return a


def main(args=None):
    rclpy.init(args=args)
    node = SimpleRoadNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

