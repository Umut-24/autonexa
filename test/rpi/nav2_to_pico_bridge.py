#!/usr/bin/env python3
"""ROS2 Nav2 -> newline JSON bridge for Pico serial control."""

import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
import serial


def yaw_from_quat_z_w(z: float, w: float) -> float:
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class Nav2ToPicoBridge(Node):
    def __init__(self):
        super().__init__("nav2_to_pico_bridge")

        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("send_hz", 20.0)

        port = self.get_parameter("serial_port").value
        baud = int(self.get_parameter("baud").value)
        send_hz = float(self.get_parameter("send_hz").value)

        self.ser = serial.Serial(port, baud, timeout=0.01)
        self.timer = self.create_timer(1.0 / send_hz, self.tick)

        self.get_logger().info(f"Serial connected: {port} @ {baud}")

        self.seq = 0
        self.state = "IDLE"

        self.v_lin = 0.0
        self.v_ang = 0.0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_yaw = 0.0
        self.has_goal = False

        self.progress_pct = 0

        # Replace with real obstacle processing when available
        self.front_m = 9.9
        self.left_m = 9.9
        self.right_m = 9.9

        self.loc_ok = True
        self.planner_ok = True
        self.controller_ok = True

        self.create_subscription(Twist, "/cmd_vel", self.cb_cmd_vel, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.cb_pose, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.cb_goal_pose, 10)
        self.create_subscription(Path, "/plan", self.cb_plan, 10)

    def cb_cmd_vel(self, msg: Twist):
        self.v_lin = float(msg.linear.x)
        self.v_ang = float(msg.angular.z)
        self.state = "TRACKING_PATH"

    def cb_pose(self, msg: PoseWithCovarianceStamped):
        pose = msg.pose.pose
        self.x = float(pose.position.x)
        self.y = float(pose.position.y)
        self.yaw = yaw_from_quat_z_w(float(pose.orientation.z), float(pose.orientation.w))
        self.loc_ok = True

    def cb_goal_pose(self, msg: PoseStamped):
        pose = msg.pose
        self.goal_x = float(pose.position.x)
        self.goal_y = float(pose.position.y)
        self.goal_yaw = yaw_from_quat_z_w(float(pose.orientation.z), float(pose.orientation.w))
        self.has_goal = True
        self.state = "TRACKING_PATH"

    def cb_plan(self, msg: Path):
        # Simple progress proxy by remaining path points.
        n = len(msg.poses)
        self.progress_pct = max(0, min(100, 100 - n))

    def build_packet(self):
        dist = 0.0
        heading_err = 0.0

        if self.has_goal:
            dx = self.goal_x - self.x
            dy = self.goal_y - self.y
            dist = math.sqrt(dx * dx + dy * dy)
            goal_heading = math.atan2(dy, dx)
            heading_err = wrap_pi(goal_heading - self.yaw)

        speed = abs(self.v_lin)
        eta = int(dist / speed) if speed > 0.05 else 999

        if self.has_goal and dist < 0.15:
            state = "GOAL_REACHED"
            v_lin = 0.0
            v_ang = 0.0
        else:
            state = self.state
            v_lin = self.v_lin
            v_ang = self.v_ang

        self.seq += 1
        return {
            "t_ms": int(time.time() * 1000),
            "state": state,
            "v_lin": float(v_lin),
            "v_ang": float(v_ang),
            "dist_to_goal_m": float(dist),
            "heading_err_rad": float(heading_err),
            "progress_pct": int(self.progress_pct),
            "eta_s": int(eta),
            "obstacle": {
                "front_m": float(self.front_m),
                "left_m": float(self.left_m),
                "right_m": float(self.right_m),
                "emergency_stop": False,
            },
            "health": {
                "loc_ok": bool(self.loc_ok),
                "planner_ok": bool(self.planner_ok),
                "controller_ok": bool(self.controller_ok),
            },
            "seq": self.seq,
        }

    def tick(self):
        pkt = self.build_packet()
        self.ser.write((json.dumps(pkt) + "\n").encode("utf-8"))


def main():
    rclpy.init()
    node = Nav2ToPicoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
