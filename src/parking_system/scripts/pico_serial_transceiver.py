#!/usr/bin/env python3
"""
RPi5 Team-B Serial Transceiver

Bridges the physical serial connection to the Raspberry Pi Pico running the 
Hiwonder Ackermann firmware.

* Downlink (ROS2 -> Pico):
  Subscribes to `/pico/control_cmd_json`. Extracts `vx_mps` and `wz_radps`, 
  and serializes them to the Pico CLI format (`VEL <vx> <wz>\n`). 
  Handles ENABLE/DISABLE/STOP states based on the `enable` and `mode` flags.

* Uplink (Pico -> ROS2):
  Reads `TEL` telemetry lines from the Pico at 10Hz.
  Format: TEL <ms>,<L_pwm>,<R_pwm>,<steer_rad>,<enc_L>,<enc_R>,<odom_x>,<odom_y>,<odom_yaw>,<estop>,<timeout>
  Publishes the raw `enc_l, enc_r, steer_rad` values out to `/pico/joint_feedback`.
"""

import json
import serial
import traceback
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState


class PicoSerialTransceiver(Node):
    def __init__(self) -> None:
        super().__init__('pico_serial_transceiver')

        # ROS Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('control_cmd_json_topic', '/pico/control_cmd_json')
        self.declare_parameter('joint_feedback_topic', '/pico/joint_feedback')
        self.declare_parameter('read_timeout_s', 0.1)

        # Vehicle Config (needed to convert raw encoder counts to rad/s for joint_feedback)
        # JGB37-520R30-12: 1320 edges per wheel rev (according to config.h)
        self.declare_parameter('encoder_edges_per_rev', 1320)

        # Fetch Params
        self.port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baud_rate').value
        cmd_topic = self.get_parameter('control_cmd_json_topic').value
        fb_topic = self.get_parameter('joint_feedback_topic').value
        self.read_timeout = self.get_parameter('read_timeout_s').value
        self.edges_per_rev = self.get_parameter('encoder_edges_per_rev').value

        self.get_logger().info(f"Connecting to Pico on {self.port} at {self.baud} baud...")

        self.ser = None
        self.connect_serial()

        # State tracking
        self.was_enabled = False
        self.last_enc_l = 0
        self.last_enc_r = 0
        self.last_time_ms = 0

        # ROS2 Setup
        self.cmd_sub = self.create_subscription(String, cmd_topic, self.on_cmd_json, 10)
        self.fb_pub = self.create_publisher(JointState, fb_topic, 10)

        # Poll serial at ~50Hz (Pico sends TEL at 10Hz, but we want to flush buffers fast)
        self.timer = self.create_timer(0.02, self.poll_serial)

    def connect_serial(self) -> None:
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=self.read_timeout)
            self.get_logger().info(f"Successfully connected to Pico on {self.port}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port {self.port}: {e}")
            self.ser = None

    def send_cmd(self, cmd_str: str) -> None:
        if self.ser is None or not self.ser.is_open:
            return
        try:
            self.ser.write((cmd_str + '\n').encode('utf-8'))
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f"Serial write error: {e}. Reconnecting...")
            self.connect_serial()

    def on_cmd_json(self, msg: String) -> None:
        """Parses the bridge JSON output and sends ASCII CLI commands to the Pico."""
        if self.ser is None:
            return

        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON received on control_cmd_json topic.")
            return

        is_enabled = cmd.get("enable", False)
        mode = cmd.get("mode", "SAFE_STOP")
        vx = cmd.get("vx_mps", 0.0)
        wz = cmd.get("wz_radps", 0.0)

        # Handle State Transitions
        if is_enabled and not self.was_enabled:
            self.send_cmd("ENABLE")
            self.was_enabled = True
        elif not is_enabled and self.was_enabled:
            if mode == "ESTOP":
                self.send_cmd("ESTOP")
            else:
                self.send_cmd("DISABLE")
            self.was_enabled = False

        # Only send velocity if enabled
        if is_enabled:
            self.send_cmd(f"VEL {vx:.3f} {wz:.3f}")

    def poll_serial(self) -> None:
        """Reads lines from the Pico and publishes telemetry back to ROS2."""
        if self.ser is None or not self.ser.is_open:
            # Try to reconnect occasionally if disconnected
            if self.get_clock().now().nanoseconds % 2000000000 < 20000000:
                self.connect_serial()
            return

        try:
            while self.ser.in_waiting > 0:
                try:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith("TEL "):
                        self.parse_telemetry(line[4:])
                except Exception as e:
                    self.get_logger().error(f"Error parsing line: {e}")
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f"Serial read error: {e}. Reconnecting...")
            self.connect_serial()

    def parse_telemetry(self, data_str: str) -> None:
        """
        Format: <ms>,<L_pwm>,<R_pwm>,<steer_rad>,<enc_L>,<enc_R>,<odom_x>,<odom_y>,<odom_yaw>,<estop>,<timeout>
        Publishes JointState to feed the pico_joint_feedback_to_odom node.
        """
        parts = data_str.split(',')
        if len(parts) < 11:
            return

        try:
            ms = int(parts[0])
            steer_rad = float(parts[3])
            enc_l = int(parts[4])
            enc_r = int(parts[5])
            
            # Note: We are ignoring the on-board Pico odom calc (parts 6-8) 
            # and passing raw encs to ROS for tf tree accuracy.

            # Calculate velocities (rad/s)
            dt_s = (ms - self.last_time_ms) / 1000.0
            if dt_s <= 0 or self.last_time_ms == 0:
                self.last_time_ms = ms
                self.last_enc_l = enc_l
                self.last_enc_r = enc_r
                return

            dl = enc_l - self.last_enc_l
            dr = enc_r - self.last_enc_r
            
            self.last_time_ms = ms
            self.last_enc_l = enc_l
            self.last_enc_r = enc_r

            # Convert ticks to revs, then revs to rad/s
            vl_rads = (dl / self.edges_per_rev) * 2.0 * math.pi / dt_s
            vr_rads = (dr / self.edges_per_rev) * 2.0 * math.pi / dt_s

            # Publish JointState mapping
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.header.frame_id = "base_link"
            
            js.name = ['left_wheel_joint', 'right_wheel_joint', 'steering_joint']
            js.position = [0.0, 0.0, steer_rad] # wheels don't need pos, steer does
            js.velocity = [vl_rads, vr_rads, 0.0]

            self.fb_pub.publish(js)

        except ValueError as e:
            self.get_logger().debug(f"Failed to parse TEL string: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PicoSerialTransceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
