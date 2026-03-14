#!/usr/bin/env python3
"""
Record a standard rosbag topic set for Nav2 -> Pico control-chain analysis.

Usage:
  ros2 run parking_system record_control_chain_bag.py
  ros2 run parking_system record_control_chain_bag.py /home/autonexa/bags
"""

import argparse
import signal
import subprocess
import sys
from datetime import datetime
from pathlib import Path


TOPICS = [
    '/cmd_vel',
    '/cmd_vel_smoothed',
    '/cmd_vel_safe',
    '/pico/control_cmd',
    '/pico/enable',
    '/pico/heartbeat',
    '/pico/odom',
    '/pico/joint_feedback',
    '/odom',
    '/scan',
    '/tf',
    '/tf_static',
]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'output_root',
        nargs='?',
        default='~/control_chain_bags',
        help='Directory where bag folders are created.',
    )
    args = parser.parse_args()

    output_root = Path(args.output_root).expanduser()
    output_root.mkdir(parents=True, exist_ok=True)

    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_path = output_root / f'control_chain_{stamp}'

    cmd = ['ros2', 'bag', 'record', '-o', str(bag_path), '--topics', *TOPICS]
    print(f'[record_control_chain_bag] Writing bag to: {bag_path}', flush=True)
    print('[record_control_chain_bag] Press Ctrl+C to stop.', flush=True)

    proc = subprocess.Popen(cmd)

    def _forward(sig, _frame):
        if proc.poll() is None:
            proc.send_signal(sig)

    signal.signal(signal.SIGINT, _forward)
    signal.signal(signal.SIGTERM, _forward)

    return proc.wait()


if __name__ == '__main__':
    sys.exit(main())
