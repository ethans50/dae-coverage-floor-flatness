#!/usr/bin/env python3
# surface_profiling/test/check_imu_noise.py
"""
Diagnostic script to run before implementing IMU fusion for real.

What it checks:
  1. Is the /imu topic actually being published (message type, rate)?
  2. Does the TurtleBot3 OpenCR firmware already fill in an orientation
     estimate, or does it only send raw accel/gyro? (If the former, it can be
     used directly with no separate AHRS filter.)
  3. Mean/std of roll/pitch while the robot sits fully still on a flat floor
     (the stationary noise floor - if this exceeds the actual tilt caused by
     the target defect size (mm-cm scale z), IMU-based correction could add
     more noise than signal.)

Usage:
  1. Park the robot fully still on a flat, vibration-free floor (do not drive).
  2. Run: python3 check_imu_noise.py --duration 20
  3. Share the printed topic status, whether orientation is filled, and the
     roll/pitch std as-is - they determine the IMU fusion architecture
     (which filter, whether to trust it).
"""

import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import tf_transformations


class ImuNoiseChecker(Node):
    def __init__(self, topic):
        super().__init__('check_imu_noise')
        self.rows = []  # (t, roll, pitch, ax, ay, az, gx, gy, gz)
        self.orientation_filled_count = 0
        self.msg_count = 0
        self.t_first = None
        self.t_last = None
        self.create_subscription(Imu, topic, self._cb, 50)

    def _cb(self, msg: Imu):
        now = time.time()
        if self.t_first is None:
            self.t_first = now
        self.t_last = now
        self.msg_count += 1

        q = msg.orientation
        # 펌웨어가 orientation을 안 채우면 관례상 (0,0,0,0)으로 옴 - 단위 쿼터니언(w=1 포함)이 아님.
        orientation_filled = not (q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0)
        if orientation_filled:
            self.orientation_filled_count += 1
            roll, pitch, _ = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        else:
            roll, pitch = float('nan'), float('nan')

        a, g = msg.linear_acceleration, msg.angular_velocity
        self.rows.append((now, roll, pitch, a.x, a.y, a.z, g.x, g.y, g.z))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--topic', default='/imu')
    ap.add_argument('--duration', type=float, default=20.0, help='How long to log while stationary (s)')
    args = ap.parse_args()

    rclpy.init()
    node = ImuNoiseChecker(args.topic)

    print(f"[*] Subscribing to '{args.topic}' - waiting {args.duration}s (keep the robot fully still)")
    t0 = time.time()
    while time.time() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.1)
    rclpy.shutdown()

    if node.msg_count == 0:
        print(f"[-] Received no messages at all on '{args.topic}'.")
        print("    Check: 1) is the topic name different (see `ros2 topic list`),")
        print("           2) is OpenCR/turtlebot3_node running,")
        print("           3) is this machine on the same ROS2 domain/network as the robot")
        return

    rate_hz = node.msg_count / (node.t_last - node.t_first) if node.t_last > node.t_first else 0.0
    print(f"\n[*] Messages received: {node.msg_count}, average rate: {rate_hz:.1f} Hz")
    print(f"[*] Fraction of messages with orientation filled: "
          f"{node.orientation_filled_count}/{node.msg_count} "
          f"({100 * node.orientation_filled_count / node.msg_count:.0f}%)")

    rows = np.array(node.rows)
    roll_deg = np.degrees(rows[:, 1])
    pitch_deg = np.degrees(rows[:, 2])
    ax, ay, az = rows[:, 3], rows[:, 4], rows[:, 5]
    gx, gy, gz = rows[:, 6], rows[:, 7], rows[:, 8]

    if node.orientation_filled_count > 0:
        print("\n[*] Firmware-orientation-based roll/pitch (stationary noise floor):")
        print(f"    roll : mean {np.nanmean(roll_deg):+.3f} deg, std {np.nanstd(roll_deg):.4f} deg")
        print(f"    pitch: mean {np.nanmean(pitch_deg):+.3f} deg, std {np.nanstd(pitch_deg):.4f} deg")
    else:
        print("\n[!] orientation field is empty - firmware sends raw accel/gyro only, no pose estimate.")
        print("    A separate AHRS filter (e.g. imu_filter_madgwick) is needed to get roll/pitch.")

    print("\n[*] Reference raw sensor std (checks vibration/noise level):")
    print(f"    linear_acceleration std (m/s^2): x={ax.std():.4f} y={ay.std():.4f} z={az.std():.4f}")
    print(f"    angular_velocity    std (rad/s): x={gx.std():.4f} y={gy.std():.4f} z={gz.std():.4f}")

    print("\n[*] Rule of thumb: converting a target defect height (e.g. 1cm) over the robot's")
    print("    wheelbase (TB3 Waffle ~0.287m) into a tilt gives atan(0.01/0.287) ~= 2.0 deg. If the")
    print("    roll/pitch std above is at or above 1/5 of that (0.4 deg), IMU-based correction may")
    print("    add more noise than signal - judge carefully.")


if __name__ == '__main__':
    main()
