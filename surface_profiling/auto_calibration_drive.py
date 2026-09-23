#!/usr/bin/env python3
# surface_profiling/auto_calibration_drive.py
"""
Automatically collects four-heading (0/90/180/270 deg) LiDAR mount calibration
data in a single command.

For each heading: start capture -> reverse -> stop -> forward -> stop ->
reverse (back to start) -> stop -> stop capture. Between headings, the robot
rotates roughly 90 deg open-loop, then closed-loop fine-alignment re-measures
the front wall angle with the LiDAR until it is square. Drive/rotate commands
are published over SSH from the robot (Jetson) - matching where cmd_vel is
published in the manual procedure. This script itself calls the capture
services directly from the laptop.

Before using this for real:
  1. Run with --detect-only first and nudge the robot by hand to confirm the
     wall-angle detector gives a sane sign in this room.
  2. Run with --dry-run to walk through the whole procedure (capture
     start/stop, alignment decisions) without moving.
  3. Confirm there is actually clear space to drive --reverse-m/--forward-m
     in each heading (no obstacle avoidance here).

Example:
  python3 auto_calibration_drive.py --host 192.168.0.10 --password 1234 \
    --reverse-m 1.5 --forward-m 3.0
"""

import argparse
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from utils.ssh_robot import JetsonSession  # noqa: E402
from utils.wall_alignment import estimate_front_wall_angle_error  # noqa: E402


class CalibrationDriver(Node):
    def __init__(self, args):
        super().__init__('auto_calibration_drive')
        self.args = args
        self.latest_points = None
        self.latest_stamp = None
        self.create_subscription(PointCloud2, '/velodyne_points', self._pc_cb, 5)

        self.start_cli = self.create_client(Trigger, '/surface_profiling/start_waypoint_capture')
        self.stop_cli = self.create_client(Trigger, '/surface_profiling/stop_waypoint_capture')
        self.finish_cli = self.create_client(Trigger, '/surface_profiling/stop_collection_success')
        if not args.detect_only:
            # --detect-only는 포인트클라우드만 보고 캡처 서비스는 전혀 안 쓰므로,
            # 이 대기는 실제 주행(캡처 시작/종료가 필요한 경우)에만 함 - 그래야
            # --detect-only가 surface_profiling 노드 없이도 최소 의존성으로 동작함.
            for cli, name in [(self.start_cli, 'start_waypoint_capture'),
                               (self.stop_cli, 'stop_waypoint_capture'),
                               (self.finish_cli, 'stop_collection_success')]:
                if not cli.wait_for_service(timeout_sec=10.0):
                    raise RuntimeError(f"[-] Failed to connect to surface_profiling's {name} service - "
                                        "is the node running?")

    def _pc_cb(self, msg):
        raw = pc2.read_points(msg, skip_nans=True, field_names=('x', 'y', 'z'))
        self.latest_points = np.array([(p[0], p[1], p[2]) for p in raw], dtype=np.float32)
        self.latest_stamp = time.time()

    def wait_fresh_scan(self, max_wait=3.0):
        """지금 시점 이후 새로 들어오는 스캔 하나를 기다려 반환함(직전 캐시가 아님)."""
        t0 = time.time()
        seen_before = self.latest_stamp
        while time.time() - t0 < max_wait:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_stamp is not None and self.latest_stamp != seen_before:
                return self.latest_points
        return self.latest_points

    def call_trigger(self, client, label):
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        result = future.result()
        if result is None or not result.success:
            print(f"[!] {label} service call failed: {result}")
        else:
            print(f"[*] {label}: {result.message}")

    def detect_wall_angle(self):
        pts = self.wait_fresh_scan()
        if pts is None or len(pts) == 0:
            return None, 0
        return estimate_front_wall_angle_error(
            pts, fov_deg=self.args.wall_fov_deg, r_min=self.args.wall_r_min,
            r_max=self.args.wall_r_max, min_points=self.args.wall_min_points)

    def align_to_wall(self, jetson):
        """정면 벽에 수직으로 서도록 폐루프로 미세 회전함."""
        deadline = time.time() + self.args.align_timeout_s
        tol_rad = np.radians(self.args.align_tolerance_deg)
        for it in range(self.args.align_max_iters):
            if time.time() > deadline:
                print(f"[!] Alignment timed out ({self.args.align_timeout_s}s) - proceeding as-is")
                return False
            angle_error, n = self.detect_wall_angle()
            if angle_error is None:
                print(f"[!] Not enough wall points (n={n}) - retrying")
                time.sleep(0.5)
                continue
            print(f"[*] [align {it + 1}] wall angle error = {np.degrees(angle_error):+.2f} deg (n={n})")
            if abs(angle_error) < tol_rad:
                print(f"[*] Alignment done (error {np.degrees(angle_error):.3f} deg < "
                      f"tolerance {self.args.align_tolerance_deg} deg)")
                return True
            step = float(np.clip(angle_error, -self.args.align_max_step_rad, self.args.align_max_step_rad))
            duration = abs(step) / self.args.rotate_speed
            if not self.args.dry_run:
                jetson.rotate(np.sign(step) * self.args.rotate_speed, duration)
            else:
                print(f"[dry-run] skipping rotation of {np.degrees(step):+.2f} deg")
            time.sleep(0.3)  # 회전 후 새 스캔이 들어올 시간
        print("[!] Reached max iterations - proceeding as-is")
        return False

    def run_heading_pass(self, jetson, heading_idx):
        print(f"\n=== Heading {heading_idx + 1}/4: reverse -> forward -> reverse (back to start) capture ===")
        self.call_trigger(self.start_cli, 'start_waypoint_capture')
        time.sleep(1.0)  # 가감속 안정화 대기(문서 권장 절차와 동일)
        if not self.args.dry_run:
            jetson.drive_distance(-self.args.reverse_m, self.args.speed)
            jetson.drive_distance(self.args.forward_m, self.args.speed)
            jetson.drive_distance(-self.args.reverse_m, self.args.speed)
        else:
            print(f"[dry-run] skipping reverse {self.args.reverse_m}m -> forward {self.args.forward_m}m "
                  f"-> reverse {self.args.reverse_m}m")
        time.sleep(1.0)
        self.call_trigger(self.stop_cli, 'stop_waypoint_capture')

    def run(self):
        jetson = JetsonSession(self.args.host, self.args.user, self.args.password)
        print("[*] Connected to Jetson over SSH - syncing clock")
        jetson.sync_clock()

        for i in range(4):
            self.run_heading_pass(jetson, i)
            if i < 3:
                print(f"\n=== Heading {i + 1} -> {i + 2}: rotating ~90 deg (open-loop) ===")
                coarse_duration = np.radians(90) / self.args.rotate_speed
                if not self.args.dry_run:
                    jetson.rotate(self.args.rotate_speed, coarse_duration)
                else:
                    print("[dry-run] skipping 90 deg rotation")
                print("=== Fine-aligning to the front wall (closed-loop) ===")
                self.align_to_wall(jetson)

        self.call_trigger(self.finish_cli, 'stop_collection_success')
        jetson.stop()
        jetson.close()
        print("\n[*] Four-heading calibration collection complete. Analyze frames_*.npz with analyze_z_bias.py.")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default=os.environ.get('ROBOT_HOST'),
                     help='Jetson IP. Falls back to the ROBOT_HOST environment variable.')
    ap.add_argument('--user', default='waffle')
    ap.add_argument('--password', default=os.environ.get('SSH_PASSWORD'),
                     help='SSH password shared by the Jetson and laptop. Falls back to SSH_PASSWORD.')
    ap.add_argument('--reverse-m', type=float, default=1.5, help='Reverse distance for each leg of the back-and-forth (m)')
    ap.add_argument('--forward-m', type=float, default=3.0, help='Forward distance for the middle leg of the back-and-forth (m)')
    ap.add_argument('--speed', type=float, default=0.16, help='m/s, same default as coverage_speed_limit_mps')
    ap.add_argument('--rotate-speed', type=float, default=0.3, help='rad/s, rotation angular speed')
    ap.add_argument('--align-tolerance-deg', type=float, default=0.3)
    ap.add_argument('--align-timeout-s', type=float, default=30.0)
    ap.add_argument('--align-max-iters', type=int, default=15)
    ap.add_argument('--align-max-step-rad', type=float, default=np.radians(15),
                     help='Max angle to correct in a single rotation step (rad) - avoids one big overcorrection')
    ap.add_argument('--wall-r-max', type=float, default=5.0,
                     help='Max wall distance counted for angle detection (m) - set higher than the '
                          'wall distance scan_room_for_calibration.py reported for this room')
    ap.add_argument('--wall-r-min', type=float, default=0.5, help='Min wall distance counted (m)')
    ap.add_argument('--wall-fov-deg', type=float, default=70.0, help='Field of view centered on the front (deg)')
    ap.add_argument('--wall-min-points', type=int, default=200,
                     help='Min wall points required to trust the angle estimate')
    ap.add_argument('--dry-run', action='store_true',
                     help='Print the procedure without sending real move/rotate commands over SSH '
                          '(capture services are still called for real)')
    ap.add_argument('--detect-only', action='store_true',
                     help='Repeatedly print the detected wall angle without driving - for sign/sanity checks')
    args = ap.parse_args()

    if args.host is None:
        ap.error('--host or the ROBOT_HOST environment variable is required')
    if args.password is None:
        ap.error('--password or the SSH_PASSWORD environment variable is required')

    rclpy.init()
    node = CalibrationDriver(args)
    try:
        if args.detect_only:
            print("[*] Wall-angle detection only mode - Ctrl+C to stop")
            while True:
                angle_error, n = node.detect_wall_angle()
                if angle_error is None:
                    print(f"[!] Not enough wall points (n={n})")
                else:
                    print(f"[*] wall angle error = {np.degrees(angle_error):+.2f} deg (n={n})")
                time.sleep(0.5)
        else:
            node.run()
    except KeyboardInterrupt:
        print("\n[!] Interrupted")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
