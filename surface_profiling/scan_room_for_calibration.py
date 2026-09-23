#!/usr/bin/env python3
# surface_profiling/scan_room_for_calibration.py
"""
Stand the LiDAR in the room to be calibrated and run this script: it collects
N seconds of raw point cloud from that spot (sensor frame, no map/AMCL
needed), draws a 2D floor plan of the room, and saves an image showing where
to place and how to orient the robot to start a four-heading calibration with
auto_calibration_drive.py, plus the distance to each wall from that point.

Works even if the room is not square - it uses the detected room outline's
minimum-area rotated rectangle as-is, so a long/short side mismatch is
reported directly.

Example:
  python3 scan_room_for_calibration.py --duration 6 --out room_placement_guide.png
"""

import argparse
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


class RoomScanner(Node):
    def __init__(self, duration):
        super().__init__('scan_room_for_calibration')
        self.duration = duration
        self.chunks = []
        self.create_subscription(PointCloud2, '/velodyne_points', self._cb, 10)

    def _cb(self, msg):
        raw = pc2.read_points(msg, skip_nans=True, field_names=('x', 'y', 'z'))
        pts = np.array([(p[0], p[1], p[2]) for p in raw], dtype=np.float32)
        if len(pts):
            self.chunks.append(pts)

    def collect(self):
        t0 = time.time()
        print(f"[*] Collecting scan for {self.duration}s...")
        while time.time() - t0 < self.duration:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.chunks:
            raise RuntimeError("[-] Received no points at all - check whether /velodyne_points is being published")
        return np.concatenate(self.chunks, axis=0)


def extract_wall_points(points, z_min, z_max, r_max):
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    r = np.hypot(x, y)
    mask = (z >= z_min) & (z <= z_max) & (r > 0.3) & (r <= r_max)
    return points[mask]


def rasterize(points_xy, cell_size, margin=0.5):
    x, y = points_xy[:, 0], points_xy[:, 1]
    x_min, x_max = x.min() - margin, x.max() + margin
    y_min, y_max = y.min() - margin, y.max() + margin
    w = int(np.ceil((x_max - x_min) / cell_size))
    h = int(np.ceil((y_max - y_min) / cell_size))
    grid = np.zeros((h, w), dtype=np.int32)
    cols = ((x - x_min) / cell_size).astype(int).clip(0, w - 1)
    rows = ((y - y_min) / cell_size).astype(int).clip(0, h - 1)
    np.add.at(grid, (rows, cols), 1)
    return grid, x_min, y_min


def find_room_rect(wall_grid, min_hits, sensor_rc):
    """wall_grid(셀별 점 개수)를 이진화하고, 센서 위치에서 flood-fill로 빈 공간
    (방 내부)을 구한 뒤 그 외곽을 감싸는 최소회전사각형을 반환함."""
    walls = (wall_grid >= min_hits).astype(np.uint8) * 255
    free = (255 - walls).astype(np.uint8)
    seed = (int(sensor_rc[0]), int(sensor_rc[1]))  # cv2 seed는 (x=col, y=row)
    filled = free.copy()
    cv2.floodFill(filled, None, seed, 128)
    room_mask = (filled == 128).astype(np.uint8)
    contours, _ = cv2.findContours(room_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    return cv2.minAreaRect(largest)  # ((cx,cy), (w,h), angle_deg) - 격자 셀 단위


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--duration', type=float, default=6.0, help='Scan collection time (s)')
    ap.add_argument('--z-min', type=float, default=-0.2, help='Min height counted as wall (sensor frame, m)')
    ap.add_argument('--z-max', type=float, default=1.2, help='Max height counted as wall (sensor frame, m)')
    ap.add_argument('--r-max', type=float, default=8.0, help='Max effective range counted as wall (m)')
    ap.add_argument('--cell-size', type=float, default=0.05, help='2D grid cell size (m)')
    ap.add_argument('--min-hits', type=int, default=15, help='Min points per cell to count it as wall')
    ap.add_argument('--out', default='room_placement_guide.png')
    args = ap.parse_args()

    rclpy.init()
    node = RoomScanner(args.duration)
    points = node.collect()
    rclpy.shutdown()

    wall_pts = extract_wall_points(points, args.z_min, args.z_max, args.r_max)
    if wall_pts.shape[0] < 500:
        raise RuntimeError(f"[-] Too few wall points ({wall_pts.shape[0]}) - adjust --z-min/--z-max/--r-max")

    grid, x_min, y_min = rasterize(wall_pts[:, :2], args.cell_size)
    sensor_col = int(round((0.0 - x_min) / args.cell_size))
    sensor_row = int(round((0.0 - y_min) / args.cell_size))

    rect = find_room_rect(grid, args.min_hits, (sensor_col, sensor_row))
    if rect is None:
        raise RuntimeError("[-] Failed to detect the room outline - adjust the scan position or "
                            "--min-hits/--z-min/--z-max")

    (cx, cy), (rw, rh), angle_deg = rect
    center_x = x_min + cx * args.cell_size
    center_y = y_min + cy * args.cell_size
    room_w_m = rw * args.cell_size
    room_h_m = rh * args.cell_size

    dx, dy = center_x, center_y  # 현재 센서 위치(0,0) 기준 추천 배치 지점까지 상대 오프셋
    print(f"[*] Estimated room size: {room_w_m:.2f} m x {room_h_m:.2f} m, rectangle rotation {angle_deg:.1f} deg")
    print(f"[*] Recommended placement: from here, move forward(x) {dx:+.2f} m, left(y) {dy:+.2f} m")
    print(f"[*] From that point, rotate {angle_deg:.1f} deg from the current heading to align with the "
          "room's long side before starting calibration")
    print(f"[*] Distance to walls (from that point): +-{max(room_w_m, room_h_m) / 2:.2f} m along the long side, "
          f"+-{min(room_w_m, room_h_m) / 2:.2f} m along the short side")

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(grid > 0, cmap='gray_r', origin='lower',
              extent=[x_min, x_min + grid.shape[1] * args.cell_size,
                      y_min, y_min + grid.shape[0] * args.cell_size])
    box = cv2.boxPoints(rect)
    box_world = np.array([[x_min + px * args.cell_size, y_min + py * args.cell_size] for px, py in box])
    box_world = np.vstack([box_world, box_world[0]])
    ax.plot(box_world[:, 0], box_world[:, 1], 'g-', linewidth=2, label='Estimated room outline')
    ax.plot(0, 0, 'b^', markersize=12, label='Current LiDAR position')
    ax.plot(center_x, center_y, 'r*', markersize=16, label='Recommended placement')
    heading_rad = np.radians(angle_deg)
    ax.annotate('', xy=(center_x + 0.4 * np.cos(heading_rad), center_y + 0.4 * np.sin(heading_rad)),
                xytext=(center_x, center_y), arrowprops=dict(facecolor='red', width=2))
    ax.set_aspect('equal')
    ax.set_xlabel('x (m, current sensor forward)')
    ax.set_ylabel('y (m, current sensor left)')
    ax.set_title(f'Room placement guide - move forward {dx:+.2f}m, left {dy:+.2f}m, then rotate {angle_deg:.1f}deg')
    ax.legend(loc='upper right', fontsize=8)
    plt.savefig(args.out, dpi=150, bbox_inches='tight')
    print(f"[*] Saved: {args.out}")


if __name__ == '__main__':
    main()
