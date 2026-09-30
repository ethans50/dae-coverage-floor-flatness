#!/usr/bin/env python3
# surface_profiling/view_pcd.py
"""
PCD의 점을 격자로 집계하지 않고 점 하나하나를 z로 칠해서 보는 스크립트.

  - 기본: Open3D 뷰어 창(회전/확대 가능). 점 색은 히트맵과 같은 jet, 범위는 [z_min, z_max]로 고정.
  - --png <경로>: 창 없이 top-down 이미지로 저장(matplotlib, 헤드리스 가능). 점마다 색을 칠하고
    셀 평균이나 최소 점 수 조건이 없음. --map을 주면 도면 벽을 배경에 깔음.

범위 밖 z는 양 끝 색으로 clip되며(히트맵과 같은 규칙), --drop-outside를 주면 범위 밖 점을 아예 뺌.

사용 예:
  python3 view_pcd.py ~/dae_floor_maps/analytics/pointclouds/combined_<ts>.pcd --z-min -0.01 --z-max 0.01
  python3 view_pcd.py <pcd> --z-min -0.01 --z-max 0.01 --png /tmp/points.png --map ~/dae_floor_maps/maps/grid/map_from_dae.yaml
"""

import argparse
import os
import sys

import numpy as np
import open3d as o3d

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def colorize(z, z_min, z_max):
    import matplotlib
    return matplotlib.colormaps['jet'](np.clip((z - z_min) / (z_max - z_min), 0.0, 1.0))[:, :3]


def save_topdown_png(points, colors, z_min, z_max, out_path, map_yaml=None, point_px=1.0):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(12, 10))
    if map_yaml:
        from utils.heatmap_generator import _load_occupancy_map
        img, res, ox, oy = _load_occupancy_map(map_yaml)
        ax.imshow(img, cmap='gray', origin='lower',
                  extent=[ox, ox + img.shape[1] * res, oy, oy + img.shape[0] * res], zorder=0)
    # s는 점 면적[pt^2]. 점끼리 이어 보이지 않게 작게 두고, 실제 점 위치에 그대로 찍음.
    ax.scatter(points[:, 0], points[:, 1], c=colors, s=point_px, marker='.', linewidths=0, zorder=1)
    ax.set_aspect('equal')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(f'Points colored by z (jet, {z_min * 100:.1f} to {z_max * 100:.1f} cm)')
    sm = plt.cm.ScalarMappable(cmap='jet', norm=plt.Normalize(z_min * 100, z_max * 100))
    fig.colorbar(sm, ax=ax, label='Height (Z) [cm]')
    plt.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    plt.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('pcd')
    ap.add_argument('--z-min', type=float, required=True)
    ap.add_argument('--z-max', type=float, required=True)
    ap.add_argument('--drop-outside', action='store_true', help='z 범위 밖 점을 제거함(기본은 양 끝 색으로 clip)')
    ap.add_argument('--png', default=None, help='지정하면 창 대신 top-down PNG로 저장')
    ap.add_argument('--map', default=None, help='--png 배경에 깔 도면 yaml')
    ap.add_argument('--point-size', type=float, default=1.0, help='--png 점 크기(pt^2)')
    args = ap.parse_args()

    pcd = o3d.io.read_point_cloud(args.pcd)
    points = np.asarray(pcd.points)
    if points.shape[0] == 0:
        sys.exit("[-] PCD에 점이 없음.")
    if args.drop_outside:
        keep = (points[:, 2] >= args.z_min) & (points[:, 2] <= args.z_max)
        points = points[keep]
    colors = colorize(points[:, 2], args.z_min, args.z_max)
    print(f"[*] points={len(points)}  z range in file: {points[:, 2].min() * 100:.2f} to {points[:, 2].max() * 100:.2f} cm")

    if args.png:
        save_topdown_png(points, colors, args.z_min, args.z_max, args.png, args.map, args.point_size)
        print(f"[+] Saved: {args.png}")
        return

    view = o3d.geometry.PointCloud()
    view.points = o3d.utility.Vector3dVector(points)
    view.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([view], window_name="PCD by z (jet)")


if __name__ == '__main__':
    main()
