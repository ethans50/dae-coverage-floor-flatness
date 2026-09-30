#!/usr/bin/env python3
# surface_profiling/fit_heading_z_bias.py
"""
저장된 프레임 로그(.npz)로 잔차 IMU pitch/roll 바이어스를 추정하고, 그
보정을 적용하면 히트맵이 어떻게 바뀌는지 재주행 없이 미리 확인함.

모델: 바닥 셀 고정효과(그 위치의 실제 굴곡, 미지수)와 로봇 바디프레임
기준 전방(local_x)/좌측(local_y) 위치에 대한 선형 기울기(잔차 tilt)를
분리해서 추정함 - 셀별 평균을 빼면(within-estimator) 셀 고정효과가
소거되고, 남는 잔차를 로컬 좌표에 최소제곱으로 적합하면 그 위치의
실제 굴곡과 무관하게 기울기만 뽑을 수 있음. 이 기울기는 "같은 셀을
헤딩만 다르게 봤을 때 z가 헤딩에 따라 체계적으로 달라진다"는 실측
상관관계(2026-09-29 세션 진단)의 원인을 전방/좌측 두 축으로 직접
분해한 것임.

REP-103(X-forward/Y-left/Z-up) 작은각 근사로:
  a = dz/d(local_x) = -잔차_pitch(rad)   (양수 pitch = 앞으로 숙임)
  b = dz/d(local_y) = +잔차_roll(rad)    (양수 roll = 왼쪽이 들림)
잔차_pitch/roll는 지금 mission_execution.imu_mount_correction_rpy_deg가
과다/과소 보정하고 있는 양이므로, 그대로 더해주면 이론상 상쇄됨
(imu_tilt_broadcaster.py의 "raw - bias" 부호 규칙에서 유도).
"""

import argparse
import os
import sys

import numpy as np
import open3d as o3d

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from utils.frame_recorder import load_frame_log, used_frame_mask  # noqa: E402
from utils.config_paths import (  # noqa: E402
    load_config, resolve_pointcloud_dir, resolve_visualization_dir, resolve_map_yaml_path,
)
from utils.floor_extractor import extract_floor_by_height  # noqa: E402
from utils.heatmap_generator import generate_floor_heatmap  # noqa: E402


def fit_local_tilt(poses, points, offsets, used, z_min, z_max, grid):
    """전체 프레임 점(points)에서 z-window/used 프레임만 남겨 (a, b, r2)를 적합함.
    반환하는 mask/local_x/local_y/frame_of_point는 보정 적용(전체 점 대상)에 재사용함."""
    frame_of_point = np.searchsorted(offsets, np.arange(points.shape[0]), side='right') - 1
    robot_xy_all = poses[frame_of_point, :2]
    yaw_all = poses[frame_of_point, 3]
    dx_all = points[:, 0] - robot_xy_all[:, 0]
    dy_all = points[:, 1] - robot_xy_all[:, 1]
    cos_y, sin_y = np.cos(yaw_all), np.sin(yaw_all)
    local_x_all = dx_all * cos_y + dy_all * sin_y
    local_y_all = -dx_all * sin_y + dy_all * cos_y

    z = points[:, 2]
    in_window = (z >= z_min) & (z <= z_max)
    frame_used = used[frame_of_point]
    mask = in_window & frame_used

    local_x, local_y, zf = local_x_all[mask], local_y_all[mask], z[mask]
    cell_ix = np.floor(points[mask, 0] / grid).astype(np.int64)
    cell_iy = np.floor(points[mask, 1] / grid).astype(np.int64)
    _, cell_inv = np.unique(np.stack([cell_ix, cell_iy], axis=1), axis=0, return_inverse=True)

    cnt = np.bincount(cell_inv)
    mean_z = np.bincount(cell_inv, weights=zf) / cnt
    mean_lx = np.bincount(cell_inv, weights=local_x) / cnt
    mean_ly = np.bincount(cell_inv, weights=local_y) / cnt

    dz = zf - mean_z[cell_inv]
    dlx = local_x - mean_lx[cell_inv]
    dly = local_y - mean_ly[cell_inv]

    Sxx, Syy, Sxy = np.dot(dlx, dlx), np.dot(dly, dly), np.dot(dlx, dly)
    Sxz, Syz = np.dot(dlx, dz), np.dot(dly, dz)
    a, b = np.linalg.solve(np.array([[Sxx, Sxy], [Sxy, Syy]]), np.array([Sxz, Syz]))

    pred = a * dlx + b * dly
    ss_res = np.sum((dz - pred) ** 2)
    ss_tot = np.sum(dz ** 2)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float('nan')

    return a, b, r2, int(mask.sum()), int(cnt.shape[0]), local_x_all, local_y_all


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('npz_filename', help='frames/ 안의 프레임 로그 파일명 또는 절대경로')
    ap.add_argument('--z-min', type=float, default=None, help='적합/히트맵에 쓸 바닥 z-window 하한(m). 기본값: params.yaml surface_profiling.z_min')
    ap.add_argument('--z-max', type=float, default=None, help='적합/히트맵에 쓸 바닥 z-window 상한(m). 기본값: params.yaml surface_profiling.z_max')
    ap.add_argument('--fit-grid', type=float, default=0.05, help='적합용 셀 크기(m) - 히트맵 grid_size와 별개, 셀당 표본을 충분히 모으기 위해 더 크게 둠')
    args = ap.parse_args()

    workspace_root, profiling_cfg = load_config()
    pointcloud_dir = resolve_pointcloud_dir(workspace_root, profiling_cfg)
    visualization_dir = resolve_visualization_dir(workspace_root, profiling_cfg)
    map_yaml_path = resolve_map_yaml_path(workspace_root, profiling_cfg)

    z_min = args.z_min if args.z_min is not None else profiling_cfg.get('z_min', -0.010)
    z_max = args.z_max if args.z_max is not None else profiling_cfg.get('z_max', 0.010)
    grid_size = profiling_cfg.get('grid_size', 0.02)

    npz_path = args.npz_filename
    if not os.path.isabs(npz_path) and not os.path.exists(npz_path):
        npz_path = os.path.join(pointcloud_dir, 'frames', args.npz_filename)
    if not os.path.exists(npz_path):
        print(f"[!] Frame log not found: {npz_path}")
        sys.exit(1)

    print(f"[*] Loading {npz_path}")
    log = load_frame_log(npz_path)
    used = used_frame_mask(log)
    poses, points, offsets = log['poses'], log['points'], log['offsets']
    print(f"[*] z-window for fit/heatmap = [{z_min}, {z_max}] m, fit grid = {args.fit_grid} m")

    a, b, r2, n_fit_points, n_cells, local_x_all, local_y_all = fit_local_tilt(
        poses, points, offsets, used, z_min, z_max, args.fit_grid)

    residual_pitch_deg = -np.degrees(a)
    residual_roll_deg = np.degrees(b)
    print(f"\n[*] 적합 표본: 점 {n_fit_points}개, 셀 {n_cells}개")
    print(f"[*] a(dz/dlocal_x)={a:.6f}, b(dz/dlocal_y)={b:.6f}  (셀 고정효과 제거 후 R^2={r2:.3f})")
    print(f"[*] 추정 잔차 tilt: pitch={residual_pitch_deg:+.4f}deg, roll={residual_roll_deg:+.4f}deg")
    print(f"[*] imu_mount_correction_rpy_deg에 더할 값(권장, 부호는 아래 보정 히트맵으로 먼저 검증): "
          f"roll += {residual_roll_deg:+.4f}, pitch += {residual_pitch_deg:+.4f}")

    # 적합에 쓴 (a, b)로 전체 점(넓은 z 밴드, used 프레임)에 보정을 적용해
    # 재주행 없이 "보정하면 히트맵이 어떻게 바뀌는지" 바로 확인함.
    frame_of_point = np.searchsorted(offsets, np.arange(points.shape[0]), side='right') - 1
    frame_used = used[frame_of_point]
    corrected_z = points[:, 2] - (a * local_x_all + b * local_y_all)
    corrected_points = points[frame_used].copy()
    corrected_points[:, 2] = corrected_z[frame_used]
    print(f"\n[*] 보정 적용 대상 점 {corrected_points.shape[0]}개(used 프레임 전체, 넓은 z 밴드)")

    stem = os.path.splitext(os.path.basename(npz_path))[0].replace('frames_', '')
    raw_out = os.path.join(pointcloud_dir, f"combined_biascorrected_test_{stem}.pcd")
    filtered_out = os.path.join(pointcloud_dir, f"combined_filtered_biascorrected_test_{stem}.pcd")
    heatmap_out = os.path.join(visualization_dir, f"floor_heatmap_biascorrected_test_{stem}.png")

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(corrected_points.astype(np.float64))
    o3d.io.write_point_cloud(raw_out, pcd)
    print(f"[+] Wrote corrected raw PCD: {raw_out}")

    extract_floor_by_height(raw_out, filtered_out, z_min=z_min, z_max=z_max)
    generate_floor_heatmap(filtered_out, heatmap_out, grid_size=grid_size,
                            z_min=z_min, z_max=z_max, map_yaml_dir=map_yaml_path)
    print(f"[+] Wrote corrected heatmap: {heatmap_out}")


if __name__ == '__main__':
    main()
