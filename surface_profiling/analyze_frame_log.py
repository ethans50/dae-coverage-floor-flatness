#!/usr/bin/env python3
# surface_profiling/analyze_frame_log.py
"""
프레임 기록(.npz)으로 셀별 (a) 점 수와 (b) 서로 다른 통과 횟수를 따로 세어, repass가
실제로 필요한지 판별하는 분석 스크립트. 주행 없이 오프라인으로 실행함.

"통과"는 셀별로, 그 셀에 점이 찍힌 프레임들을 시간순으로 놓고 프레임 간격이
--pass-gap 초를 넘는 지점에서 끊어 센 것임(라이다 범위 안에 머무는 연속 프레임은
한 번의 통과, 시간이 벌어져 다시 찍히면 별개의 통과). 점은 최종 결과에 반영된 프레임(기각되지 않았고 캡처
구간인 프레임)의 z-window 안 점만 셈.

출력: 통과 횟수별 셀당 점 수 분포. 한 번만 지난 셀의 점 수가 충분하면 repass가
필요 없고, 부족하면 repass가 필요함을 뜻함. --map을 주면 벽까지의 거리 구간별로도
나눠 보여줌(벽 근처 점 부족 여부 확인용).

사용 예:
  python3 analyze_frame_log.py ~/dae_floor_maps/analytics/pointclouds/frames/frames_<ts>.npz \\
      --z-min -0.025 --z-max 0.030 --grid 0.02 --map ~/dae_floor_maps/maps/grid/map_from_dae.yaml
"""

import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from utils.frame_recorder import load_frame_log, used_frame_mask  # noqa: E402
from utils.heatmap_generator import _load_occupancy_map  # noqa: E402
from utils.frame_video import _wall_mask  # noqa: E402


def compute_cell_stats(log, z_min, z_max, grid, pass_gap_sec):
    """셀별 (점 수, 통과 횟수)를 계산함.

    반환: (cell_x, cell_y, n_points, n_passes, (ix0, iy0)) - 점이 1개 이상 있는 셀만, 1차원 배열.
    셀 world 인덱스는 (cell_x + ix0, cell_y + iy0)이고 floor(x/grid), floor(y/grid) 기준임.
    """
    stamps, offsets, points = log['stamps'], log['offsets'], log['points']
    used = used_frame_mask(log)

    frame_ids = np.nonzero(used & (offsets[1:] > offsets[:-1]))[0]
    if frame_ids.size == 0:
        return (np.empty(0, np.int64),) * 4 + ((0, 0),)

    counts = (offsets[1:] - offsets[:-1])
    frame_of_point = np.repeat(np.arange(len(stamps)), counts)
    sel = np.isin(frame_of_point, frame_ids) & (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
    pts = points[sel]
    frames = frame_of_point[sel]
    if pts.shape[0] == 0:
        return (np.empty(0, np.int64),) * 4 + ((0, 0),)

    ix = np.floor(pts[:, 0] / grid).astype(np.int64)
    iy = np.floor(pts[:, 1] / grid).astype(np.int64)
    ix0, iy0 = ix.min(), iy.min()
    ix -= ix0
    iy -= iy0
    ny = iy.max() + 1
    cell = ix * ny + iy

    uniq_cell, inv, n_points = np.unique(cell, return_inverse=True, return_counts=True)

    # 통과 횟수는 셀별로 셈: 그 셀에 점이 찍힌 서로 다른 프레임의 시각을 정렬해
    # 간격이 pass_gap_sec을 넘는 곳마다 새 통과로 봄(연속 캡처 중 로봇이 같은 셀을
    # 다시 지나가는 경우도 셀 시각열이 끊기므로 구분됨).
    pair = np.unique(np.stack([inv, frames], axis=1), axis=0)   # (셀 idx, 프레임) 고유쌍
    t = stamps[pair[:, 1]]
    order = np.lexsort((t, pair[:, 0]))
    c_sorted, t_sorted = pair[order, 0], t[order]
    new_pass = np.ones(len(c_sorted), dtype=bool)
    new_pass[1:] = (c_sorted[1:] != c_sorted[:-1]) | (np.diff(t_sorted) > pass_gap_sec)
    n_passes = np.bincount(c_sorted[new_pass], minlength=len(uniq_cell))
    return uniq_cell // ny, uniq_cell % ny, n_points, n_passes, (ix0, iy0)


def _wall_distance_m(log_cells_xy_world, map_yaml):
    """셀 중심(world)에서 가장 가까운 벽까지의 거리[m]를 도면에서 구함."""
    map_img, res, ox, oy = _load_occupancy_map(map_yaml)
    walls = _wall_mask(map_img)
    dist_px = cv2.distanceTransform((~walls).astype(np.uint8), cv2.DIST_L2, 5)
    xs, ys = log_cells_xy_world
    cols = np.clip(((xs - ox) / res).astype(int), 0, walls.shape[1] - 1)
    rows = np.clip(((ys - oy) / res).astype(int), 0, walls.shape[0] - 1)
    return dist_px[rows, cols] * res


def _summarize(name, n_points, min_points):
    if n_points.size == 0:
        print(f"  {name:<22} cells=0")
        return
    p10, p50, p90 = np.percentile(n_points, [10, 50, 90])
    print(f"  {name:<22} cells={n_points.size:>7}  pts/cell p10={p10:6.1f} p50={p50:6.1f} p90={p90:6.1f}"
          f"  <{min_points}pts: {100.0 * np.mean(n_points < min_points):5.1f}%")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('npz')
    ap.add_argument('--z-min', type=float, required=True)
    ap.add_argument('--z-max', type=float, required=True)
    ap.add_argument('--grid', type=float, default=0.02, help='셀 크기 [m]')
    ap.add_argument('--pass-gap', type=float, default=5.0,
                    help='한 셀의 프레임 시각 간격이 이 값[s]을 넘으면 새 통과로 셈(연속 프레임 주기 ~0.1s, 기각 프레임 구멍보다 충분히 크게)')
    ap.add_argument('--min-points', type=int, default=5, help='"부족"으로 셀 비율을 보고할 셀당 점 수 기준')
    ap.add_argument('--map', default=None, help='지정하면 벽 거리 구간별 분포도 출력')
    args = ap.parse_args()

    log = load_frame_log(args.npz)
    res = compute_cell_stats(log, args.z_min, args.z_max, args.grid, args.pass_gap)
    if res[0].size == 0:
        print("[-] z-window 안에 반영된 점이 없음.")
        return
    cx, cy, n_points, n_passes, (ix0, iy0) = res

    print(f"[*] frames={len(log['stamps'])}  cells with points={cx.size}  grid={args.grid}m  pass-gap={args.pass_gap}s")
    print(f"[*] 통과 횟수 분포 (셀 수): " +
          ", ".join(f"{k}회={int(np.sum(n_passes == k))}" for k in range(1, int(n_passes.max()) + 1)))

    print("\n[셀당 점 수 by 통과 횟수]")
    _summarize("1회 통과", n_points[n_passes == 1], args.min_points)
    _summarize("2회 이상 통과", n_points[n_passes >= 2], args.min_points)
    _summarize("전체", n_points, args.min_points)

    if args.map:
        # 셀 중심 world 좌표 복원(ix0/iy0은 compute_cell_stats에서 뺀 최솟값 인덱스).
        wx = (cx + ix0 + 0.5) * args.grid
        wy = (cy + iy0 + 0.5) * args.grid
        dist = _wall_distance_m((wx, wy), args.map)
        bands = [(0.0, 0.3), (0.3, 0.6), (0.6, 1.0), (1.0, np.inf)]
        for label, mask_passes in (("1회 통과", n_passes == 1), ("2회 이상 통과", n_passes >= 2)):
            print(f"\n[벽 거리 구간별 셀당 점 수 - {label}]")
            for lo, hi in bands:
                m = mask_passes & (dist >= lo) & (dist < hi)
                _summarize(f"{lo:.1f}~{hi:.1f}m", n_points[m], args.min_points)


if __name__ == '__main__':
    main()
