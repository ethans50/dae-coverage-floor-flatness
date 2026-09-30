#!/usr/bin/env python3
# surface_profiling/analyze_ring_coverage.py
"""
프레임 기록(.npz)의 실제 궤적으로 VLP-16 ring이 각 셀을 몇 번 스치는지(n(x))
계산해, 같은 파일의 실측 셀별 점 수와 비교하는 검증 스크립트.

repass의 새 정의(2026-09) - "두 번 훑는다"가 아니라 "모든 점에 대해 ring
하나 이상이 지나가도록 보장한다" - 가 실제로 맞는 모델인지 확인하기 위함.
n(x)=0인데 실측 점이 있으면 모델(ring 반경/각도) 자체가 틀린 것이고,
n(x)>=1인데 실측 점이 0이면 방위각 샘플링 간격 때문에 ring은 지나갔지만
정확히 그 셀을 못 맞춘 경우임(확률적으로 있을 수 있음, n(x)가 클수록 비율이
낮아져야 모델이 맞다는 뜻).

사용 예:
  python3 analyze_ring_coverage.py frames_<ts>.npz --z-min -0.025 --z-max 0.030
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from utils.frame_recorder import load_frame_log, used_frame_mask  # noqa: E402
from utils.ring_coverage import (  # noqa: E402
    vlp16_ring_radii_m, downsample_trajectory, compute_ring_pass_count, build_candidate_grid,
)
from analyze_frame_log import compute_cell_stats  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('npz')
    ap.add_argument('--z-min', type=float, required=True)
    ap.add_argument('--z-max', type=float, required=True)
    ap.add_argument('--grid', type=float, default=0.05, help='셀 크기[m] - 실측 대조와 후보 격자 둘 다에 씀. 가늘수록 정확하지만 느려짐')
    ap.add_argument('--mount-height', type=float, default=0.338, help='velodyne_link 높이[m] (params.yaml의 lidar_mount_height와 맞출 것)')
    ap.add_argument('--traj-step', type=float, default=0.03, help='궤적 다운샘플 최소 이동거리[m] - d(s) min/max 계산량을 줄임')
    ap.add_argument('--pass-gap', type=float, default=5.0, help='analyze_frame_log와 동일 - 실측 통과 횟수 판정용(참고 출력에만 씀)')
    args = ap.parse_args()

    log = load_frame_log(args.npz)
    used = used_frame_mask(log)
    traj_xy = log['poses'][used][:, :2]
    if traj_xy.shape[0] < 2:
        print("[-] 사용 가능한 프레임이 2개 미만 - 궤적을 만들 수 없음.")
        return
    traj_xy = downsample_trajectory(traj_xy, args.traj_step)
    print(f"[*] 궤적 샘플 {traj_xy.shape[0]}개(다운샘플 후, step>={args.traj_step}m)")

    ring_radii = vlp16_ring_radii_m(args.mount_height)
    print(f"[*] ring 반경(m): {np.round(ring_radii, 3).tolist()}  (mount_height={args.mount_height}m)")

    centers, cand_ix, cand_iy = build_candidate_grid(traj_xy, args.grid, margin_m=ring_radii.max())
    print(f"[*] 후보 셀 {centers.shape[0]}개(grid={args.grid}m) 계산 중...")
    n_model = compute_ring_pass_count(centers, traj_xy, ring_radii)

    real_cx, real_cy, real_n_points, _, (ix0, iy0) = compute_cell_stats(
        log, args.z_min, args.z_max, args.grid, args.pass_gap)
    real_by_cell = dict(zip(zip((real_cx + ix0).tolist(), (real_cy + iy0).tolist()), real_n_points.tolist()))
    real_n = np.array([real_by_cell.get((int(x), int(y)), 0) for x, y in zip(cand_ix, cand_iy)])

    print(f"\n[*] 후보 셀 {centers.shape[0]}개 중 실측 점이 있는 셀 {int((real_n > 0).sum())}개")

    print("\n[모델 n(x)=0 셀 검증] - 실측 점이 있으면 모델(ring 반경/각도)이 틀린 것임")
    zero_model = n_model == 0
    n_zero = int(zero_model.sum())
    n_zero_but_real = int((zero_model & (real_n > 0)).sum())
    print(f"  n(x)=0 셀: {n_zero}개, 그중 실측 점 있음: {n_zero_but_real}개"
          f" ({100.0 * n_zero_but_real / n_zero:.2f}%)" if n_zero else "  n(x)=0 셀 없음")

    print("\n[모델 n(x)>=1 셀의 실측 미검출률] - ring은 지나갔지만 방위각 샘플링 간격으로 못 찍힌 비율")
    print("  n(x) | 후보셀수 | 실측 0인 비율 | 실측 점수(median, >0인 셀만)")
    for k in range(1, int(n_model.max()) + 1):
        mask = n_model == k
        cnt = int(mask.sum())
        if cnt == 0:
            continue
        miss_rate = 100.0 * float((real_n[mask] == 0).mean())
        positive = real_n[mask][real_n[mask] > 0]
        med = float(np.median(positive)) if positive.size else float('nan')
        print(f"  {k:>4} | {cnt:>7} | {miss_rate:>10.2f}% | {med:>10.1f}")


if __name__ == '__main__':
    main()
