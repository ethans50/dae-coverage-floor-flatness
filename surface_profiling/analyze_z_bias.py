#!/usr/bin/env python3
# surface_profiling/analyze_z_bias.py
"""
프레임 기록(.npz)에서 바닥 z의 체계 오차(센서 기울기)와 무작위 노이즈를 분리해 보는 스크립트.

평평한 바닥에서는 로봇 좌표계 (전방 f, 좌측 l)에서 z = a*f + b*l + c 평면이 나와야 함.
프레임마다 이 평면을 최소제곱으로 맞춰서
  - pitch = atan(a), roll = atan(b): 센서(또는 차체) 기울기. 프레임 간 표준편차가 작으면 고정
    오프셋(마운트/차체 자세)이고, 크면 주행 중 동적으로 변하는 기울기임.
  - 평면 제거 전/후 z 표준편차: 기울기가 z 퍼짐의 얼마를 설명하는지.
를 출력함. 벽면 점이 섞이면 결과가 크게 오염되므로 --map을 주어 벽에서 --wall-margin
이내의 점을 제외함(주지 않으면 z 밴드만 적용됨).

사용 예:
  python3 analyze_z_bias.py frames_<ts>.npz --map ~/dae_floor_maps/maps/grid/map_from_dae.yaml
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


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('npz')
    ap.add_argument('--map', default=None)
    ap.add_argument('--wall-margin', type=float, default=0.4, help='벽에서 이 거리[m] 이내 점 제외')
    ap.add_argument('--z-band', type=float, default=0.1, help='|z|가 이 값[m] 이하인 점만 사용')
    ap.add_argument('--r-min', type=float, default=1.2)
    ap.add_argument('--r-max', type=float, default=3.5)
    ap.add_argument('--min-points', type=int, default=300, help='프레임당 평면 적합에 필요한 최소 점 수')
    args = ap.parse_args()

    log = load_frame_log(args.npz)
    off, poses = log['offsets'], log['poses']
    used = used_frame_mask(log)

    wall_dist = None
    if args.map:
        img, res, ox, oy = _load_occupancy_map(args.map)
        walls = _wall_mask(img)
        dist = cv2.distanceTransform((~walls).astype(np.uint8), cv2.DIST_L2, 5) * res

        def wall_dist(p):
            cols = np.clip(((p[:, 0] - ox) / res).astype(int), 0, walls.shape[1] - 1)
            rows = np.clip(((p[:, 1] - oy) / res).astype(int), 0, walls.shape[0] - 1)
            return dist[rows, cols]

    rows_out, zs = [], []
    for i in np.nonzero(used & (off[1:] > off[:-1]))[0]:
        p = log['points'][off[i]:off[i + 1]]
        p = p[np.abs(p[:, 2]) <= args.z_band]
        if wall_dist is not None:
            p = p[wall_dist(p) > args.wall_margin]
        dx, dy = p[:, 0] - poses[i, 0], p[:, 1] - poses[i, 1]
        c, s = np.cos(poses[i, 3]), np.sin(poses[i, 3])
        f, l = dx * c + dy * s, -dx * s + dy * c
        r = np.hypot(f, l)
        k = (r >= args.r_min) & (r <= args.r_max)
        if k.sum() < args.min_points:
            continue
        A = np.c_[f[k], l[k], np.ones(k.sum())]
        z = p[k, 2]
        coef, *_ = np.linalg.lstsq(A, z, rcond=None)
        rows_out.append((np.degrees(np.arctan(coef[0])), np.degrees(np.arctan(coef[1])),
                         coef[2] * 100, z.std() * 100, (z - A @ coef).std() * 100))
        zs.append(z)

    if not rows_out:
        print("[-] 조건을 만족하는 프레임이 없음.")
        return
    R, z = np.array(rows_out), np.concatenate(zs)
    print(f"[*] frames={len(R)}  points={z.size}  range {args.r_min}-{args.r_max}m  "
          f"wall-margin={args.wall_margin if wall_dist else 'off'}")
    print(f"z(cm): mean {z.mean()*100:.3f}  std {z.std()*100:.3f}  p1/p99 {np.percentile(z*100, 1):.2f}/{np.percentile(z*100, 99):.2f}")
    print(f"pitch(deg, +=전방이 높음): mean {R[:,0].mean():.3f}  std(프레임 간) {R[:,0].std():.3f}")
    print(f"roll (deg, +=좌측이 높음): mean {R[:,1].mean():.3f}  std(프레임 간) {R[:,1].std():.3f}")
    print(f"제안값: lidar_mount_correction_rpy_deg = [{-R[:,1].mean():.3f}, {R[:,0].mean():.3f}, 0.0]  "
          "(보정을 끈 데이터 기준 값. 보정이 켜진 데이터라면 현재 설정값에 이 값을 더한 것이 새 보정값)")
    print(f"프레임 내 z std(cm): 평면 제거 전 {R[:,3].mean():.2f} -> 후 {R[:,4].mean():.2f}")


if __name__ == '__main__':
    main()
