#!/usr/bin/env python3
# surface_profiling/test/check_map_alignment.py
"""
프레임 기록(frames_*.npz)의 점군이 지도(map_from_dae.yaml)와 맞게 놓였는지 확인함.

점은 AMCL 이 준 map->odom 로 map 좌표에 변환돼 저장되므로, 측정 위치에서 AMCL 초기 위치가
틀렸으면 점군 전체가 지도와 어긋남. analyze_z_bias.py 의 --map 은 이 map 좌표를 그대로 믿고
"지도 벽에서 일정 거리 이내 점"을 제외하므로, 어긋나 있으면 엉뚱한 점이 제외됨.

출력:
  - 센서 위치의 평균/표준편차(정지 측정이면 표준편차가 작아야 함)와 지도상 벽까지 거리
  - 벽면 높이(z 0.15~0.45m) 점 중 지도 벽 근처에 있는 비율(정합이면 90% 이상)

사용 예:
  python3 check_map_alignment.py frames_<timestamp>.npz --map ~/dae_floor_maps/maps/grid/map_from_dae.yaml
"""

import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from utils.frame_recorder import describe_capture_conditions, load_frame_log, used_frame_mask  # noqa: E402
from utils.heatmap_generator import _load_occupancy_map  # noqa: E402
from utils.frame_video import _wall_mask  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('npz')
    ap.add_argument('--map', required=True)
    ap.add_argument('--near', type=float, default=0.15, help='벽 근처로 보는 거리(m)')
    args = ap.parse_args()

    log = load_frame_log(os.path.expanduser(args.npz))
    cond_lines, cond_warns = describe_capture_conditions(log)
    for ln in cond_lines:
        print(f"측정 조건: {ln}")
    for w in cond_warns:
        print(f"[!] {w}")
    off, poses = log['offsets'], log['poses']
    img, res, ox, oy = _load_occupancy_map(os.path.expanduser(args.map))
    walls = _wall_mask(img)
    dist = cv2.distanceTransform((~walls).astype(np.uint8), cv2.DIST_L2, 5) * res

    def wall_dist(xy):
        c = np.clip(((xy[:, 0] - ox) / res).astype(int), 0, walls.shape[1] - 1)
        r = np.clip(((xy[:, 1] - oy) / res).astype(int), 0, walls.shape[0] - 1)
        return dist[r, c]

    idx = np.nonzero(used_frame_mask(log) & (off[1:] > off[:-1]))[0]
    if len(idx) == 0:
        sys.exit('[-] 캡처된 프레임이 없음')
    p = poses[idx]
    print(f"프레임 {len(idx)}개  센서 위치 평균 ({p[:, 0].mean():.2f}, {p[:, 1].mean():.2f}) m, "
          f"표준편차 ({p[:, 0].std():.3f}, {p[:, 1].std():.3f}) m")
    print(f"센서 위치의 지도상 벽까지 거리 평균 {wall_dist(p[:, :2]).mean():.2f} m (0 근처면 벽 위 = 위치 이상)")

    step = max(1, len(idx) // 60)
    pts = np.concatenate([log['points'][off[i]:off[i + 1]] for i in idx[::step]])
    w = pts[(pts[:, 2] > 0.15) & (pts[:, 2] < 0.45)]
    if len(w) == 0:
        sys.exit('[-] 벽면 높이 점이 없음')
    d = wall_dist(w[:, :2])
    print(f"벽면 높이 점 {len(w)}개 중 지도 벽에서 {args.near}m 이내 {100 * (d < args.near).mean():.0f}%, "
          f"{2 * args.near}m 이내 {100 * (d < 2 * args.near).mean():.0f}%")


if __name__ == '__main__':
    main()
