#!/usr/bin/env python3
# surface_profiling/test/check_chassis_state.py
"""
로봇이 바닥 돌출(raised-cosine 돔) 위를 지날 때의 차체 상태가 후처리 모드(raw/frame/window)에 주는 영향을,
실주행 프레임 기록(.npz)에 합성해 봄. 시뮬레이션을 돌리기 전에 하는 모델 기반 민감도 확인이며, 모델이
실제 차체 거동과 맞는지는 검증하지 않음.

돔 중심은 로봇이 실제로 지나간 위치(벽에서 1.3 m 이상, 반경 0.25 m 안 프레임이 가장 많은 곳)로 잡음.
프레임 f 의 로봇 위치를 p_f 라 할 때, 점 p 의 측정 z 를 시나리오별로 다음처럼 바꿈(돔 높이 함수 D):
  A  z + D(p)                                  차체 영향 없음(기준)
  B  z + D(p) - [D(p_f) + grad D(p_f).(p-p_f)]  차체가 돔 접평면을 따라 기울고, TF 는 평면(IMU TF off)
  C  z + D(p) - D(p_f)                          기울기는 IMU 로 보상, 로봇이 돔 높이만큼 뜬 것은 TF 에 안 실림
센서가 D(p_f) 만큼 떠 있어도 TF 는 지면 위 고정 높이라고 믿으므로 바닥이 그만큼 낮게 측정됨(부호 -).
1차(소각도) 근사이고 차체는 강체로 접평면을 따른다고 가정함. 모든 항이 높이에 선형이라 결과는 높이에 비례함.

지표는 돔 밖(돔 중심 거리 > R) 점의 잔차 변화 RMS[mm](돔이 없는 곳이므로 0에 가까워야 함). 돔 복원 이득은
오차가 돔 모양과 상관되면 1을 넘을 수 있어 참고용임.

사용 예:
  python3 check_chassis_state.py frames_<ts>.npz            # 약 3~4분
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import local_plane_heatmap as L  # noqa: E402

MODES = ('raw', 'frame', 'window')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('npz')
    ap.add_argument('--height', type=float, default=0.02, help='돔 높이[m]')
    ap.add_argument('--radii', type=float, nargs='+', default=[0.75, 1.0, 1.25], help='돔 반경[m]')
    args = ap.parse_args()

    log = L.load_frame_log(L.resolve_npz(args.npz))
    inside = L.load_node_lookup(os.path.join(L.HOME, 'maps', 'topology', 'final_topological_map.npz'))
    d, _, _ = L.load_points(log, 1.1, L.ring_limit(0.338, 4), 0.06, inside)
    x, y, px, py = d['x'], d['y'], d['cx'], d['cy']

    wall_dist = L.load_wall_distance(os.path.join(L.HOME, 'maps', 'grid', 'map_from_dae.yaml'))
    _, first = np.unique(d['fid'], return_index=True)
    poses = np.c_[px[first], py[first]]
    cand = poses[wall_dist(poses[:, 0], poses[:, 1]) >= 1.3]
    near = ((cand[:, None, :] - cand[None, :, :]) ** 2).sum(-1) < 0.25 ** 2
    cx, cy = cand[near.sum(1).argmax()]
    print(f"돔 중심 ({cx:.2f}, {cy:.2f}), 반경 0.25 m 안 프레임 {near.sum(1).max()}개, 높이 {args.height * 1e3:.0f} mm")

    base = {m: L.residual(m, d, d['z'], 3.0) for m in MODES}
    for R in args.radii:
        prof = L.dome(d, cx, cy, R, args.height)
        c_f = L.dome({'x': px, 'y': py}, cx, cy, R, args.height)
        rf = np.maximum(np.hypot(px - cx, py - cy), 1e-9)
        dzdr = np.where(rf < R, -(args.height / 2) * (np.pi / R) * np.sin(np.pi * rf / R), 0.0)
        tangent = c_f + dzdr * (px - cx) / rf * (x - px) + dzdr * (py - cy) / rf * (y - py)
        scen = {'A 차체 영향 없음': d['z'] + prof,
                'B IMU TF off (접평면을 따라 기울고 TF 는 평면)': d['z'] + prof - tangent,
                'C IMU TF on (기울기 보상, 뜬 높이는 미반영)': d['z'] + prof - c_f}
        on_dome = np.unique(d['fid'][np.hypot(px - cx, py - cy) < R]).size
        outside = np.hypot(x - cx, y - cy) > R
        print(f"--- R={R} m (돔 위 프레임 {on_dome}개, 최대 뜬 높이 {c_f.max() * 1e3:.1f} mm)")
        for name, zz in scen.items():
            cells = []
            for m in MODES:
                dr = L.residual(m, d, zz, 3.0) - base[m]
                ok = (prof > 0) & ~np.isnan(dr)
                gain = (dr[ok] * prof[ok]).sum() / (prof[ok] ** 2).sum()
                fo = outside & ~np.isnan(dr)
                cells.append(f"{m}: 돔 밖 RMS {np.sqrt((dr[fo] ** 2).mean()) * 1e3:5.2f} mm (이득 {gain:.2f})")
            print(f"{name:<44} " + " | ".join(cells))


if __name__ == '__main__':
    main()
