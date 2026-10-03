#!/usr/bin/env python3
# surface_profiling/test/check_floor_adjustment.py
"""
utils/floor_adjustment.py 의 합성 데이터 검증. 프레임마다 임의의 평면 오차(높이, 기울기)를 더한 평평한 바닥 + 원형 돔에서
  1) 프레임 평면 오차를 지우고 평평한 바닥이 평평하게 복원되는가
  2) 프레임 평면 적합(frame 모드)이 지우는 큰 돔(반경 1.5 m)을 더 많이 복원하는가

사용 예:
  python3 check_floor_adjustment.py
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from utils.floor_adjustment import FloorAdjustment  # noqa: E402
from utils.local_plane import frame_plane_residual  # noqa: E402


def main():
    rng = np.random.default_rng(0)
    C, nx, ny = 0.25, 36, 36
    cell_xy = np.c_[((np.arange(nx * ny) % nx) + .5) * C, ((np.arange(nx * ny) // nx) + .5) * C]
    X, Y, FI, FX, FY, TZ = [], [], [], [], [], []
    frame_xy = []
    for f in range(220):
        px, py = rng.uniform(2.2, 6.8, 2)
        ang, r = rng.uniform(0, 2 * np.pi, 3000), rng.uniform(1.1, 2.44, 3000)
        x, y = px + r * np.cos(ang), py + r * np.sin(ang)
        X.append(x); Y.append(y); FI.append(np.full(x.size, f)); frame_xy.append((px, py))
    x, y, fi = np.concatenate(X), np.concatenate(Y), np.concatenate(FI)
    frame_xy = np.array(frame_xy)
    cell = (np.clip((y / C).astype(int), 0, ny - 1)) * nx + np.clip((x / C).astype(int), 0, nx - 1)
    adj = FloorAdjustment(x, y, fi, cell, (ny, nx), cell_xy, frame_xy)

    c, a, b = rng.normal(0, 0.003, 220), rng.normal(0, 0.003, 220), rng.normal(0, 0.003, 220)
    nuis = c[fi] + a[fi] * (x - frame_xy[fi, 0]) + b[fi] * (y - frame_xy[fi, 1])
    noise = rng.normal(0, 0.002, x.size)
    h0 = adj.solve(nuis + noise)
    ok = ~np.isnan(h0)
    assert np.nanstd(h0[ok]) < 0.0006, np.nanstd(h0[ok])            # 평평한 바닥(프레임 평면 오차만 있음)이 평평하게 복원됨

    R, hgt = 1.5, 0.02
    d = np.hypot(x - 4.5, y - 4.5)
    dome = np.where(d < R, hgt / 2 * (1 + np.cos(np.pi * d / R)), 0.0)
    h1 = adj.solve(nuis + noise + dome)
    dc = np.hypot(cell_xy[:, 0] - 4.5, cell_xy[:, 1] - 4.5)
    pc = np.where(dc < R, hgt / 2 * (1 + np.cos(np.pi * dc / R)), 0.0)
    m = (pc > 0) & ok
    gain_adj = (((h1 - h0)[m]) * pc[m]).sum() / (pc[m] ** 2).sum()

    res = np.empty(x.size)
    for f in range(220):
        s = fi == f
        res[s] = frame_plane_residual(x[s], y[s], (nuis + noise + dome)[s], *frame_xy[f])
    res0 = np.empty(x.size)
    for f in range(220):
        s = fi == f
        res0[s] = frame_plane_residual(x[s], y[s], (nuis + noise)[s], *frame_xy[f])
    dd = res - res0
    cellsum = np.bincount(cell, dd, minlength=nx * ny) / np.maximum(np.bincount(cell, minlength=nx * ny), 1)
    gain_frame = (cellsum[m] * pc[m]).sum() / (pc[m] ** 2).sum()
    assert gain_adj > 0.8 and gain_adj > gain_frame + 0.1, (gain_adj, gain_frame)
    print(f"OK  평평한 바닥 복원 std {np.nanstd(h0[ok])*1e3:.2f} mm, 반경 {R} m 돔 복원 이득 조정 {gain_adj:.2f} / frame 평면 {gain_frame:.2f}")


if __name__ == '__main__':
    main()
