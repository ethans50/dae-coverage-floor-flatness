#!/usr/bin/env python3
# surface_profiling/test/check_local_plane.py
"""
utils/local_plane.py 의 합성 데이터 검증. 정답을 아는 데이터(평면 + 원형 돌출)로 확인함.

  1) fit_plane 이 노이즈 없는 평면을 정확히 복원하는가
  2) 프레임 평면 / 창 평면이 임의 기울기(프레임별 자세 오차)를 지우는가
  3) 창 지름보다 작은 돌출은 남기고(보존율 높음), 창보다 큰 완만한 기복은 지우는가(보존율 낮음)

사용 예:
  python3 check_local_plane.py
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from utils.local_plane import fit_plane, frame_plane_residual, window_plane_residual  # noqa: E402


def retention(res, x, y, cx, cy, radius, height):
    d = np.hypot(x - cx, y - cy)
    core, ring = res[d < 0.7 * radius], res[(d > 1.2 * radius) & (d < 1.8 * radius)]
    return (np.nanmean(core) - np.nanmean(ring)) / height


def main():
    rng = np.random.default_rng(0)

    # 1) 평면 복원
    x, y = rng.uniform(-2, 2, 2000), rng.uniform(-2, 2, 2000)
    a, b, c = fit_plane(x, y, 0.003 * x - 0.002 * y + 0.01)
    assert np.allclose([a, b, c], [0.003, -0.002, 0.01], atol=1e-9), (a, b, c)

    # 2) 프레임별로 다른 기울기 + 노이즈. 프레임 평면은 기울기를 지움
    cx, cy = 3.0, 3.0
    xs, ys, zs, res_f = [], [], [], []
    for _ in range(60):
        px, py = rng.uniform(1.5, 4.5, 2)
        ang = rng.uniform(0, 2 * np.pi, 4000); r = rng.uniform(1.1, 2.45, 4000)
        fx, fy = px + r * np.cos(ang), py + r * np.sin(ang)
        tilt = rng.normal(0, 0.0025, 2)                       # 프레임별 roll/pitch 오차(약 0.14도)
        z = tilt[0] * (fx - px) + tilt[1] * (fy - py) + rng.normal(0, 0.002, fx.size)
        xs.append(fx); ys.append(fy); zs.append(z)
        res_f.append(frame_plane_residual(fx, fy, z, px, py))
    res_f = np.concatenate(res_f)
    assert np.nanstd(res_f) < 0.0025, np.nanstd(res_f)       # 노이즈 0.002 + 평면 적합 여유

    # 3) 창 평면: 돌출 보존율의 크기 의존성
    x, y = rng.uniform(0, 6, 400000), rng.uniform(0, 6, 400000)
    base = rng.normal(0, 0.002, x.size)
    h = 0.01
    out = {}
    for radius in (0.3, 3.0):
        z = base + h * (np.hypot(x - 3, y - 3) < radius)
        out[radius] = retention(window_plane_residual(x, y, z, window_m=3.0), x, y, 3, 3, radius, h) \
            if radius < 1.0 else None
    assert out[0.3] > 0.8, out[0.3]                           # 창보다 작은 돌출은 거의 남음
    z = base + h * np.exp(-((x - 3) ** 2 + (y - 3) ** 2) / (2 * 1.5 ** 2))
    peak = window_plane_residual(x, y, z, window_m=3.0)[np.hypot(x - 3, y - 3) < 0.3]
    assert np.nanmean(peak) < 0.5 * h, np.nanmean(peak)       # 창과 비슷한 크기의 완만한 기복은 크게 줄어듦
    print(f"OK  frame 잔차 std {np.nanstd(res_f)*1e3:.2f} mm, 작은 돌출 보존율 {out[0.3]:.2f}, "
          f"완만한 기복 잔존 {np.nanmean(peak)/h:.2f}")


if __name__ == '__main__':
    main()
