# surface_profiling/test/test_wall_alignment.py
"""wall_alignment.estimate_front_wall_angle_error의 합성 데이터 검증.
실제 라이다 부호 확인은 auto_calibration_drive.py --detect-only로 별도 진행함."""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from utils.wall_alignment import estimate_front_wall_angle_error  # noqa: E402


def make_wall_points(distance, tilt_deg, length=4.0, n=2000, noise_std=0.01, seed=0):
    """센서 앞 distance[m]에, 센서 좌우축(+y) 기준 tilt_deg만큼 기울어진
    가상의 평면 벽에서 나온 점을 만듦."""
    rng = np.random.default_rng(seed)
    t = rng.uniform(-length / 2, length / 2, n)
    tilt = np.radians(tilt_deg)
    x = distance + t * np.sin(tilt) + rng.normal(0, noise_std, n)
    y = t * np.cos(tilt) + rng.normal(0, noise_std, n)
    z = rng.uniform(-0.1, 0.3, n)
    return np.c_[x, y, z].astype(np.float32)


def main():
    for tilt_deg in (0.0, 5.0, -5.0, 15.0):
        pts = make_wall_points(distance=1.5, tilt_deg=tilt_deg)
        angle_error, n = estimate_front_wall_angle_error(pts)
        assert angle_error is not None, f"tilt={tilt_deg}: not enough points (n={n})"
        est_deg = np.degrees(angle_error)
        print(f"true tilt={tilt_deg:+.1f} deg -> estimated angle_error={est_deg:+.2f} deg (n={n})")
        assert abs(est_deg - tilt_deg) < 1.0, f"error too large: true {tilt_deg}, estimated {est_deg}"

    # 점이 거의 없는 경우 None을 반환하는지 확인
    empty = np.zeros((10, 3), dtype=np.float32)
    angle_error, n = estimate_front_wall_angle_error(empty)
    assert angle_error is None and n == 0

    print("[OK] wall_alignment synthetic-data check passed")


if __name__ == '__main__':
    main()
