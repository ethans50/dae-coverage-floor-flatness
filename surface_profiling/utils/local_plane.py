# surface_profiling/utils/local_plane.py
"""국소 평면 대비 편차(잔차) 계산.

평탄도를 "국소 평면 대비 편차"로 정의하면, 점마다 주변 점들에 맞춘 평면을 빼고 남는 값이
평탄도 신호가 됨. 평면의 "국소"를 어떻게 잡느냐에 따라 두 방식이 있음.

  - 프레임 단위(frame_plane_residual): 한 프레임의 점들에 평면 하나를 맞춤. 프레임마다 다른
    차체 자세 오차(roll/pitch)가 평면에 흡수되지만, 프레임 반경 안의 실제 완만한 기복도 같이 빠짐.
  - 공간 창 단위(window_plane_residual): 여러 프레임의 점을 map 좌표에서 모아, 점마다 주변
    window_m 지름 원 안의 점 전부로 평면을 맞춤. 프레임별 자세 오차는 서로 다른 프레임이 섞여
    평균되고, 창보다 짧은 기복만 남음.

두 방식 모두 시그마 클리핑으로 평면을 맞춰서, 평면에서 크게 벗어난 점(국소 결함)이 평면을
휘게 만들어 스스로를 지우는 것을 줄임. 잔차는 평면 적합에 쓰이지 않은 점에도 모두 계산함.
"""

import numpy as np
from scipy import ndimage


def _mad_sigma(res):
    return 1.4826 * np.median(np.abs(res - np.median(res)))


def fit_plane(x, y, z, n_iter=3, clip=2.5):
    """z = a*x + b*y + c 를 최소제곱으로 맞춤(x, y는 호출 측에서 원점 근처로 옮겨 둘 것).

    매 반복마다 |잔차| <= clip*sigma 인 점만 남겨 다시 맞춤. (a, b, c)를 반환함."""
    keep = np.ones(x.size, dtype=bool)
    coef = np.zeros(3)
    for _ in range(n_iter):
        A = np.c_[x[keep], y[keep], np.ones(keep.sum())]
        coef, *_ = np.linalg.lstsq(A, z[keep], rcond=None)
        res = z - (coef[0] * x + coef[1] * y + coef[2])
        new = np.abs(res) <= clip * max(_mad_sigma(res[keep]), 1e-6)
        if new.sum() < 10 or (new == keep).all():
            break
        keep = new
    return coef


def frame_plane_residual(x, y, z, cx, cy, min_points=300, clip=2.5):
    """한 프레임의 점(x, y, z)에서 프레임 평면을 뺀 잔차를 반환함. 점이 min_points 미만이면 NaN.

    (cx, cy)는 평면 좌표의 원점(프레임의 센서 위치)이며 수치 안정성을 위해서만 씀."""
    if x.size < min_points:
        return np.full(x.size, np.nan)
    u, v = x - cx, y - cy
    a, b, c = fit_plane(u, v, z, clip=clip)
    return z - (a * u + b * v + c)


def window_plane_residual(x, y, z, window_m=3.0, cell_m=0.25, clip=2.5, n_iter=2, min_points=500):
    """map 좌표 점 전체에서, 점마다 주변 window_m 지름 원 안의 점으로 맞춘 평면을 뺀 잔차를 반환함.

    성능을 위해 평면은 cell_m 격자 셀마다 한 번만 구하고, 셀 안의 점들은 그 셀의 평면을 공유함
    (창은 셀 중심 기준 지름 window_m 원이며 셀 모멘트의 합성곱으로 계산함). 창 안 점이
    min_points 미만인 셀의 점은 NaN임."""
    xm, ym = x.mean(), y.mean()
    u, v = (x - xm).astype(np.float64), (y - ym).astype(np.float64)
    u0, v0 = u.min(), v.min()
    ix, iy = ((u - u0) / cell_m).astype(int), ((v - v0) / cell_m).astype(int)
    nx, ny = ix.max() + 1, iy.max() + 1
    flat = iy * nx + ix
    uc = u0 + (np.arange(nx) + 0.5) * cell_m
    vc = v0 + (np.arange(ny) + 0.5) * cell_m
    UC, VC = np.meshgrid(uc, vc)

    rad = window_m / 2.0 / cell_m
    k = int(np.ceil(rad))
    yy, xx = np.mgrid[-k:k + 1, -k:k + 1]
    kern = ((xx ** 2 + yy ** 2) <= rad ** 2).astype(np.float64)

    def moments(w):
        feats = [np.ones_like(u), u, v, z.astype(np.float64), u * u, u * v, v * v, u * z, v * z]
        out = []
        for f in feats:
            s = np.bincount(flat, weights=f * w, minlength=nx * ny).reshape(ny, nx)
            out.append(ndimage.convolve(s, kern, mode='constant'))
        return out

    keep = np.ones(u.size)
    res = np.full(u.size, np.nan)
    for _ in range(n_iter + 1):
        n, Su, Sv, Sz, Suu, Suv, Svv, Suz, Svz = moments(keep)
        # 창 중심(UC, VC) 기준 모멘트로 환산
        c_u, c_v = Su - UC * n, Sv - VC * n
        c_uu = Suu - 2 * UC * Su + UC ** 2 * n
        c_vv = Svv - 2 * VC * Sv + VC ** 2 * n
        c_uv = Suv - UC * Sv - VC * Su + UC * VC * n
        c_uz, c_vz = Suz - UC * Sz, Svz - VC * Sz
        valid = n >= min_points
        M = np.zeros((ny, nx, 3, 3))
        M[..., 0, 0], M[..., 0, 1], M[..., 0, 2] = c_uu, c_uv, c_u
        M[..., 1, 0], M[..., 1, 1], M[..., 1, 2] = c_uv, c_vv, c_v
        M[..., 2, 0], M[..., 2, 1], M[..., 2, 2] = c_u, c_v, n
        M[~valid] = np.eye(3)
        rhs = np.stack([c_uz, c_vz, Sz], axis=-1)[..., None]
        rhs[~valid] = 0.0
        coef = np.linalg.solve(M, rhs)[..., 0]
        a, b, c0 = coef[iy, ix, 0], coef[iy, ix, 1], coef[iy, ix, 2]
        res = z - (a * (u - UC[iy, ix]) + b * (v - VC[iy, ix]) + c0)
        res[~valid[iy, ix]] = np.nan
        ok = ~np.isnan(res)
        thr = clip * max(_mad_sigma(res[ok & (keep > 0)]), 1e-6)
        keep = (ok & (np.abs(res) <= thr)).astype(np.float64)
    return res
