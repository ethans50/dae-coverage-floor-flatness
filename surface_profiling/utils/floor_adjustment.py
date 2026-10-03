# surface_profiling/utils/floor_adjustment.py
"""바닥 높이장과 프레임별 평면을 함께 추정함(겹치는 관측을 이용한 조정).

프레임마다 자세/높이 오차는 평면 하나(높이 c, 기울기 a, b)로 모델링함. 프레임별 평면 적합(local_plane.py)은 각
프레임의 평면을 따로 빼므로 프레임 반경보다 큰 실제 기복도 같이 빠짐. 여기서는 바닥 높이 h(셀)와 프레임별 평면을

    셀 평균 z(프레임 f, 셀 c) = h[c] + a_f*dx + b_f*dy + c_f      (dx, dy: 셀 평균 위치 - 프레임 센서 위치)

로 함께 최소제곱 추정함. 프레임 평면이 자유 변수이면 해가 정해지지 않는 방향은 겹침이 연결된 영역의 전역 평면뿐이므로,
프레임 반경보다 긴 기복도 겹침을 통해 식별됨. 전역 평면(상수, 기울기 2개)은 식별 불가라 h 의 평균과 x, y 모멘트를 0 으로
고정함. 관측이 드문 셀은 이웃 셀과의 차이에 약한 벌점(smooth)을 줌.

기하(행렬)는 한 번만 만들고 z 만 바꿔 여러 번 풀 수 있음. 직전 해를 초기값으로 씀.
"""

import numpy as np
import scipy.sparse as sp
from scipy.sparse.linalg import lsmr


class FloorAdjustment:
    def __init__(self, x, y, frame_idx, cell, grid_shape, cell_xy, frame_xy,
                 smooth=0.2, gauge=1e3, max_count=30):
        """x, y: 점 좌표. frame_idx: 점의 프레임 번호(0..F-1). cell: 점의 셀 번호(iy*nx+ix).
        grid_shape: (ny, nx). cell_xy: (ny*nx, 2) 셀 중심. frame_xy: (F, 2) 프레임 센서 위치.
        max_count: (프레임, 셀) 관측의 가중치 sqrt(min(점 수, max_count))의 상한."""
        ny, nx = grid_shape
        H, F = ny * nx, len(frame_xy)
        key = frame_idx.astype(np.int64) * H + cell
        uk, self.inv = np.unique(key, return_inverse=True)
        self.n = np.bincount(self.inv).astype(np.float64)
        of, oc = uk // H, uk % H
        dx = np.bincount(self.inv, x) / self.n - frame_xy[of, 0]
        dy = np.bincount(self.inv, y) / self.n - frame_xy[of, 1]
        self.w = np.sqrt(np.minimum(self.n, max_count))
        k = np.arange(uk.size)
        A = sp.coo_matrix(
            (np.r_[self.w, self.w * dx, self.w * dy, self.w],
             (np.r_[k, k, k, k], np.r_[oc, H + 3 * of, H + 3 * of + 1, H + 3 * of + 2])),
            shape=(uk.size, H + 3 * F)).tocsr()

        self.observed = np.bincount(oc, minlength=H) > 0
        idx = np.arange(H).reshape(ny, nx)
        edges = np.r_[np.c_[idx[:, :-1].ravel(), idx[:, 1:].ravel()], np.c_[idx[:-1].ravel(), idx[1:].ravel()]]
        edges = edges[self.observed[edges[:, 0]] & self.observed[edges[:, 1]]]
        r = np.arange(len(edges))
        S = sp.coo_matrix((np.r_[np.full(len(edges), smooth), np.full(len(edges), -smooth)],
                           (np.r_[r, r], np.r_[edges[:, 0], edges[:, 1]])), shape=(len(edges), H + 3 * F)).tocsr()

        ob = np.nonzero(self.observed)[0]
        xc, yc = cell_xy[ob, 0] - cell_xy[ob, 0].mean(), cell_xy[ob, 1] - cell_xy[ob, 1].mean()
        G = sp.vstack([
            sp.coo_matrix((np.full(ob.size, gauge / ob.size), (np.zeros(ob.size, int), ob)), shape=(1, H + 3 * F)),
            sp.coo_matrix((xc * gauge / ob.size, (np.zeros(ob.size, int), ob)), shape=(1, H + 3 * F)),
            sp.coo_matrix((yc * gauge / ob.size, (np.zeros(ob.size, int), ob)), shape=(1, H + 3 * F)),
        ])
        self.M = sp.vstack([A, S, G]).tocsr()
        self.n_rest = S.shape[0] + 3
        self.H = H
        self.x = np.zeros(H + 3 * F)

    def solve(self, z):
        """점 z[m]에 대한 셀 높이 h[m](관측 없는 셀은 NaN)를 반환함. z 는 생성자의 점과 같은 순서임."""
        mz = np.bincount(self.inv, z) / self.n * 1e3                      # mm 단위로 풂
        rhs = np.r_[self.w * mz, np.zeros(self.n_rest)]
        self.x = lsmr(self.M, rhs, atol=1e-10, btol=1e-10, maxiter=5000, x0=self.x)[0]
        h = np.full(self.H, np.nan)
        h[self.observed] = self.x[:self.H][self.observed] / 1e3
        return h
