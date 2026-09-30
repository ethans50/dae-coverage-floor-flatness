# surface_profiling/utils/ring_coverage.py
"""VLP-16 ring 기하로 궤적이 각 바닥 셀을 몇 번 스치는지 계산함.

바닥 점 x가 관측되려면, 궤적을 따라 센서-점 거리 d(s)가 어떤 ring 반경 r_k와
만나야 함(d(s)가 연속함수이므로 min_s d(s) <= r_k <= max_s d(s)를 만족하는 k가
하나라도 있으면, 궤적 위 어느 시점에 정확히 그 거리를 지났다는 뜻). ring이
지나가는 것과 실제로 그 위치에 점이 찍히는 것 사이엔 방위각 샘플링 간격만큼의
확률적 간격이 있음 - 이 모듈은 "ring이 지나가는가"만 계산하고, 실측 점 수와의
비교는 analyze_ring_coverage.py에서 함.
"""

import numpy as np

# VLP-16 16채널(-15~+15도, 2도 간격) 중 바닥까지 도달이 실측으로 확인된
# 5개 하향 빔 - 마운트 높이에서 h/tan(각도)로 만드는 반경이 실측 ring 반경과
# 일치함을 확인함.
VLP16_FLOOR_BEAM_ANGLES_DEG = (15.0, 13.0, 11.0, 9.0, 7.0)


def vlp16_ring_radii_m(mount_height_m, angles_deg=VLP16_FLOOR_BEAM_ANGLES_DEG):
    """마운트 높이에서 지정 하향 빔 각도들이 만드는 바닥 ring 반경(m)을 반환함."""
    angles = np.asarray(angles_deg, dtype=np.float64)
    return mount_height_m / np.tan(np.radians(angles))


def downsample_trajectory(xy, min_step_m):
    """궤적 점을 앞점에서 min_step_m 이상 움직였을 때만 남겨 개수를 줄임.

    d(s)의 min/max는 거리가 촘촘히 바뀌는 구간보다 궤적이 셀에 가장 가깝고/먼
    지점에서 결정되므로, 이 정도 성긴 샘플링으로도 min/max 자체는 거의 그대로
    보존됨(원 궤적이 각지지 않고 완만하다는 전제)."""
    if len(xy) == 0:
        return xy
    keep = [0]
    last = xy[0]
    for i in range(1, len(xy)):
        if np.hypot(*(xy[i] - last)) >= min_step_m:
            keep.append(i)
            last = xy[i]
    return xy[keep]


def compute_ring_pass_count(cell_centers_xy, traj_xy, ring_radii_m, batch_size=1500):
    """cell_centers_xy(N,2), traj_xy(M,2) -> 셀별 관측 가능 ring 개수 n(x) (N,) int32.

    메모리를 억제하기 위해 셀을 batch_size개씩 나눠 (batch, M) 거리 행렬을 만듦."""
    n = cell_centers_xy.shape[0]
    counts = np.zeros(n, dtype=np.int32)
    radii = np.asarray(ring_radii_m, dtype=np.float64)
    for start in range(0, n, batch_size):
        end = min(start + batch_size, n)
        diff = cell_centers_xy[start:end, None, :] - traj_xy[None, :, :]
        dist = np.hypot(diff[..., 0], diff[..., 1])
        dmin = dist.min(axis=1)
        dmax = dist.max(axis=1)
        hits = (dmin[:, None] <= radii[None, :]) & (radii[None, :] <= dmax[:, None])
        counts[start:end] = hits.sum(axis=1)
    return counts


def build_candidate_grid(traj_xy, grid_m, margin_m):
    """궤적 bounding box + margin_m 범위를 grid_m 간격으로 채운 셀 중심 좌표와
    절대 정수 셀 인덱스(analyze_frame_log.compute_cell_stats와 동일한
    floor(x/grid) 규칙)를 반환함. 반환: (centers_xy (N,2), cell_ix (N,), cell_iy (N,))."""
    x_min, y_min = traj_xy.min(axis=0) - margin_m
    x_max, y_max = traj_xy.max(axis=0) + margin_m
    ix0, ix1 = int(np.floor(x_min / grid_m)), int(np.floor(x_max / grid_m))
    iy0, iy1 = int(np.floor(y_min / grid_m)), int(np.floor(y_max / grid_m))
    ix, iy = np.meshgrid(np.arange(ix0, ix1 + 1), np.arange(iy0, iy1 + 1), indexing='ij')
    ix, iy = ix.ravel(), iy.ravel()
    centers = np.stack([(ix + 0.5) * grid_m, (iy + 0.5) * grid_m], axis=1)
    return centers, ix, iy
