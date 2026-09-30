# mission_generation/mission_planning/utils/ring_coverage.py
"""VLP-16 ring 기하로 coverage exit 지점 바로 다음 구간이 이미 충분히
관측됐는지(그래서 되짚기가 불필요한지) 계산함.

실행 측 `mission_execution/utils/ring_coverage.py`와 완전히 동일한 계산을
재현함(mission_generation/mission_execution이 서로 import하지 않는 이 저장소의
관례를 따름, repass_preview.py 참고) - 두 쪽이 같은 지점에 대해 같은 판단을
내려야 planning된 exit 지점과 실행 시 되짚기 여부가 어긋나지 않음.

바닥 점 x가 관측되려면, 센서-점 거리 d(s)가 궤적을 따라 어떤 ring 반경 r_k와
만나야 함(d(s)가 연속함수이므로 min_s d(s) <= r_k <= max_s d(s)를 만족하는 k가
하나라도 있으면 그 ring이 실제로 그 지점을 지나간 것임). n(x)는 이를 만족하는
ring 개수임.
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


def compute_ring_pass_count(points_xy, traj_xy, ring_radii_m):
    """points_xy(N,2), traj_xy(M,2) -> 점별 관측 가능 ring 개수 n(x) (N,) int32."""
    radii = np.asarray(ring_radii_m, dtype=np.float64)
    diff = points_xy[:, None, :] - traj_xy[None, :, :]
    dist = np.hypot(diff[..., 0], diff[..., 1])
    dmin = dist.min(axis=1)
    dmax = dist.max(axis=1)
    hits = (dmin[:, None] <= radii[None, :]) & (radii[None, :] <= dmax[:, None])
    return hits.sum(axis=1).astype(np.int32)


def exit_point_ring_count(traj_xy, exit_xy, ring_radii_m):
    """exit 지점(exit_xy, 세그먼트 궤적의 마지막 점) 자체가 이 세그먼트 자체의
    궤적(traj_xy)만으로 몇 개의 ring에 관측되는지(n(x))를 반환함. exit 지점은
    궤적의 마지막 점이므로 dmin=0이 자명하고 dmax는 사실상 접근 구간의
    길이임 - 그래서 이 값은 "접근하며 이미 몇 개의 ring이 이 지점을 훑고
    지나갔는가"를 그대로 나타냄. 짧은 세그먼트일수록 dmax가 작아 큰 반경의
    ring이 아예 접근 범위 밖이 되므로 n(x)가 낮게 나와, 되짚기로 보완해야 할
    필요를 정확히 반영함."""
    traj_xy = np.asarray(traj_xy, dtype=np.float64)
    counts = compute_ring_pass_count(np.asarray(exit_xy, dtype=np.float64)[None, :], traj_xy, ring_radii_m)
    return int(counts[0])
