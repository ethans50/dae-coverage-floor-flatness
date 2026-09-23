# surface_profiling/utils/wall_alignment.py
"""라이다 원시 점군(센서 좌표계)에서 정면 벽면의 각도 오차를 추정함.
auto_calibration_drive.py의 폐루프 회전 정렬이 이 오차를 0으로 미는 데 씀.
map/AMCL 없이 센서 자신의 좌표계(+x=전방, +y=좌측)만으로 동작함."""

import numpy as np


def estimate_front_wall_angle_error(points_xyz, fov_deg=70.0, z_min=-0.15, z_max=0.5,
                                     r_min=0.5, r_max=3.0, min_points=200):
    """정면(+x) 벽면이 로봇의 좌우축(+y)과 이루는 각도 오차(rad)를 반환함.

    양수면 벽이 반시계 방향으로 틀어져 있음(로봇을 양의 각속도로 돌려 상쇄).
    부호는 이론적으로 유도했을 뿐 실측 검증 전이므로, 실제 사용 전에
    auto_calibration_drive.py --detect-only로 로봇을 손으로 살짝 돌려보며
    부호가 맞는지 반드시 확인함.

    fov_deg: 정면 기준 좌우로 볼 각도 폭. 좁으면 옆벽/모서리 점 유입이 줄지만
        점 수가 줄어 잡음에 약해짐.
    z_min/z_max: 벽으로 볼 높이 범위(센서 기준, m). 바닥/천장 점 제외용.
    r_min/r_max: 벽으로 볼 거리 범위(m). 너무 가까우면 자기반사, 너무 멀면
        다른 벽/잡음이 섞임.

    반환: (angle_error_rad, 사용된 점 수). 점이 min_points 미만이면
    (None, 점 수)를 반환하므로 호출부는 회전 없이 재시도하거나 파라미터를
    넓혀야 함.
    """
    x, y, z = points_xyz[:, 0], points_xyz[:, 1], points_xyz[:, 2]
    r = np.hypot(x, y)
    az_deg = np.degrees(np.arctan2(y, x))  # 0=정면, +=좌측

    mask = (
        (z >= z_min) & (z <= z_max) &
        (r >= r_min) & (r <= r_max) &
        (np.abs(az_deg) <= fov_deg / 2.0)
    )
    pts = np.c_[x[mask], y[mask]]
    if pts.shape[0] < min_points:
        return None, pts.shape[0]

    # 간이 반복 이상치 제거: PCA 주축을 벽의 연장 방향으로 보고,
    # 벽에서 수직거리가 큰 점(모서리, 문틀, 다른 물체)을 순차적으로 걸러냄.
    wall_dir = np.array([0.0, 1.0])
    for _ in range(3):
        centroid = pts.mean(axis=0)
        centered = pts - centroid
        cov = centered.T @ centered
        eigvals, eigvecs = np.linalg.eigh(cov)
        wall_dir = eigvecs[:, np.argmax(eigvals)]
        normal = np.array([-wall_dir[1], wall_dir[0]])
        dist_to_wall = centered @ normal
        keep = np.abs(dist_to_wall) < (3.0 * dist_to_wall.std() + 1e-6)
        if keep.sum() < min_points:
            break
        pts = pts[keep]

    # wall_dir 부호는 고유벡터 특성상 임의이므로 +y 쪽으로 통일.
    if wall_dir[1] < 0:
        wall_dir = -wall_dir
    angle_error = float(np.arctan2(wall_dir[0], wall_dir[1]))
    return angle_error, pts.shape[0]
