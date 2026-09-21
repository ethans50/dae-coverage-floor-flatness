# surface_profiling/utils/coverage_metrics.py
"""
저장된 PCD/CSV에서 커버리지 비교용 지표를 계산하는 순수 함수 모음.

`analyze_coverage_comparison.py`(CLI)가 입력 경로를 정리해 넘겨주면 여기서
완전성/gap/셀당 z-표준편차/셀당 리턴 수와 거리·시간·회전량을 계산함. 로봇이나
ROS에 의존하지 않아 이미 저장된 데이터만으로 반복 재계산 가능함 - 지표 정의가
바뀌어도 `combined_*.pcd`만 있으면 재주행이 필요 없는 구조임.

완전성의 분모는 맵 전체 free-space가 아니라 커버리지 노드의 합집합임(transit 전용
영역은 설계상 측정하지 않기 때문).
"""

import os
import csv
import math

import numpy as np
import cv2
import open3d as o3d


# ----------------------------------------------------------------------
# 2D 맵 free-space 마스크 로딩
# ----------------------------------------------------------------------

def _load_free_space_mask(map_yaml_path):
    """map_yaml_path(ROS map_server 포맷)를 읽어 (free_mask, resolution,
    origin_x, origin_y)를 반환함. free_mask[row, col]=True는 주행 가능한
    자유공간 픽셀이고, origin은 이미지 '왼쪽 아래' 픽셀이 world 좌표
    (origin_x, origin_y)에 대응하는 ROS map_server 규격임.

    free-space 판정 임계값(그레이스케일 250/255 초과)은
    environment_modeling/algorithms/limits.py(map_preprocessor.py 계열)가
    쓰는 것과 동일한 기준을 재사용함 - 서로 다른 임계값을 쓰면 완전성
    지표가 흔들릴 수 있음. yaw!=0인 맵은 heatmap_generator.py와 동일하게
    미지원임(경고만 출력)."""
    import yaml

    with open(map_yaml_path, 'r') as f:
        meta = yaml.safe_load(f)

    image_path = meta['image']
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(map_yaml_path), image_path)

    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"[-] Map image not found: {image_path}")

    # heatmap_generator._load_occupancy_map과 동일하게, origin='lower'
    # 좌표계와 맞추기 위해 상하 반전시킴
    img = np.flipud(img)
    _, free_binary = cv2.threshold(img, 250, 255, cv2.THRESH_BINARY)
    free_mask = free_binary > 0

    resolution = meta['resolution']
    origin = meta.get('origin', [0.0, 0.0, 0.0])
    yaw = origin[2] if len(origin) > 2 else 0.0
    if abs(yaw) > 1e-6:
        print(f"[!] Warning: map origin yaw={yaw}(rad) != 0 - free-space mask misalignment risk.")

    return free_mask, resolution, origin[0], origin[1]


# ----------------------------------------------------------------------
# 완전성/gap/z-표준편차/리턴 수 지표
# ----------------------------------------------------------------------

def _load_target_area_mask(topology_npz_path, free_mask):
    """커버리지 '측정 대상' 영역 마스크를 돌려줌 - final_topological_map.npz의
    노드 마스크 합집합과 free-space의 교집합임(없으면 None).

    완전성의 분모를 맵 전체 free-space로 두면, 애초에 측정 대상이 아닌
    영역(노드로 분할되지 않은 복도 등 - transit은 전부 record_pcd=False라
    설계상 측정하지 않음)까지 분모에 들어가 완전성이 구조적으로 과소평가됨.
    (맵 전체 free-space 기준으로는 모든 노드를 완벽히 채워도 상한이 약 30%임.)
    """
    if topology_npz_path is None or not os.path.exists(topology_npz_path):
        return None

    data = np.load(topology_npz_path, allow_pickle=True)
    if 'nodes' not in data:
        return None

    nodes = data['nodes']
    if len(nodes) == 0:
        return None

    union = np.zeros(nodes[0].shape, dtype=bool)
    for node in nodes:
        union |= node.astype(bool)

    # free_mask는 _load_free_space_mask에서 flipud된 상태(row=y)이고 npz의
    # 노드 마스크는 원본 이미지 방향이라 동일하게 뒤집어 맞춤.
    return np.flipud(union) & free_mask


def compute_coverage_metrics(pcd_path, raw_pcd_path, map_yaml_path, z_min, z_max, grid_size,
                             topology_npz_path=None):
    """완전성/gap/셀당 z-표준편차/셀당 리턴 수 지표를 계산해 dict로 반환함.

    pcd_path(다운샘플본)로 완전성/gap을 계산하고, raw_pcd_path(선택,
    다운샘플 이전 원본)가 주어지면 셀당 다중 리턴 수/z-표준편차도 계산함.
    두 계산 모두 map_yaml_path에서 얻은 동일한 원점/해상도 기준 격자에
    정렬되므로, 서로 다른 시점에 실행한 알고리즘 3개의 결과가 같은 물리적
    셀 경계를 공유해 비교 가능함."""
    free_mask, map_res, ox, oy = _load_free_space_mask(map_yaml_path)
    map_h, map_w = free_mask.shape  # flipud된 상태 - row는 y, col은 x에 대응함

    x0, y0 = ox, oy
    nx = int(np.ceil(map_w * map_res / grid_size))
    ny = int(np.ceil(map_h * map_res / grid_size))

    # 분석 격자 각 셀 중심의 world 좌표를 맵 픽셀 인덱스로 최근접 변환해
    # free_mask를 분석 격자 해상도로 옮김. 이 프로젝트의 맵 해상도
    # (target_resolution 기본 0.02m)가 grid_size(기본 0.01m)보다 성기므로
    # 실질적으로 항상 업샘플링에 해당함 - 반대 상황(다운샘플링)이면 얇은
    # 통로가 사라지는 aliasing이 생길 수 있어 grid_size를 map_res보다
    # 작게 유지할 것.
    cell_x = (np.arange(nx) + 0.5) * grid_size + x0
    cell_y = (np.arange(ny) + 0.5) * grid_size + y0
    col_idx = np.clip(((cell_x - ox) / map_res).astype(np.int64), 0, map_w - 1)
    row_idx = np.clip(((cell_y - oy) / map_res).astype(np.int64), 0, map_h - 1)
    analysis_free = free_mask[np.ix_(row_idx, col_idx)].T  # (nx, ny)
    total_valid_cells = int(analysis_free.sum())

    # 완전성의 기본 분모는 '측정 대상 영역'(커버리지 노드 합집합)임. 토폴로지를
    # 못 찾으면 맵 전체 free-space로 폴백함.
    target_mask_full = _load_target_area_mask(topology_npz_path, free_mask)
    if target_mask_full is not None:
        analysis_target = target_mask_full[np.ix_(row_idx, col_idx)].T
        target_source = 'coverage_nodes'
    else:
        analysis_target = analysis_free
        target_source = 'map_free_space'
    total_target_cells = int(analysis_target.sum())

    def _to_grid_indices(x, y):
        """x,y(world)를 분석 격자 인덱스로 변환함 - 격자 밖으로 벗어나는
        점은 valid 마스크로 걸러내고, z처럼 함께 골라내야 하는 다른 배열이
        있을 때 이 valid를 그대로 재사용하도록 반환함."""
        ix = np.floor((x - x0) / grid_size).astype(np.int64)
        iy = np.floor((y - y0) / grid_size).astype(np.int64)
        valid = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
        return ix[valid], iy[valid], valid

    # --- 완전성/gap: 다운샘플본 기준 ---
    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points)
    total_downsampled_points = int(points.shape[0])

    floor_mask = (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
    floor_pts = points[floor_mask]
    total_downsampled_floor_points = int(floor_pts.shape[0])

    ix, iy, _ = _to_grid_indices(floor_pts[:, 0], floor_pts[:, 1])
    covered = np.zeros((nx, ny), dtype=bool)
    covered[ix, iy] = True

    covered_and_target = covered & analysis_target
    covered_cell_count = int(covered_and_target.sum())
    completeness_ratio = covered_cell_count / total_target_cells if total_target_cells > 0 else 0.0

    # 맵 전체 free-space 기준값도 함께 남김 - 이 기준으로 계산된
    # 맵 전체 기준으로 계산한 결과와 비교가 가능해야 함.
    covered_mapwide = int((covered & analysis_free).sum())
    completeness_ratio_mapwide = covered_mapwide / total_valid_cells if total_valid_cells > 0 else 0.0

    gap_mask = analysis_target & ~covered
    gap_cell_count = int(gap_mask.sum())
    gap_area_m2 = gap_cell_count * (grid_size ** 2)

    result = {
        'grid_size_m': grid_size,
        'z_min': z_min,
        'z_max': z_max,
        'completeness_target_source': target_source,
        'total_target_cells': total_target_cells,
        'target_area_m2': total_target_cells * (grid_size ** 2),
        'total_valid_cells': total_valid_cells,
        'covered_cell_count': covered_cell_count,
        'completeness_ratio': completeness_ratio,
        'covered_cell_count_mapwide': covered_mapwide,
        'completeness_ratio_mapwide': completeness_ratio_mapwide,
        'gap_cell_count': gap_cell_count,
        'gap_area_m2': gap_area_m2,
        'total_downsampled_points': total_downsampled_points,
        'total_downsampled_floor_points': total_downsampled_floor_points,
        'raw_stats': None,
    }

    # --- 셀당 z-표준편차/리턴 수: raw본이 있을 때만 ---
    if raw_pcd_path is not None:
        raw_pcd = o3d.io.read_point_cloud(raw_pcd_path)
        raw_points = np.asarray(raw_pcd.points)
        raw_floor_mask = (raw_points[:, 2] >= z_min) & (raw_points[:, 2] <= z_max)
        raw_floor = raw_points[raw_floor_mask]

        rix, riy, rvalid = _to_grid_indices(raw_floor[:, 0], raw_floor[:, 1])
        rz = raw_floor[rvalid, 2]

        keys = rix * ny + riy
        n_keys = nx * ny
        count = np.bincount(keys, minlength=n_keys)
        sum_z = np.bincount(keys, weights=rz, minlength=n_keys)
        sumsq_z = np.bincount(keys, weights=rz ** 2, minlength=n_keys)

        free_flat = analysis_free.reshape(-1)
        hit_and_free = free_flat & (count > 0)
        counts_at_hit = count[hit_and_free]

        multi_mask = free_flat & (count >= 2)
        mean_z = np.zeros(n_keys)
        mean_z[multi_mask] = sum_z[multi_mask] / count[multi_mask]
        var_z = np.zeros(n_keys)
        var_z[multi_mask] = np.clip(sumsq_z[multi_mask] / count[multi_mask] - mean_z[multi_mask] ** 2, 0, None)
        std_z = np.sqrt(var_z[multi_mask])

        result['raw_stats'] = {
            'total_raw_points': int(raw_points.shape[0]),
            'total_raw_floor_points': int(raw_floor.shape[0]),
            'cells_with_multi_return': int(multi_mask.sum()),
            'mean_cell_z_std_m': float(std_z.mean()) if std_z.size else None,
            'median_cell_z_std_m': float(np.median(std_z)) if std_z.size else None,
            'mean_returns_per_occupied_cell': float(counts_at_hit.mean()) if counts_at_hit.size else 0.0,
            'median_returns_per_occupied_cell': float(np.median(counts_at_hit)) if counts_at_hit.size else 0.0,
            'max_returns_per_occupied_cell': int(counts_at_hit.max()) if counts_at_hit.size else 0,
        }

    return result


# ----------------------------------------------------------------------
# 거리/시간 (robot_path_*.csv) 및 회전량 근사 (drive_debug_*.csv)
# ----------------------------------------------------------------------

def compute_path_stats(robot_path_csv):
    """robot_path_*.csv(header: timestamp,x,y)에서 총 이동거리(m)와 총
    소요시간(s)을 계산함. 이 CSV에는 orientation 컬럼이 없어 회전량은 여기서
    계산할 수 없음 - 근사가 필요하면 compute_rotation_from_drive_debug 참고."""
    rows = []
    with open(robot_path_csv, 'r') as f:
        reader = csv.reader(f)
        next(reader, None)  # 헤더 행(timestamp,x,y) 스킵
        for row in reader:
            rows.append((float(row[0]), float(row[1]), float(row[2])))
    rows.sort(key=lambda r: r[0])

    if len(rows) < 2:
        return {'total_distance_m': 0.0, 'total_time_sec': 0.0, 'num_samples': len(rows)}

    total_distance = 0.0
    for (_, x0, y0), (_, x1, y1) in zip(rows[:-1], rows[1:]):
        total_distance += math.hypot(x1 - x0, y1 - y0)
    total_time = rows[-1][0] - rows[0][0]

    return {
        'total_distance_m': total_distance,
        'total_time_sec': total_time,
        'num_samples': len(rows),
    }


def compute_stall_stats(stall_report_csv):
    """stall_report_<epoch>.csv(mission_execution/utils/stall_logger.py가
    저장, 1행은 "# stall threshold = ..." 주석)에서 이번 미션 동안 발생한
    stall들의 건수와 총 지속시간(초)을 계산함. total_time_sec에서 이 값을
    빼면 nav2 recovery 대기로 늘어난 시간을 제외한 "순수 주행
    시간"(active_time_sec)을 얻을 수 있음 - raw total_time_sec은 stall
    유무/길이 때문에 반복 실행 간 변동이 커서 조합 비교에 부적합함."""
    with open(stall_report_csv, 'r') as f:
        next(f)  # 1행 주석 스킵
        reader = csv.DictReader(f)
        stall_count = 0
        total_stalled_sec = 0.0
        for row in reader:
            total_stalled_sec += float(row['duration_sec'])
            stall_count += 1
    return {'stall_count': stall_count, 'total_stalled_sec': total_stalled_sec}


def compute_rotation_from_drive_debug(drive_debug_csv):
    """drive_debug_*.csv의 yaw_deg 컬럼으로 총 회전량(deg)을 근사함 - 정확한
    회전량 컬럼이 robot_path_*.csv에 없어 이걸로 대체하는 근사치임.
    drive_debug_interval_sec(기본 3초) 간격 샘플이라, 그 사이에 벌어지는
    빠른 회전(제자리 Spin 등)은 과소평가될 수 있음."""
    yaws = []
    with open(drive_debug_csv, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                yaws.append(float(row['yaw_deg']))
            except (KeyError, ValueError, TypeError):
                continue

    if len(yaws) < 2:
        return 0.0

    total_deg = 0.0
    for a, b in zip(yaws[:-1], yaws[1:]):
        diff = abs(b - a) % 360.0
        if diff > 180.0:
            diff = 360.0 - diff
        total_deg += diff
    return total_deg
