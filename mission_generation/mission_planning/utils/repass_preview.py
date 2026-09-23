# mission_generation/mission_planning/utils/repass_preview.py
"""
boundary repass(blind cone 보완)가 만들 지점을 planning 단계에서 px 단위로 근사함.

실행 측 `mission_execution/utils/boundary_repass.py`의 기하 규칙
(`_repass_distance_m`/`_offset_pose`)을 그대로 재현함 - planning과 실행이 같은
지점을 가리켜야 `final_path.json`의 transit 시작점이 실제 로봇 위치와 맞음.

`compute_adjusted_exit()`는 시각화뿐 아니라 Step3의 `current_pos`와
`_reorder_pendant_groups`의 허브 anchor에도 쓰여 `final_path.json` 자체를
바꾸므로, 여기 규칙을 고치면 실행 측도 같이 고쳐야 함.
"""

import numpy as np


def compute_adjusted_exit(raw_points, enable_boundary_repass,
                          boundary_repass_distance_m, map_resolution,
                          boundary_repass_max_segment_m=2.8):
    """coverage 노드 하나(raw_points)의 실제 물리적 exit 지점 - F2C
    스와스 자체의 마지막 점(raw_points[-1])이 아니라, 실행 시
    BoundaryRepassController.run_exit_repass가 되짚기를 마친 뒤 로봇이
    실제로 서 있게 될 위치(retrace 지점)를 반환함.

    boundary_repass.py의 _repass_distance_m/_offset_pose와 완전히
    동일한 기하 규칙을 따름: 마지막 다리(raw_points[-2:])의 heading을
    구하고, 왕복 거리는 설정값(boundary_repass_distance_m)과 그 다리
    길이의 90% 중 작은 쪽으로 clamp한 뒤, 그만큼 되짚어 물러난 지점을
    계산함. enable_boundary_repass가 꺼져 있거나, 다리가 없거나(단일
    점 방), clamp된 거리가 0.3m 미만이면(run_exit_repass 자신도 이 경우
    되짚기 없이 즉시 캡처를 끄므로) 원래 F2C 종료 지점을 그대로
    반환함 - 그 경우 로봇은 실제로 거기 그대로 있기 때문임.

    Step3의 current_pos(다음 노드로 가는 transit A*의 실제 시작점)와
    _reorder_pendant_groups의 허브 anchor 양쪽에서 재사용함 - 로봇이
    실제로 그 위치에서 다음 이동을 시작하므로, 오프라인 planning(및 그
    시각화)도 거기서부터 transit을 그려야 실제 주행과 일치함."""
    if not raw_points:
        return None
    if not enable_boundary_repass or len(raw_points) < 2:
        return raw_points[-1]

    a = np.array(raw_points[-2], dtype=float)
    b = np.array(raw_points[-1], dtype=float)
    vec = b - a
    seg_len_px = float(np.hypot(vec[0], vec[1]))
    if seg_len_px < 1e-6:
        return raw_points[-1]
    if seg_len_px * map_resolution > boundary_repass_max_segment_m:
        return raw_points[-1]

    d_px = boundary_repass_distance_m / map_resolution
    d = min(d_px, seg_len_px * 0.9)
    if d * map_resolution < 0.3:
        return raw_points[-1]

    unit = vec / seg_len_px
    retrace = b - unit * d
    return (int(round(retrace[0])), int(round(retrace[1])))


def build_preview(path_segments, enable_boundary_repass,
                  boundary_repass_distance_m, map_resolution,
                  boundary_repass_max_segment_m=2.8):
    """미션 실행 시 BoundaryRepassController가 만들 왕복 경로를 planning
    단계에서 근사해 시각화 전용으로 반환함. boundary_repass.py의 기하
    규칙(_offset_pose/_repass_distance_m)을 px 단위로 그대로 재현함.
    path_segments(=실제 final_path.json 원본)는 건드리지 않음.

    heading 계산 시 주의: path_segments의 'coverage' 항목 하나는
    F2C 스와스 전부(꺾이는 코너 포함)를 이어붙인 좌표 목록이라, 여러
    스와스가 꺾여있는 노드는 path[0]->path[-1] 전체 직선(코너 무시한
    거시적 방향)이 실제 로봇이 그 시작/끝 지점에서 나아가는 방향과 전혀
    다를 수 있음. mission_executor.py의
    실제 run_start_prepass/run_exit_repass는 heading 변화 기준으로
    분할된 sub-segment(첫/마지막 직선 다리 하나)만 넘겨받으므로 이 문제가
    없음 - 여기서도 동일하게 첫 다리(path[0]->path[1])/마지막 다리
    (path[-2]->path[-1])만으로 heading과 길이(clamp 기준)를 계산해서
    맞춤. 앵커 지점(p0/p_end) 자체는 그대로 path[0]/path[-1] 사용.
    """
    preview_segments = []
    if not path_segments:
        return preview_segments

    d_m = boundary_repass_distance_m
    d_px = d_m / map_resolution

    def _unit_and_len(path):
        p0 = np.array(path[0], dtype=float)
        p1 = np.array(path[-1], dtype=float)
        vec = p1 - p0
        length = float(np.hypot(vec[0], vec[1]))
        if length < 1e-6:
            return None, 0.0
        return vec / length, length

    print("[*] Boundary repass preview:")

    first_seg = path_segments[0]
    if not enable_boundary_repass:
        print("    [start prepass] SKIPPED - enable_boundary_repass is false.")
    elif first_seg['type'] == 'coverage' and len(first_seg['path']) >= 2:
        unit, seg_len_px = _unit_and_len(first_seg['path'][:2])  # 첫 다리만
        if unit is not None:
            d = min(d_px, seg_len_px * 0.9)
            p0 = np.array(first_seg['path'][0], dtype=float)
            runway = p0 + unit * d
            # 화살표는 실제 로봇 이동 순서(runway -> p0)를 나타내야 함 -
            # run_start_prepass는 로봇이 runway 지점에서 스폰되어 p0로
            # 들어가는 편도 주행이므로, coverage 진행 방향(p0->runway 방향인
            # unit)과는 반대 방향으로 그려야 맞음. exit repass 화살표(아래,
            # p_end->retrace)와 동일한 "모션 시작점->끝점" 관례를 따름.
            preview_segments.append({
                'type': 'repass_preview',
                'path': [tuple(runway.astype(int)), tuple(p0.astype(int))],
                'label': f'START HERE ({d * map_resolution:.2f}m)',
                'label_at': 0,  # runway 지점(실제 로봇을 놔야 하는 곳)에 라벨을 붙임 -
                                # p0는 이미 순번 "1"이 찍혀있어서 그쪽에 붙이면 안 보임.
            })
            print(f"    [start prepass] mission start (path_segments[0]) - "
                  f"runway {d * map_resolution:.2f}m")
        else:
            print("    [start prepass] SKIPPED - start coverage segment has zero length.")
    else:
        print("    [start prepass] SKIPPED - path_segments[0] is not type='coverage'.")

    n_coverage_exits = 0
    n_previewed = 0
    for idx, seg in enumerate(path_segments):
        if seg['type'] != 'coverage':
            continue
        n_coverage_exits += 1
        is_mission_end = (idx == len(path_segments) - 1)
        tag = f"coverage exit #{n_coverage_exits} (path_segments[{idx}]" \
              f"{', mission end' if is_mission_end else ''})"

        if len(seg['path']) < 2:
            print(f"    [exit repass] {tag} SKIPPED - single-point coverage, no heading to retrace along.")
            continue

        # _compute_repass_adjusted_exit와 완전히 동일한 계산을 재사용함
        # (Step3의 current_pos 갱신이 실제로 쓰는 바로 그 함수) - 이 함수와
        # 별개로 공식을 중복 구현하면 enable_boundary_repass=False일 때도
        # 그 사실을 모른 채 무조건 화살표를 그리는 불일치가 생기므로 통일함.
        # path_segments 자체가 이미 이 지점에서 시작하므로, 여기서는
        # "coverage 끝점 -> 그 실제 시작점" 구간만 시각적으로 이어주면 됨.
        p_end = np.array(seg['path'][-1], dtype=float)
        retrace_raw = compute_adjusted_exit(seg['path'], enable_boundary_repass,
                                            boundary_repass_distance_m, map_resolution,
                                            boundary_repass_max_segment_m)
        if retrace_raw is None or tuple(retrace_raw) == tuple(seg['path'][-1]):
            print(f"    [exit repass] {tag} SKIPPED - no repass applied "
                  f"(disabled, segment longer than boundary_repass_max_segment_m, "
                  f"or clamped distance too short).")
            continue

        retrace = np.array(retrace_raw, dtype=float)
        d_m = float(np.hypot(*(p_end - retrace))) * map_resolution
        preview_segments.append({
            'type': 'repass_preview',
            'path': [tuple(p_end.astype(int)), tuple(retrace.astype(int))],
            'label': f'exit repass #{n_coverage_exits} ({d_m:.2f}m)',
            'label_at': 1,  # retrace 지점(되짚어 나가야 하는 곳)에 라벨
        })
        n_previewed += 1
        print(f"    [exit repass] {tag} - retrace {d_m:.2f}m")

    print(f"[*] Boundary repass preview summary: {n_previewed}/{n_coverage_exits} "
          f"coverage exits got a repass preview (rest skipped as logged above).")

    return preview_segments
