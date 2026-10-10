import math
import json

# 코너 앞 보조점이 코너에서 떨어진 거리(m): Nav2 도달 허용 오차(0.06 m)보다 크게 둠
CORNER_REACH_M = 0.1

def _get_heading(p1, p2):
    """두 점 사이의 진행 방향(각도)을 반환함."""
    x1, y1 = p1['pose']['position']['x'], p1['pose']['position']['y']
    x2, y2 = p2['pose']['position']['x'], p2['pose']['position']['y']
    return math.atan2(y2 - y1, x2 - x1)

def _distance(p1, p2):
    """두 점 사이의 유클리디안 거리를 반환함."""
    x1, y1 = p1['pose']['position']['x'], p1['pose']['position']['y']
    x2, y2 = p2['pose']['position']['x'], p2['pose']['position']['y']
    return math.hypot(x2 - x1, y2 - y1)

def _resample_single_segment(poses, seg_type, record_pcd, extra_header=None, reach_corners=False):
    """
    단일 세그먼트(coverage 스와스 하나, 또는 transit run 하나)에서 진짜
    꺾이는 지점(앵커)만 추출함. mission_planner.py가 A* 결과에
    simplify_path(Douglas-Peucker)를 이미 적용해 격자 지그재그를 제거해
    두므로, 여기 남는 점은 전부 실제로 의미 있는 코너임. 직선 중간의
    보간점(_midpoint)은 두지 않음 - 시작/꼭짓점/끝만으로 "지나쳐야
    채워진다" 원칙을 충분히 만족함이 실측 검증됨.

    세그먼트의 시작점과 끝점은 항상 원본 좌표 그대로 보존됨(보간 없음).

    extra_header: node_id(coverage)/from_node_id,to_node_id(transit) 등
    mission_executor.py의 실행 중 디버그 로그가 "지금 몇 번 노드를 처리
    중인지" 표시하는 데 쓰는 부가 정보. 있으면 모든 앵커 포인트의 header에
    그대로 병합됨.
    """
    extra_header = extra_header or {}
    if len(poses) == 0:
        return []

    if len(poses) == 1:
        p = json.loads(json.dumps(poses[0]))
        p['header'] = {'frame_id': 'map', 'task_type': f"{seg_type}_single", 'record_pcd': record_pcd, **extra_header}
        return [p]

    anchors = [poses[0]]
    for i in range(1, len(poses) - 1):
        p_prev, p_curr, p_next = poses[i - 1], poses[i], poses[i + 1]
        if _distance(p_prev, p_curr) < 1e-4 or _distance(p_curr, p_next) < 1e-4:
            continue
        theta1 = _get_heading(p_prev, p_curr)
        theta2 = _get_heading(p_curr, p_next)
        angle_diff = abs(theta1 - theta2)
        if angle_diff > math.pi:
            angle_diff = 2 * math.pi - angle_diff
        if angle_diff > 0.017:
            anchors.append(p_curr)
    anchors.append(poses[-1])

    sampled = []
    start = json.loads(json.dumps(anchors[0]))
    start['header'] = {'frame_id': 'map', 'task_type': f"{seg_type}_start", 'record_pcd': record_pcd, **extra_header}
    sampled.append(start)

    for i in range(len(anchors) - 1):
        A, B = anchors[i], anchors[i + 1]
        if _distance(A, B) < 1e-6:
            continue
        if i < len(anchors) - 2:
            if reach_corners and record_pcd and _distance(A, B) > 2 * CORNER_REACH_M:
                # 코너 B 바로 앞(들어오는 직선 위)에 점을 하나 더 둠. 실행기는 heading이 바뀌는 점에서 구간을
                # 나누고 그 점을 다음 구간의 첫 목표로 보내는데, Nav2가 반경 안에 든 중간 목표를 지워 코너를
                # 잘라버림. 이 점이 앞 구간의 마지막 목표가 되면 로봇이 코너 근처까지 가서 서므로 코너가 안 잘림.
                pre = json.loads(json.dumps(A))
                f = (_distance(A, B) - CORNER_REACH_M) / _distance(A, B)
                pre['pose']['position']['x'] = A['pose']['position']['x'] + (B['pose']['position']['x'] - A['pose']['position']['x']) * f
                pre['pose']['position']['y'] = A['pose']['position']['y'] + (B['pose']['position']['y'] - A['pose']['position']['y']) * f
                pre['header'] = {'frame_id': 'map', 'task_type': f"{seg_type}_turn", 'record_pcd': record_pcd, **extra_header}
                sampled.append(pre)
            turn_wp = json.loads(json.dumps(B))
            turn_wp['header'] = {'frame_id': 'map', 'task_type': f"{seg_type}_turn", 'record_pcd': record_pcd, **extra_header}
            sampled.append(turn_wp)

    end = json.loads(json.dumps(anchors[-1]))
    end['header'] = {'frame_id': 'map', 'task_type': f"{seg_type}_end", 'record_pcd': record_pcd, **extra_header}
    sampled.append(end)

    return sampled


def interpolate_with_semantics(translated_segments, reach_corners=False):
    all_sampled = []
    for seg in translated_segments:
        seg_type = seg['type']
        record_pcd = seg.get('record_pcd', seg_type == 'coverage')

        extra_header = {}
        if seg.get('node_id') is not None:
            extra_header['node_id'] = seg['node_id']
        if seg.get('from_node_id') is not None:
            extra_header['from_node_id'] = seg['from_node_id']
        if seg.get('to_node_id') is not None:
            extra_header['to_node_id'] = seg['to_node_id']

        seg_samples = _resample_single_segment(seg['poses'], seg_type, record_pcd, extra_header=extra_header, reach_corners=reach_corners)
        print(f"[*] Segment '{seg_type}': {len(seg['poses'])} original points -> {len(seg_samples)} anchor points")
        all_sampled.extend(seg_samples)

    print(f"[*] Anchor extraction completed: {len(translated_segments)} segments -> {len(all_sampled)} total points")

    return all_sampled
