import math
import os
import cv2
import numpy as np
import colorsys

# 픽셀 공간(viz_mask/topology, map_from_dae.pgm과 동일 크기·해상도·원점)의 축 방향과
# map frame(미터) 축 방향 대응 - ROS map_server 관례상 이미지 row 0이 위쪽이라
# row가 늘어날수록 map y는 줄어듦(위아래 반전). col은 그대로 map x와 같은 방향.
# 그래서 이미지 기준 오른쪽=+X(East), 위쪽=+Y(North)로 라벨링함 - 실제 나침반
# 방향이 아니라 이 맵의 x/y축 페어를 사람이 읽기 쉬운 이름으로 부르는 것뿐임.
_CARDINAL_PX_STEPS = {
    'East':  (1, 0),
    'West':  (-1, 0),
    'North': (0, -1),
    'South': (0, 1),
}
_CARDINAL_MAP_ANGLE_DEG = {'East': 0.0, 'North': 90.0, 'West': 180.0, 'South': 270.0}


def _scan_wall_dist_px(free_u8, x0, y0, dx, dy):
    """(x0,y0)에서 (dx,dy) 방향으로 한 칸씩 나아가며 벽(free_u8==0) 또는 이미지
    경계에 부딪힐 때까지의 픽셀 거리를 반환함. 실측 시 줄자로 벽까지 재는 것과
    같은 축 방향(동서/남북) 거리를 얻기 위함 - 전방위 최단거리(대각선 코너 포함)와
    달리 방을 가로지르는 방향으로만 잼."""
    h, w = free_u8.shape
    x, y = x0, y0
    steps = 0
    while True:
        x += dx
        y += dy
        steps += 1
        if x < 0 or x >= w or y < 0 or y >= h or free_u8[y, x] == 0:
            return steps


def _px_vec_to_map_angle_deg(dx_px, dy_px):
    """픽셀 벡터(dx=col 변화, dy=row 변화)를 map frame 각도(도, 0=+X East,
    90=+Y North, CCW+)로 변환함 - row 반전(위 설명) 반영."""
    return math.degrees(math.atan2(-dy_px, dx_px)) % 360.0


def _signed_delta_deg(target_deg, ref_deg):
    """target_deg - ref_deg를 (-180, 180]로 정규화 - 양수=반시계(CCW), 음수=시계(CW)."""
    d = (target_deg - ref_deg + 180.0) % 360.0 - 180.0
    return d


def _render_full_viz(nodes, path_segments, global_mask, map_resolution=None):
    """
    내부 헬퍼 함수: 노드와 경로 데이터를 바탕으로 시각화용 RGB 이미지를 생성함.

    map_resolution이 주어지면, 실제 로봇을 물리적으로 배치해야 하는 runway
    지점(repass_preview, label_at=0)에 정확한 heading(빨간 화살표)과 가장
    가까운 벽/장애물까지의 거리(cm)를 함께 표시함 - 실주행 시 로봇 배치
    오차(특히 heading)가 그대로 map->odom TF 오차로 굳어버리는 문제를 배치 단계에서 눈으로 확인할 수 있게 함.
    """
    h, w = global_mask.shape[:2]
    viz_mask = np.zeros((h, w, 3), dtype=np.uint8)
    num_nodes = len(nodes)
    
    # 1. 배경 노드 색칠 (HLS 컬러 공간 활용)
    total_nd_mask = np.zeros((h, w), dtype=np.uint8)
    for i, node in enumerate(nodes):
        # 노드별 고유 색상 생성 (시각적 구분을 위해 Hue값 분산)
        start_hue = 0.75 
        end_hue = 0.0     
        hue = start_hue - (i / (num_nodes - 1)) * (start_hue - end_hue) if num_nodes > 1 else start_hue
        
        bg_rgb = colorsys.hls_to_rgb(hue, 0.77, 1.0)
        bg_color = np.array([c * 255 for c in bg_rgb], dtype=np.uint8)[::-1] # RGB to BGR
        
        viz_d = node['driveable_mask']
        viz_nd = node.get('nondriveable_mask')
        
        viz_mask[viz_d > 0] = bg_color
        if viz_nd is not None:
            total_nd_mask = cv2.bitwise_or(total_nd_mask, viz_nd)

    # 비구동 영역(장애물) 표시 - Magenta
    viz_mask[total_nd_mask > 0] = [255, 0, 255]

    # 벽/장애물 여부 맵 - repass_preview의 runway 지점 표시용(동서/남북 방향
    # 벽까지 거리 스캔에 씀). 자유공간(전역 driveable, 장애물 아님)을 255/벽을 0으로.
    free_u8 = None
    if map_resolution:
        not_free = (global_mask == 0) | (total_nd_mask > 0)
        free_u8 = np.where(not_free, 0, 255).astype(np.uint8)

    # runway 지점의 heading 화살표/거리 라벨은 가장 마지막(최상단 레이어)에 그려야
    # 뒤이어 그려지는 exit repass 화살표 등에 가려지지 않음(실측: 같은 직선 스와스
    # 위에 start/exit repass가 겹치는 노드에서 발생) - 그리기 정보만 여기서 모아두고
    # 실제 cv2 호출은 함수 맨 끝에서 수행함.
    deferred_start_marker_draws = []

    # 2. 경로 렌더링 (Coverage: 하양/빨강점, Transit: 초록색)
    for idx, segment in enumerate(path_segments):
        seg_type = segment['type']
        path = segment['path']
        
        if not path or len(path) < 2:
            continue
            
        if seg_type == 'coverage':
            # 측정 경로는 굵은 흰색 선
            for k in range(len(path) - 1):
                cv2.line(viz_mask, path[k], path[k+1], (255, 255, 255), 2, cv2.LINE_AA)
            
        elif seg_type == 'transit':
            record_pcd = segment.get('record_pcd', False)
            if record_pcd:
                # 측정 중인 경유 구간 - coverage와 동일한 흰색으로 표시
                for k in range(len(path) - 1):
                    pt1 = tuple(map(int, path[k])); pt2 = tuple(map(int, path[k + 1]))
                    cv2.line(viz_mask, pt1, pt2, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.circle(viz_mask, tuple(map(int, path[0])), 3, (0, 165, 255), -1)   # 시작: 주황 점
                cv2.circle(viz_mask, tuple(map(int, path[-1])), 3, (0, 165, 255), -1)  # 끝: 주황 점
            else:
                for k in range(len(path) - 1):
                    pt1 = tuple(map(int, path[k])); pt2 = tuple(map(int, path[k + 1]))
                    cv2.line(viz_mask, pt1, pt2, (0, 255, 0), 1, cv2.LINE_AA)

        elif seg_type == 'repass_preview':
            # boundary_repass(mission_execution/utils/boundary_repass.py)가 실행 시
            # 만들 왕복(러닝스타트/되짚기) 예상 경로 - 시안색 화살표로 별도 표시.
            # path[0]=coverage 시작/끝점 또는 runway, path[1]=예상 retrace/p0 지점.
            # 라벨은 두 점 중 "이미 순번이 찍히지 않은, 실제로 새로 알아야 하는
            # 지점"에 붙여야 하므로 label_at으로 지정함(기본값 1 = 기본 동작 유지).
            pt0 = tuple(map(int, path[0])); pt1 = tuple(map(int, path[1]))
            cv2.arrowedLine(viz_mask, pt0, pt1, (255, 255, 0), 2, cv2.LINE_AA, tipLength=0.15)
            cv2.circle(viz_mask, pt1, 4, (255, 255, 0), -1)
            label_at = segment.get('label_at', 1)
            label_pt = pt0 if label_at == 0 else pt1
            if label_at == 0:
                # runway(로봇을 실제로 놔야 하는 지점) - 눈에 잘 띄게 큰 원으로 별도 표시
                cv2.circle(viz_mask, pt0, 7, (255, 255, 0), 2)

                # runway 지점에서 로봇은 p0를 바라보고 서 있어야 하므로
                # (boundary_repass.compute_runway_pose), pt0->pt1 방향이 곧 실제
                # heading임. map frame 각도(0=East,90=North,...)로 변환해둠 -
                # 아래 회전 안내 계산에 재사용.
                dx_px = pt1[0] - pt0[0]
                dy_px = pt1[1] - pt0[1]
                if map_resolution and (dx_px != 0 or dy_px != 0):
                    heading_deg = _px_vec_to_map_angle_deg(dx_px, dy_px)

                    # 동서/남북 각 방향으로 벽까지 거리(cm) - 대각선 코너가 아니라
                    # 방을 가로지르는 축 방향으로 잰 값이라 실측(줄자)과 바로 비교 가능.
                    dist_cm = {}
                    for name, (sx, sy) in _CARDINAL_PX_STEPS.items():
                        dist_cm[name] = _scan_wall_dist_px(free_u8, pt0[0], pt0[1], sx, sy) * map_resolution * 100.0

                    ew_side = 'East' if dist_cm['East'] <= dist_cm['West'] else 'West'
                    ns_side = 'North' if dist_cm['North'] <= dist_cm['South'] else 'South'

                    # 회전 기준(0°) = 넷 중 가장 가까운 벽을 정면으로 마주보는 방향.
                    # 대각선(코너) 방향이 아니라 반드시 축 방향 중 하나가 되도록,
                    # 전방위 최단거리 대신 위 4방향 스캔 중 최솟값을 씀 - "벽을 정면으로
                    # 본다"는 지시가 물리적으로 의미 있으려면 벽면에 수직인 방향이어야 함.
                    nearest_wall_side = min(dist_cm, key=dist_cm.get)
                    wall_ref_deg = _CARDINAL_MAP_ANGLE_DEG[nearest_wall_side]
                    rot_deg = _signed_delta_deg(heading_deg, wall_ref_deg)
                    rot_dir = 'CCW' if rot_deg >= 0 else 'CW'

                    print(f"    [start prepass] 배치 안내: 가장 가까운 벽({nearest_wall_side})을 "
                          f"정면으로 마주보고 선 상태에서 {rot_dir} 방향으로 {abs(rot_deg):.0f}° 회전 "
                          f"(map heading={heading_deg:.0f}°, 0°=East/+X, 90°=North/+Y 기준). "
                          f"동서 최단거리={dist_cm[ew_side]:.0f}cm({ew_side}), "
                          f"남북 최단거리={dist_cm[ns_side]:.0f}cm({ns_side}).")

                    # 실제 그리기(화살표+텍스트)는 맨 위 레이어로 미룸 - 뒤이어 그려질
                    # exit repass 화살표 등에 가려지지 않게 함.
                    deferred_start_marker_draws.append({
                        'pt0': pt0, 'dx_px': dx_px, 'dy_px': dy_px,
                        'rot_dir': rot_dir, 'rot_deg': abs(rot_deg),
                        'nearest_wall_side': nearest_wall_side,
                        'ew_side': ew_side, 'ew_cm': dist_cm[ew_side],
                        'ns_side': ns_side, 'ns_cm': dist_cm[ns_side],
                    })
            label = segment.get('label', 'repass')
            cv2.putText(viz_mask, label, (label_pt[0] + 6, label_pt[1] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3)
            cv2.putText(viz_mask, label, (label_pt[0] + 6, label_pt[1] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)

    # 3. 노드 ID 표시
    for i, node in enumerate(nodes):
        mask = node['driveable_mask']
        if cv2.countNonZero(mask) > 0:
            M = cv2.moments(mask)
            if M["m00"] != 0:
                cX, cY = int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])
                text = f"ID:{node['id']}"
                cv2.putText(viz_mask, text, (cX - 20, cY + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3)
                cv2.putText(viz_mask, text, (cX - 20, cY + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)

    # 4. 미션 수행 순서(Sequence) 번호 매기기
    target_points = []
    for seg in path_segments:
        if seg['type'] == 'coverage' and len(seg['path']) > 0:
            target_points.append(seg['path'][0])
            if seg['path'][0] != seg['path'][-1]:
                target_points.append(seg['path'][-1])
    
    for idx, pt in enumerate(target_points):
        # 처음과 마지막 지점은 노란색, 중간은 빨간색
        is_edge = (idx == 0 or idx == len(target_points) - 1)
        color = (0, 255, 255) if is_edge else (0, 0, 255)
        
        cv2.circle(viz_mask, pt, 2 if is_edge else 1, color, -1)
        cv2.putText(viz_mask, str(idx + 1), (pt[0] - 8, pt[1] + 5), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 2)
        cv2.putText(viz_mask, str(idx + 1), (pt[0] - 8, pt[1] + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

    # 5. runway 지점 heading 화살표/배치 안내 - 항상 최상단에 그림(위 설명 참고).
    for mark in deferred_start_marker_draws:
        pt0 = mark['pt0']
        vec_len = math.hypot(mark['dx_px'], mark['dy_px'])
        if vec_len > 1e-6:
            unit = (mark['dx_px'] / vec_len, mark['dy_px'] / vec_len)
            tip = (int(pt0[0] + unit[0] * 25), int(pt0[1] + unit[1] * 25))
            cv2.arrowedLine(viz_mask, pt0, tip, (0, 0, 255), 2, cv2.LINE_AA, tipLength=0.4)

        rot_label = f"{mark['rot_dir']} {mark['rot_deg']:.0f} deg from {mark['nearest_wall_side']} wall"
        dist_label = f"EW {mark['ew_cm']:.0f}cm({mark['ew_side']}) NS {mark['ns_cm']:.0f}cm({mark['ns_side']})"
        for i, text in enumerate((rot_label, dist_label)):
            text_pt = (pt0[0] + 6, pt0[1] + 14 + i * 13)
            cv2.putText(viz_mask, text, text_pt, cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 0), 3)
            cv2.putText(viz_mask, text, text_pt, cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)

    return viz_mask

def save_debug_image(nodes, path_segments, global_mask, output_dir="./debug_image",
                      filename="full_mission_connected.png", map_resolution=None):
    """
    최종 미션 상태를 이미지 파일로 저장함.
    """
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    viz_image = _render_full_viz(nodes, path_segments, global_mask, map_resolution)
    save_path = os.path.join(output_dir, filename)
    cv2.imwrite(save_path, viz_image)
    print(f"[*] Debug image saved to: {save_path}")
    return save_path

def plot_mission_state(nodes, path_segments, global_mask, wait_key=0, map_resolution=None):
    """
    현재 미션 상태를 화면에 표시함.
    """
    viz_image = _render_full_viz(nodes, path_segments, global_mask, map_resolution)
    cv2.imshow("Mission State Visualization", viz_image)
    cv2.waitKey(wait_key)
    return viz_image

def draw_waypoint_on_image(viz_image, pixel_points):
    """
    이미지 위에 샘플링된 경로 포인트들을 그림.
    """
    for pt in pixel_points:
        # (x, y) 좌표에 점을 찍음 (색상: 검정색)
        cv2.circle(viz_image, pt, 2, (0, 0, 0), -1)
    return viz_image