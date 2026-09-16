# mission_generation/mission_planning/path_exporter.py
"""
계획이 끝난 px 단위 경로를 미터 좌표로 옮겨 디스크에 내보내는 단계.

`MissionPlanner.plan()`의 마지막 단계를 담당함 - translator로 픽셀→미터 변환,
sampler로 일정 간격 재샘플링, `raw_path.json`/`final_path.json`/
`final_path_meta.json` 저장, 그리고 디버그 이미지 위 웨이포인트 오버레이.

`final_path_meta.json`은 계획 시점에 실제로 쓴 파라미터의 사이드카 스냅샷임 -
실행 시점(`mission_executor.py`)의 `params.yaml` 값과 어긋나면 계획된 transit
시작점과 실제 로봇 위치가 조용히 달라지므로, 미션 시작 시 자동 대조해
불일치하면 즉시 중단시킴(`_verify_plan_meta`, HISTORY.md §2 참고).

planner 객체를 그대로 받아 읽기만 함(역방향 호출 없음) - 넘겨야 할 스칼라가
10개가 넘어 인자로 풀어쓰는 것보다 결합도가 낮음.
"""

import os
import json
import math

import cv2

from mission_planning import translator
from mission_planning.utils import visualizer, sampler


def export(planner, output_dir, save_debug=True):
    """planner의 path_segments를 미터 좌표로 변환·저장하고 샘플링된 경로를 반환함."""
    # 4. Translator를 통한 좌표 변환 및 메시지 포맷팅 (Pixel -> Meter)
    # Y축 대칭 반전 역산을 위해 전역 마스크 이미지의 세로 픽셀 크기(Height)를 추출함.
    map_height = planner.global_mask.shape[0]

    print("[*] Translating path segments to Metric coordinates...")

    raw_nav2_path = translator.convert_segments_to_nav2(
        path_segments=planner.path_segments,
        origin=planner.origin,
        resolution=planner.map_resolution,
        map_height=map_height
    )

    # sampling_step만큼의 거리마다 샘플링
    sampled_nav2_path = sampler.interpolate_with_semantics(
        raw_nav2_path
    )
    
    raw_flat_path = []
    for seg in raw_nav2_path:
        for p in seg['poses']:
            p_copy = json.loads(json.dumps(p))
            p_copy['header'] = {
                'frame_id': 'map',
                'task_type': seg['type'],
                'record_pcd': seg.get('record_pcd', seg['type'] == 'coverage'),
            }
            x, y = p_copy['pose']['position']['x'], p_copy['pose']['position']['y']
            
            # 거리 기반 비교
            if not raw_flat_path or math.hypot(raw_flat_path[-1]['pose']['position']['x'] - x,
                                            raw_flat_path[-1]['pose']['position']['y'] - y) > 0.001:
                raw_flat_path.append(p_copy)

    os.makedirs(output_dir, exist_ok=True)
    raw_output_file = os.path.join(output_dir, "raw_path.json")
    sampled_output_file = os.path.join(output_dir, "final_path.json")

    with open(raw_output_file, 'w') as f:
        json.dump(raw_flat_path, f, indent=4)
        
    with open(sampled_output_file, 'w') as f:
        json.dump(sampled_nav2_path, f, indent=4)

    # final_path.json 자체가 이 값들(특히 boundary_repass_distance_m/
    # enable_boundary_repass, _compute_repass_adjusted_exit 참고)에
    # 기하학적으로 의존하므로, 계획 시점과 실행 시점(mission_executor.py가
    # params.yaml에서 직접 읽음)의 값이 어긋나면 계획된 transit 시작점과
    # 실제 repass 후 로봇 위치가 조용히 달라짐 - 계획 시점에 실제로 쓴
    # 값을 사이드카 파일로 남겨서 mission_executor.py가 시작 시 자기
    # params.yaml 값과 자동 대조하게 함(다르면 다른 CRITICAL ERROR들과
    # 동일하게 즉시 중단 - _load_final_path 참고, 도입 경위는 HISTORY.md
    # §2 참고).
    meta_output_file = os.path.join(output_dir, "final_path_meta.json")
    plan_meta = {
        'robot_width': planner.robot_width,
        'path_safety_margin': planner.path_safety_margin,
        'boundary_repass_distance_m': planner.boundary_repass_distance_m,
        'enable_boundary_repass': planner.enable_boundary_repass,
        'map_resolution': planner.map_resolution,
        'blind_radius_m': planner.blind_radius_m,
        # 아래 5개는 실행 시 참조/대조되지 않음(순수 계획 단계 좌표
        # 생성에만 관여) - ablation 실험 시 이 final_path.json이 어떤
        # 토글 조합으로 생성됐는지 추적하기 위한 기록용 메타데이터.
        'enable_pendant_reorder': planner.enable_pendant_reorder,
        'enable_entry_hint_ordering': planner.enable_entry_hint_ordering,
        'enable_path_simplification': planner.enable_path_simplification,
        'coverage_mode': planner.coverage_mode,
        'enable_optimal_swath_angle': planner.enable_optimal_swath_angle,
    }
    with open(meta_output_file, 'w') as f:
        json.dump(plan_meta, f, indent=4)

    print("[*] Mission Planner Successfully Completed.")
    print(f"    -> Raw Keypoints Path saved to: {raw_output_file} ({len(raw_flat_path)} pts)")
    print(f"    -> Sampled Path saved to: {sampled_output_file} ({len(sampled_nav2_path)} pts)")
    print(f"    -> Plan-time parameter snapshot saved to: {meta_output_file}")

    if save_debug:
        print("[*] Drawing path points on debug images...")
        
        # 1. 픽셀 좌표 변환 함수
        def get_pixel_points(pose_list_or_segments, is_raw=False):
            pts = []
            # 원본(raw)인 경우 중첩 리스트 구조, sampled된 경로인 경우 포인트들의 단일 리스트임.
            poses = []
            if is_raw:
                for seg in pose_list_or_segments:
                    poses.extend(seg['poses'])
            else:
                poses = pose_list_or_segments

            for p in poses:
                mx = p['pose']['position']['x']
                my = p['pose']['position']['y']
                px = int((mx - planner.origin[0]) / planner.map_resolution)
                py = int(map_height - (my - planner.origin[1]) / planner.map_resolution)
                if 0 <= px < planner.global_mask.shape[1] and 0 <= py < planner.global_mask.shape[0]:
                    pts.append((px, py))
            return pts

        # 좌표 추출
        raw_pixel_points = get_pixel_points(raw_nav2_path, is_raw=True)
        sampled_pixel_points = get_pixel_points(sampled_nav2_path, is_raw=False)

        # 오버레이할 베이스 이미지 경로
        base_img_path = os.path.join(planner.visualization_dir, "full_mission_path.png")

        # 2. 이미지 로드 및 오버레이
        def create_overlay_image(output_path, points):
            if os.path.exists(base_img_path):
                img = cv2.imread(base_img_path)
                if img is not None:
                    img = visualizer.draw_waypoint_on_image(img, points)
                    cv2.imwrite(output_path, img)
                    print(f"[*] Overlay saved to: {output_path}")
                else:
                    print(f"[!] Failed to load base image: {base_img_path}")
            else:
                print(f"[!] Base image not found: {base_img_path}")

        create_overlay_image(os.path.join(planner.visualization_dir, "raw_waypoint.png"), raw_pixel_points)
        create_overlay_image(os.path.join(planner.visualization_dir, "sampled_waypoint.png"), sampled_pixel_points)
        
    # 샘플링된 웨이포인트 반환. 필요하다면 원본(raw) 포인트를 반환해도 됨.
    return sampled_nav2_path
