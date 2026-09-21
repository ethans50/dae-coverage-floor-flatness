#!/usr/bin/env python3
"""
surface_profiling/analyze_coverage_comparison.py

커버리지 알고리즘 비교(baseline 및 leave-one-out ablation 조합)
실험용 분석 도구임. 재주행 없이 이미 저장된
combined_*.pcd(+선택적으로 combined_raw_*.pcd, robot_path_*.csv,
drive_debug_*.csv)만 입력받아 지표를 계산함 - reprocess_pcd.py와 동일하게
params.yaml을 그대로 재사용하고 실측 데이터만 반복 재처리하는 컨벤션을 따름.

계산하는 지표:
    1. 완전성(completeness): 유효 바닥 영역(2D 맵의 free-space) 대비
       데이터가 있는 grid_size 셀 비율. combined_*.pcd(다운샘플본)로 계산함.
    2. gap: 완전성의 보완 지표 - 미측정 셀 개수/면적.
    3. 셀당 z-표준편차(측정 노이즈)/셀당 리턴 수: combined_raw_*.pcd(다운샘플
       이전)가 있을 때만 계산함 - 다운샘플된 PCD는 셀당 최대 1점이라 이
       지표를 낼 수 없음.
    4. 거리/시간: robot_path_*.csv(timestamp,x,y)에서 계산함.
    5. 회전량(근사): drive_debug_*.csv의 yaw_deg 컬럼으로 근사함 -
       robot_path_*.csv에는 orientation 컬럼이 없어 정확한 회전량을 낼 수
       없고, drive_debug_interval_sec(기본 3초) 간격 샘플이라 그 사이의
       빠른 회전은 과소평가될 수 있는 근사치임.
    6. 순주행시간(active_time_sec): stall_report_*.csv가 있을 때만 계산함
       - total_time_sec에서 stall(nav2 recovery 대기 등) 지속시간을 뺀
       값. 같은 조합을 반복 실행해도 raw total_time_sec은 stall 유무/길이
       때문에 크게 흔들려 조합 간 비교가 어렵기 때문임.

사용법:
    python3 analyze_coverage_comparison.py combined_<timestamp>.pcd
    python3 analyze_coverage_comparison.py combined_<timestamp>.pcd \\
        --raw-pcd combined_raw_<timestamp>.pcd \\
        --robot-path robot_path_1785493611.csv \\
        --drive-debug drive_debug_<timestamp>.csv \\
        --stall-report stall_report_1785493611.csv
"""

import os
import sys
import json
import argparse


def _add_utils_to_path():
    """reprocess_pcd.py와 동일한 방식으로 utils 모듈을 임포트 가능하게 함 -
    소스 트리/install 트리 어느 쪽에서 실행해도 동작하도록 함."""
    this_dir = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, this_dir)


_add_utils_to_path()

try:
    from utils.config_paths import (
        load_full_config,
        resolve_pointcloud_dir,
        resolve_map_yaml_path,
    )
    from utils.coverage_metrics import (
        compute_coverage_metrics,
        compute_path_stats,
        compute_stall_stats,
        compute_rotation_from_drive_debug,
    )
except ImportError:
    from surface_profiling.utils.config_paths import (
        load_full_config,
        resolve_pointcloud_dir,
        resolve_map_yaml_path,
    )
    from surface_profiling.utils.coverage_metrics import (
        compute_coverage_metrics,
        compute_path_stats,
        compute_stall_stats,
        compute_rotation_from_drive_debug,
    )


# ----------------------------------------------------------------------
# CLI
# ----------------------------------------------------------------------

def _resolve_input_path(filename, base_dir):
    if filename is None:
        return None
    if os.path.isabs(filename) or os.path.exists(filename):
        return filename
    return os.path.join(base_dir, filename)


def main():
    parser = argparse.ArgumentParser(
        description="커버리지 알고리즘 3-way 비교용 지표 계산 도구 (재주행 불필요)"
    )
    parser.add_argument("pcd_filename", help="pointcloud_dir 안의 다운샘플 combined_*.pcd (완전성/gap 계산용)")
    parser.add_argument("--eval-label", type=str, default=None, help="실험 라벨 - 주어지면 pcd/csv/map을 workspace_root 대신 eval_runs/<라벨>/ 아래 자기완결 폴더(run_generation_pipeline.py --snapshot-label로 만든)에서 찾음")
    parser.add_argument("--raw-pcd", type=str, default=None, help="pointcloud_dir 안의 combined_raw_*.pcd (셀당 z-표준편차/리턴 수 계산용, save_raw_pcd=true로 수집한 경우만 존재)")
    parser.add_argument("--robot-path", type=str, default=None, help="mission_execution.output_path_dir 안의 robot_path_*.csv (거리/시간 계산용)")
    parser.add_argument("--drive-debug", type=str, default=None, help="mission_execution.drive_debug_log_dir 안의 drive_debug_*.csv (회전량 근사용)")
    parser.add_argument("--stall-report", type=str, default=None, help="mission_execution.stall_log_dir 안의 stall_report_*.csv (stall 제외 순주행시간 active_time_sec 계산용)")
    parser.add_argument("--z-min", type=float, default=None, help="params.yaml의 z_min을 이번 실행에서만 덮어씀 (m)")
    parser.add_argument("--z-max", type=float, default=None, help="params.yaml의 z_max를 이번 실행에서만 덮어씀 (m)")
    parser.add_argument("--grid-size", type=float, default=None, help="완전성/gap 계산 격자 크기를 이번 실행에서만 덮어씀 (m, 기본은 params.yaml의 heatmap grid_size)")
    parser.add_argument("--map-yaml-dir", type=str, default=None, help="params.yaml의 map_yaml_dir를 이번 실행에서만 덮어씀")
    parser.add_argument("--output", type=str, default=None, help="결과 JSON을 저장할 경로 (기본: pointcloud_dir/analysis_<stem>.json)")
    args = parser.parse_args()

    workspace_root, config = load_full_config()
    profiling_cfg = config.get('surface_profiling', {})
    mission_exec_cfg = config.get('mission_execution', {})

    # --eval-label이 주어지면 run_generation_pipeline.py --snapshot-label /
    # mission_execution.launch.py·surface_profiling.launch.py eval_run_label:=
    # 로 만들어둔 자기완결 폴더(eval_runs/<라벨>/)에서 전부 찾음 - 나중에
    # workspace_root의 flat 경로가 다른 실행으로 덮어써져도 이 폴더 하나만
    # 있으면 그때 그 알고리즘의 결과를 그대로 재현 분석할 수 있음.
    output_root = os.path.join(workspace_root, 'eval_runs', args.eval_label) if args.eval_label else workspace_root

    pointcloud_dir = resolve_pointcloud_dir(output_root, profiling_cfg)
    path_dir = os.path.join(output_root, mission_exec_cfg.get('output_path_dir', 'analytics/paths'))
    drive_debug_dir = os.path.join(output_root, mission_exec_cfg.get('drive_debug_log_dir', 'analytics/logs'))
    stall_log_dir = os.path.join(output_root, mission_exec_cfg.get('stall_log_dir', 'analytics/logs'))

    pcd_path = _resolve_input_path(args.pcd_filename, pointcloud_dir)
    if not os.path.exists(pcd_path):
        print(f"[!] PCD file not found: {pcd_path}")
        sys.exit(1)

    raw_pcd_path = _resolve_input_path(args.raw_pcd, pointcloud_dir)
    if raw_pcd_path is not None and not os.path.exists(raw_pcd_path):
        print(f"[!] Warning: --raw-pcd given but not found: {raw_pcd_path} - skipping raw-based metrics.")
        raw_pcd_path = None

    robot_path_csv = _resolve_input_path(args.robot_path, path_dir)
    if robot_path_csv is not None and not os.path.exists(robot_path_csv):
        print(f"[!] Warning: --robot-path given but not found: {robot_path_csv} - skipping distance/time metrics.")
        robot_path_csv = None

    drive_debug_csv = _resolve_input_path(args.drive_debug, drive_debug_dir)
    if drive_debug_csv is not None and not os.path.exists(drive_debug_csv):
        print(f"[!] Warning: --drive-debug given but not found: {drive_debug_csv} - skipping rotation estimate.")
        drive_debug_csv = None

    stall_report_csv = _resolve_input_path(args.stall_report, stall_log_dir)
    if stall_report_csv is not None and not os.path.exists(stall_report_csv):
        print(f"[!] Warning: --stall-report given but not found: {stall_report_csv} - skipping active_time_sec.")
        stall_report_csv = None

    z_min = args.z_min if args.z_min is not None else profiling_cfg.get('z_min', -0.005)
    z_max = args.z_max if args.z_max is not None else profiling_cfg.get('z_max', 0.035)
    grid_size = args.grid_size if args.grid_size is not None else profiling_cfg.get('grid_size', 0.02)

    if args.map_yaml_dir is not None:
        # 명시적 override는 항상 실제 workspace_root 기준(라벨과 무관하게
        # 사용자가 지정한 그 위치를 그대로 존중함)
        override_cfg = dict(profiling_cfg)
        override_cfg['map_yaml_dir'] = args.map_yaml_dir
        map_yaml_path = resolve_map_yaml_path(workspace_root, override_cfg)
    else:
        map_yaml_path = resolve_map_yaml_path(output_root, profiling_cfg)

    if map_yaml_path is None or not os.path.exists(map_yaml_path):
        print(f"[!] CRITICAL ERROR: map_yaml_path가 필요하지만 찾을 수 없음: {map_yaml_path} "
              "(완전성/gap 계산에는 free-space 기준 맵이 반드시 필요함).")
        sys.exit(1)

    # 완전성 분모로 쓸 커버리지 노드 마스크. eval_run 스냅샷마다 같은 위치에
    # 저장되므로 map_yaml과 같은 output_root 기준으로 찾음.
    topology_npz_path = os.path.join(output_root, 'maps', 'topology', 'final_topological_map.npz')
    if not os.path.exists(topology_npz_path):
        print(f"[!] Warning: topology npz not found at {topology_npz_path} - "
              "완전성 분모를 맵 전체 free-space로 폴백함(과소평가됨).")
        topology_npz_path = None

    print("[*] Config values in use:")
    print(f"    z_min          = {z_min}")
    print(f"    z_max          = {z_max}")
    print(f"    grid_size      = {grid_size}")
    print(f"    map_yaml_path  = {map_yaml_path}")
    print(f"    topology npz   = {topology_npz_path}")
    print(f"    input pcd      = {pcd_path}")
    print(f"    raw pcd        = {raw_pcd_path}")
    print(f"    robot_path csv = {robot_path_csv}")
    print(f"    drive_debug csv= {drive_debug_csv}")
    print(f"    stall_report csv= {stall_report_csv}")

    result = compute_coverage_metrics(pcd_path, raw_pcd_path, map_yaml_path, z_min, z_max, grid_size,
                                      topology_npz_path=topology_npz_path)

    if robot_path_csv is not None:
        result['path_stats'] = compute_path_stats(robot_path_csv)
    else:
        result['path_stats'] = None

    if drive_debug_csv is not None:
        result['rotation_deg_approx'] = compute_rotation_from_drive_debug(drive_debug_csv)
    else:
        result['rotation_deg_approx'] = None

    if stall_report_csv is not None:
        stall_stats = compute_stall_stats(stall_report_csv)
        result['stall_stats'] = stall_stats
        # total_time_sec에서 stall 지속시간을 뺀 "순수 주행 시간"
        # (compute_stall_stats 참고).
        if result.get('path_stats') is not None:
            result['path_stats']['active_time_sec'] = (
                result['path_stats']['total_time_sec'] - stall_stats['total_stalled_sec']
            )
    else:
        result['stall_stats'] = None

    print("\n[+] Results:")
    print(json.dumps(result, indent=2, ensure_ascii=False))

    if args.output is not None:
        output_path = args.output
    else:
        stem = os.path.splitext(os.path.basename(pcd_path))[0]
        output_path = os.path.join(pointcloud_dir, f"analysis_{stem}.json")
    with open(output_path, 'w') as f:
        json.dump(result, f, indent=2, ensure_ascii=False)
    print(f"\n[+] Saved analysis result: {output_path}")


if __name__ == "__main__":
    main()
