# mission_generation/run_generation_pipeline.py

import os
import shutil
import yaml
import json
import traceback

from ament_index_python.packages import get_package_share_directory

# 코어 모듈 - 맵 분할과 경로 생성
from mission_generation.environment_modeling.environment_modeler import EnvironmentModeler
from mission_generation.mission_planning.mission_planner import MissionPlanner


def _snapshot_planning_outputs(label, workspace_root, grid_dir, map_yaml_path, topology_dir, topology_file, metric_dir, final_path_file, env_vis_dir=None, planner_vis_dir=None):
    """알고리즘 비교 실험용 - 맵/토폴로지/최종 경로/시각화 디버그
    이미지를 <workspace_root>/eval_runs/<label>/ 아래에 복사해둠(원본은
    그대로 flat 경로에 남겨서 기존 "재사용?" 프롬프트가 계속 정상 동작하게
    함).

    map_from_dae.yaml/.pgm, final_topological_map.npz, final_path.json,
    final_path_meta.json은 항상 같은 고정 경로에 저장되는 파일이라,
    이 함수 없이 다음 알고리즘을 이어서 생성하면 이전 결과가 흔적도 없이
    덮어써져 실험 간 데이터 정합성이 깨짐 - 그래서 라벨을 준
    경우에만 복사본을 별도로 남김. 시각화 디렉토리(env_vis_dir/
    planner_vis_dir)도 같은 이유로 매 생성마다 덮어써져서 함께 스냅샷함
    - 이 둘은 단일 파일이 아니라 디렉토리 통째로 복사함."""
    eval_root = os.path.join(workspace_root, 'eval_runs', label)

    # 1. 맵(yaml + 그 안에서 참조하는 이미지 파일)
    dst_grid_dir = os.path.join(eval_root, os.path.relpath(grid_dir, workspace_root))
    os.makedirs(dst_grid_dir, exist_ok=True)
    if os.path.exists(map_yaml_path):
        shutil.copy2(map_yaml_path, dst_grid_dir)
        with open(map_yaml_path, 'r') as f:
            map_meta = yaml.safe_load(f)
        image_name = map_meta.get('image')
        if image_name:
            image_path = image_name if os.path.isabs(image_name) else os.path.join(os.path.dirname(map_yaml_path), image_name)
            if os.path.exists(image_path):
                shutil.copy2(image_path, dst_grid_dir)
    else:
        print(f"[!] Warning: snapshot 대상 맵 파일이 없어 건너뜀: {map_yaml_path}")

    # 2. 공간 분할 토폴로지
    dst_topology_dir = os.path.join(eval_root, os.path.relpath(topology_dir, workspace_root))
    os.makedirs(dst_topology_dir, exist_ok=True)
    if os.path.exists(topology_file):
        shutil.copy2(topology_file, dst_topology_dir)
    else:
        print(f"[!] Warning: snapshot 대상 토폴로지 파일이 없어 건너뜀: {topology_file}")

    # 3. 최종 경로 + planning 시점 파라미터 사이드카
    dst_metric_dir = os.path.join(eval_root, os.path.relpath(metric_dir, workspace_root))
    os.makedirs(dst_metric_dir, exist_ok=True)
    if os.path.exists(final_path_file):
        shutil.copy2(final_path_file, dst_metric_dir)
    else:
        print(f"[!] Warning: snapshot 대상 final_path.json이 없어 건너뜀: {final_path_file}")
    meta_file = os.path.join(metric_dir, "final_path_meta.json")
    if os.path.exists(meta_file):
        shutil.copy2(meta_file, dst_metric_dir)

    # 4. 시각화 디버그 이미지(env modeling/mission planning) - 디렉토리 통째로 복사
    for vis_dir in (env_vis_dir, planner_vis_dir):
        if not vis_dir:
            continue
        if os.path.isdir(vis_dir):
            dst_vis_dir = os.path.join(eval_root, os.path.relpath(vis_dir, workspace_root))
            shutil.copytree(vis_dir, dst_vis_dir, dirs_exist_ok=True)
        else:
            print(f"[!] Warning: snapshot 대상 시각화 폴더가 없어 건너뜀: {vis_dir}")

    print(f"[+] Snapshot saved to: {eval_root}")


def run_generation_pipeline(snapshot_label=None):
    print("\n=======================================================")
    print("[*] Mission Generator (Workstation)")
    print("    : Environment Modeling and Mission Planning")
    print("=======================================================\n")

    try:
        package_share_dir = get_package_share_directory('dae_coverage_floor_flatness')
        config_path = os.path.join(package_share_dir, 'config', 'params.yaml')
    except Exception:
        # 현재 파일(__file__)의 부모 디렉터리(..)로 이동 후 config/params.yaml 추적
        config_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config", "params.yaml"))
    
    # 1. Load Global Config
    print(f"[*] Resolving parameters from: {config_path}")
    if not os.path.exists(config_path):
        print(f"[!] Critical Error: Global Configuration file not found at {config_path}")
        return

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    # params.yaml 내부 'global'
    global_cfg = config.get('global', {})
    workspace_root = os.path.expanduser(global_cfg.get('workspace_root', '~/dae_floor_maps'))

    # 'environment_modeling'
    env_cfg = config.get('environment_modeling', {})
    topology_dir = os.path.join(workspace_root, env_cfg.get('output_topology_dir', 'maps/topology'))
    grid_dir = os.path.join(workspace_root, env_cfg.get('output_grid_dir', 'maps/grid'))

    map_file = os.path.normpath(os.path.join(topology_dir, "final_topological_map.npz"))
    yaml_path = os.path.normpath(os.path.join(grid_dir, "map_from_dae.yaml"))

    # 'mission_planner'
    mission_cfg = config.get('mission_planner', {})
    metric_dir = os.path.join(workspace_root, mission_cfg.pop('output_metric_dir', 'analytics/metrics'))
    cache_file = os.path.normpath(os.path.join(metric_dir, "final_path.json"))

    # 'mission_execution' - boundary_repass_distance_m/enable_boundary_repass는
    # 실행 단계 섹션에 있지만, planning 단계에서도 동일 값이 필요해 명시적으로
    # 꺼내옴(mission_planner 섹션을 그대로 넘기는 방식으로는 다른 섹션의
    # 값이 전달되지 않음). 이 값이
    # 실행 시점(mission_executor.py)의 값과 다르면 planning된 transit 시작점과
    # 실제 로봇이 repass 후 서 있을 위치가 어긋남 - 다만
    # mission_planner.py가 저장하는 final_path_meta.json을 mission_executor.py가
    # 시작 시 자동 대조하므로, 어긋나면 미션이 스스로 CRITICAL ERROR로
    # 중단됨(사람이 기억할 필요 없음).
    mission_exec_cfg = config.get('mission_execution', {})
    boundary_repass_distance_m = mission_exec_cfg.get('boundary_repass_distance_m', 1.5)
    enable_boundary_repass = mission_exec_cfg.get('enable_boundary_repass', True)

    # 시각화 디렉토리 경로 - regenerate 여부와 무관하게 스냅샷 시점에 항상
    # 필요하므로 여기서 미리 계산해둠(mission_cfg.pop은 아래 regenerate
    # 분기에서 MissionPlanner 생성자 인자로도 재사용하므로 그대로 유지).
    env_vis_path = os.path.join(workspace_root, env_cfg.get('visualization_dir', 'visualization/mission_generation/environment_modeling'))
    planner_vis_rel = mission_cfg.pop('visualization_dir', 'visualization/mission_generation/mission_planning')
    planner_vis_path = os.path.join(workspace_root, planner_vis_rel)

    # 2. Map Processing
    need_process = False

    if not os.path.exists(map_file):
        print(f"[*] Preprocessed topological asset missing at: {map_file}")
        need_process = True
    else:
        ans = input(f"[*] Topological map asset exists. Re-process 3D Map to 2D? (y/n): ")
        need_process = ans.lower() in ['y', 'yes']

    if need_process:
        print("[*] Instantiating EnvironmentModeling...")
        try:
            env_modeler = EnvironmentModeler(config)
            if hasattr(env_modeler, 'build_environment_model'):
                success = env_modeler.build_environment_model()
            else:
                success = env_modeler.run_all()
                
            if not success:
                print("[!] Map preprocessing returned failure. Aborting mission planning.")
                return
        except Exception as e:
            print(f"[!] Map Pre-processing System Crash: {e}")
            traceback.print_exc()
            return

    # 3. Mission Planning (global waypoint) 
    raw_path_file = os.path.join(metric_dir, "raw_path.json") # 샘플링 이전의 웨이포인트들

    regenerate = True
    if os.path.exists(cache_file) and os.path.exists(raw_path_file) and not need_process:
        ans = input(f"[*] Target waypoint registry files exist. Re-generate Path? (y/n): ")
        regenerate = ans.lower() in ['y', 'yes']

    if regenerate:
        print("[*] Launching MissionPlanner Engine...")
        try:
            robot_width = env_cfg.get('robot_width', 0.28)
            path_safety_margin = mission_cfg.pop('path_safety_margin', 0.20)
            lidar_mount_height = mission_cfg.pop('lidar_mount_height', 0.338)
            lidar_vertical_fov_deg = mission_cfg.pop('lidar_vertical_fov_deg', 15.0)

            
            planner = MissionPlanner(
                topomap_path=map_file,
                visualization_dir=planner_vis_path,
                robot_width=robot_width,
                path_safety_margin=path_safety_margin,
                lidar_mount_height=lidar_mount_height,
                lidar_vertical_fov_deg=lidar_vertical_fov_deg,
                boundary_repass_distance_m=boundary_repass_distance_m,
                enable_boundary_repass=enable_boundary_repass,
                **mission_cfg
            )
            
            final_path = planner.plan(
                save_debug=True,
                show_plot=False,
                output_dir=metric_dir,
            )
            
            if final_path:
                print(f"[+] Success! New full coverage path (sampled) saved to: {cache_file}")
                print(f"    Total Generated Waypoints: {len(final_path)}")
                print(f"\n[!] Mission execution assets generated. Please sync the workspace root '{workspace_root}' to Jetson Orin Nano.")
            else:
                print("[!] Mission Planner returned empty route. Path generation aborted.")
                return
        except Exception as e:
            print(f"[!] Mission Planning System Crash: {e}")
            traceback.print_exc()
            return
    else:
        print("[*] Safe Mode: Reusing existing final_path.json registry. Skip optimization.")

    if snapshot_label:
        _snapshot_planning_outputs(
            snapshot_label, workspace_root, grid_dir, yaml_path,
            topology_dir, map_file, metric_dir, cache_file,
            env_vis_dir=env_vis_path, planner_vis_dir=planner_vis_path,
        )

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Mission Generator (Workstation) - Environment Modeling and Mission Planning")
    parser.add_argument(
        "--snapshot-label", type=str, default=None,
        help="알고리즘 비교 실험용 - 주어지면 이번에 쓰인 맵/토폴로지/final_path를 "
             "<workspace_root>/eval_runs/<라벨>/에 복사해둠. 안 주면(기본값) 기본 동작과 동일함."
    )
    args = parser.parse_args()
    run_generation_pipeline(snapshot_label=args.snapshot_label)
