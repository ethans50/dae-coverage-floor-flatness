# 코드 구조

[English](../en/architecture.md) · [한국어](architecture.md) · [← README](../../README.kr.md)

추적 중인 파일이 파이프라인의 어느 단계에 속하는지, 각 단계가 무엇을 읽고 쓰는지를 정리한 문서임. 알고리즘을 그렇게 고른 이유는 README의 [핵심 설계 선택](../../README.kr.md#핵심-설계-선택)에, 파라미터는 [설정 가이드](configuration.md)에 있음.

---

## 1. 파이프라인과 실행 위치

```text
 3D 설계 파일 (.dae)
        │
        ▼
 [1] environment_modeling ──► 2D 맵, 노드 분할                    (워크스테이션, 오프라인)
        │
        ▼
 [2] mission_planning ──────► final_path.json (+ 메타데이터 사이드카)  (워크스테이션, 오프라인)
        │
        ▼
 [3] mission_execution ─────► Nav2 / AMCL로 path를 주행            (로봇, 온라인)
        │  capture 시작 / 종료 서비스
        ▼
 [4] surface_profiling ─────► 점군, 프레임 기록, 히트맵            (라이다 노트북, 온라인)
```

| 단계 | 디렉터리 | 진입점 | 실행 위치 |
|---|---|---|---|
| 1 + 2 | `mission_generation/` | `python3 mission_generation/run_generation_pipeline.py` | 워크스테이션 |
| 3 | `mission_execution/` | `mission_executor` (`mission_execution.launch.py`) | 로봇(Jetson) 또는 시뮬레이션 |
| 4 | `surface_profiling/` | `surface_profiler` (`surface_profiling.launch.py`) | 라이다 노트북 또는 시뮬레이션 |
| 지원 | `launch/`, `config/`, `models/`, `urdf/`, `worlds/`, `scripts/` | 해당 없음 | 전부 |

`setup.py`에 등록된 console script는 `mission_executor`, `imu_tilt_broadcaster`, `surface_profiler` 세 개임. 3단계와 4단계는 ROS로만 소통함. 3단계가 `/surface_profiling/{start_waypoint_capture,stop_waypoint_capture,stop_collection_success}` 서비스로 capture 구간을 열고 닫으면, 4단계는 TF 트리와 `/velodyne_points`를 읽음.

## 2. 단계별 입출력 파일

데이터는 패키지 밖, `config/params.yaml`의 `workspace_root`(기본 `~/dae_floor_maps`) 아래에 쌓임. 아래 디렉터리 이름은 같은 파일에 있는 키의 기본값임.

| 단계 | 읽는 것 | 쓰는 것 |
|---|---|---|
| Environment modeling | `assets/<dae_file>` | `maps/grid/map_from_dae.{pgm,yaml}`, `maps/grid/map_preprocessed_data.npz`, `maps/topology/final_topological_map.npz`, `visualization/` 아래 디버그 이미지 |
| Mission planning | topology 맵, 2D 맵, `params.yaml` | `analytics/metrics/final_path.json`, `analytics/metrics/final_path_meta.json`, 디버그 이미지, 그리고 `analytics/metrics/archive/<생성 시각>_ring<링 수>_<해시>/` 아래 날짜별 사본(경로, 사이드카, 원본 경로, 생성에 쓴 `params.yaml`, 링 수와 토글을 적은 `config_summary.txt`) |
| Mission execution | `final_path.json`, `final_path_meta.json`, 2D 맵 | `analytics/paths/robot_path_<ts>.csv`, `analytics/logs/drive_debug_<ts>.csv`, `analytics/logs/stall_report_<ts>.csv`, `visualization/mission_execution/` 아래 플롯 |
| Surface profiling | `/velodyne_points`, TF, capture 서비스 | `analytics/pointclouds/combined_<ts>.pcd`, `combined_raw_<ts>.pcd`, `combined_filtered_<ts>.pcd`, `frames_<ts>.npz`, `floor_heatmap_<ts>.png`, 선택적으로 누적 영상 |

`final_path_meta.json`에는 path를 planning할 때 쓴 파라미터가 기록됨. executor는 이 중 실행에 영향을 주는 항목(로봇 폭, 벽 여유, boundary repass 설정, 라이다 장착 높이, 맵 해상도)을 현재 `params.yaml`과 대조하고, 다르면 실행을 거부함. 나머지 항목은 어떤 최적화 토글로 만든 path인지 남기는 용도일 뿐임. `map_preprocessed_data.npz`는 파일이 있으면 재사용되므로, `dae_file`을 바꿀 때는 먼저 지울 것.

두 launch에 `eval_run_label`을 주면 3, 4단계의 산출물이 `eval_runs/<label>/` 아래에 쌓이고, executor도 그 안에 저장된 path 스냅샷을 읽음. 스냅샷은 `run_generation_pipeline.py --snapshot-label <label>`로 만듦.

---

## 3. 최상위 파일

| 파일 | 역할 |
|---|---|
| `package.xml`, `setup.py`, `setup.cfg`, `resource/dae-coverage-floor-flatness` | ROS 2(ament_python) 패키지 정의. `setup.py`가 launch, config, behavior tree, URDF, world, model을 share 디렉터리에 설치하고 console script를 등록함 |
| `requirements-mission_generation.txt`, `requirements-surface_profiling.txt` | 워크스테이션용, 라이다 노트북용 Python 의존성 |
| `README.md`, `README.kr.md`, `CITATION.cff`, `LICENSES/Apache-2.0.txt` | 개요, 인용 정보, ROBOTIS / Open Robotics에서 파생된 파일의 라이선스 원문 |
| `config/params.yaml` | 알고리즘, 주행, 측정 파라미터 전부. 단계별 섹션으로 나뉨 |
| `config/tb3_waffle_nav2_params.yaml` | Nav2 스택 설정(footprint 다각형과 inflation이 있는 costmap, controller, behavior tree 경로) |
| `behavior_trees/navigate_through_poses_transit.xml`, `navigate_to_pose_transit.xml` | 측정하지 않는 transit 구간용 behavior tree. Regulated Pure Pursuit controller를 지정함 |
| `models/INU_9_211/`, `models/Apt/` | 건물 모델(`.dae` mesh + `model.sdf`). 파이프라인 입력이면서 시뮬레이션의 건물이기도 하며, `dae_file` 값을 따라감 |
| `urdf/turtlebot3_waffle.urdf.xacro`, `turtlebot3_waffle.gazebo.xacro` | 로봇 description과 Gazebo 플러그인(TurtleBot3에서 가져옴) |
| `worlds/coverage_flatness_env.world.xacro` | 시뮬레이션 world 템플릿. 선택한 건물 모델이 들어가도록 전개됨 |
| `rviz/tb3_navigation2.rviz` | Nav2용 RViz 구성 |
| `scripts/stack.py`, `scripts/stop_all.sh` | 로봇 쪽과 노트북 쪽 스택을 단계마다 확인하며 순서대로 올림([가이드](stack.md)). 관련 프로세스를 단계적으로 종료 |

## 4. `launch/`

| 파일 | 역할 |
|---|---|
| `sim_env.launch.py` | 시뮬레이션 환경 전체(Gazebo world, 로봇 스폰, robot state publisher, `imu_tilt_broadcaster`). 인자: `world`, `x_pose`, `y_pose`, `use_imu_tilt`, `use_sim_time` |
| `sim_robot_state_publisher.launch.py`, `sim_spawn_robot.launch.py` | 시뮬레이션용 로봇 description과 스폰(스폰 파일은 TurtleBot3 기반) |
| `real_bringup.launch.py`, `real_robot_state_publisher.launch.py` | 실기체 bring-up과 로봇 description |
| `tb3_waffle_nav2.launch.py` | Nav2, AMCL, map server(TurtleBot3 기반) |
| `mission_execution.launch.py` | `mission_executor` 노드. `is_sim`, `eval_run_label` 인자를 받음 |
| `surface_profiling.launch.py` | `surface_profiler` 노드. `is_sim`, `eval_run_label` 인자를 받음 |

---

## 5. `mission_generation/` (오프라인)

### 5.1 진입점

| 파일 | 역할 |
|---|---|
| `run_generation_pipeline.py` | environment modeling을 돌린 뒤 mission planning을 돌림. 이전 산출물이 있으면 맵을 다시 만들지, path를 다시 생성할지 물어봄. `--snapshot-label`을 주면 맵, topology, path, 디버그 이미지를 `eval_runs/<label>/`에 복사함 |

### 5.2 `environment_modeling/`

| 파일 | 역할 |
|---|---|
| `environment_modeler.py` | 단계 전체를 조율함(2D 맵 생성, 전처리, 분할, topology 저장) |
| `map_generator.py` | `.dae`의 장애물 삼각형을 `target_resolution`의 2D occupancy grid로 변환 |
| `map_preprocessor.py` | 건물 바깥과 닫힌 비바닥 영역을 지우고 가장 큰 연결 바닥 영역만 남김 |
| `space_segmenter.py` | 아래 분할 5단계를 실행하고 `final_topological_map.npz`를 저장 |
| `algorithms/limits.py` | Step 1. 로봇 반경 기준으로 바닥을 주행 가능 영역과 주행 불가(그래도 센싱 대상) 영역으로 나눔 |
| `algorithms/doors.py` | Step 2. 문에서 공간을 자르고, 로봇이 통과할 수 있는 문인지 판단 |
| `algorithms/holes.py` | Step 3. 기둥 같은 구멍을 찾아 그 주변에서 노드를 절단 |
| `algorithms/convexity.py` | Step 4. L자, U자 공간을 Solidity 기준으로 재귀 분할 |
| `algorithms/subdivider.py` | Step 5. 종횡비가 큰 노드를 분할 |
| `utils/visualizer.py` | 분할 단계별 디버그 이미지 |
| `__main__.py` | 이 단계만 단독으로 실행 |

### 5.3 `mission_planning/`

| 파일 | 역할 |
|---|---|
| `mission_planner.py` | planning 전체를 조율함(노드 방문 순서, 노드별 coverage, transit path, boundary repass 미리보기, path 조립) |
| `algorithms/tsp.py` | 인접 노드 사이의 연결 waypoint, A\*가 좁은 연결부를 지날 수 있게 하는 planning mask, 노드 방문 순서(연결 비용 그래프 위의 근사 TSP 순회, NetworkX 사용). 풀기 전에 노드를 무게중심 기준으로 정렬하므로 토폴로지 파일에 노드가 저장된 순서와 무관하게 같은 순서가 나옴 |
| `algorithms/pendant_reorder.py` | 허브에 붙은 말단 노드의 방문 순서를, 허브의 coverage가 실제로 끝나는 지점 기준으로 재배열 |
| `algorithms/coverage.py` | 노드 마스크를 Fields2Cover 셀로 바꾸고 swath를 생성. 옵션으로 Boustrophedon 분해와 swath 각도 탐색(`enable_optimal_swath_angle`)을 씀 |
| `algorithms/transit.py` | 회전, 벽 근접 페널티를 둔 노드 간 A\* |
| `utils/sampler.py` | path를 일정 간격으로 Resampling하고, 측정 구간의 코너 앞에 보조 waypoint를 추가 |
| `utils/coverage_check.py` | planning된 path만으로 LiDAR ring hit이 모자랄 셀을 예측함. 어느 transit을 측정 구간으로 쓸지 정하는 데 쓰임 |
| `utils/ring_coverage.py` | 예측과 repass 판단에 쓰는 VLP-16 ring 기하 |
| `utils/repass_preview.py` | boundary repass가 추가할 지점을 픽셀 공간에서 근사 |
| `utils/geometry.py` | 중심점, 거리 행렬 등 기하 보조 함수 |
| `utils/visualizer.py` | planning된 path의 디버그 이미지 |
| `translator.py` | 픽셀 path를 미터 단위 Nav2 pose(위치와 heading)로 변환 |
| `path_exporter.py` | Resampling 후 `raw_path.json`, `final_path.json`, `final_path_meta.json`을 저장하고 디버그 이미지에 waypoint를 겹쳐 그림 |
| `__main__.py` | 이 단계만 단독으로 실행 |

### 5.4 `test/`

| 파일 | 역할 |
|---|---|
| `check_executed_paths.py` | `utils/coverage_check.py`의 코너 컷 모델 자체 점검 |

---

## 6. `mission_execution/` (온라인)

`MissionExecutor`는 Mixin 세 개와 `utils/`의 보조 모듈로 조립된 ROS 노드임.

| 파일 | 역할 |
|---|---|
| `mission_executor.py` | 노드 본체. path를 읽어 직선 sub-segment로 나누고(heading이 `direction_change_threshold_deg` 넘게 바뀌거나 capture 플래그가 바뀌는 지점에서 새로 시작), 하나씩 실행하며 capture 구간을 열고 닫음. 코너에서 방향이 크게 되돌아 나가면(`subsegment_merge_max_turn_deg` 초과) 그 코너를 단독 목표로 보냄 |
| `localization_mixin.py` | 위치와 heading을 함께 확인하는 AMCL 초기화, jump 감지, `map→base_link` 조회. TF는 전용 스레드에서 받음 |
| `nav2_drive_mixin.py` | Nav2 액션 래퍼(`NavigateToPose`, `NavigateThroughPoses`, 제자리 회전, 후진)와 stall이 이어질 때의 단계적 복구 |
| `run_context_mixin.py` | path, 메타데이터, 파라미터를 읽고, 실행이 끝나면 주행 path, stall 보고서, 플롯을 저장 |
| `imu_tilt_broadcaster.py` | `base_footprint→base_link` 발행. `use_imu_tilt:=true`면 IMU roll, pitch에서 설정된 바이어스를 뺀 값을, 아니면 회전 0을 실음 |
| `utils/boundary_repass.py` | coverage 경계의 시작 prepass와 종료 repass. 경계 근처 LiDAR 사각지대를 나가는 길에 측정함 |
| `utils/controller_switch.py` | coverage와 transit 구간에 따라 Nav2 controller 설정(회전 임계, 속도 제한, behavior tree)을 전환 |
| `utils/ring_coverage.py` | coverage 종료 직후 구간이 이미 ring hit을 충분히 받는지 판단해 repass를 건너뛸지 결정 |
| `utils/mission_logger.py` | 로봇이 어디서 무엇을 하는지 주기적으로 기록(`drive_debug_<ts>.csv`) |
| `utils/stall_logger.py` | Nav2 feedback에서 stall을 감지해 `stall_report_<ts>.csv`로 저장 |
| `utils/nav2_utils.py` | `BasicNavigator`의 초기 pose 발행 패치(실시간 타임스탬프, 재발행 throttle) |
| `utils/ros_utils.py` | 맵 안으로 clamp한 `PoseStamped` 생성, Gazebo에서 로봇 텔레포트(시뮬레이션 시작 시) |
| `utils/map_utils.py` | YAML과 PGM에서 맵의 물리적 경계를 계산 |
| `utils/visualizer.py` | planning된 path의 벽 근접도와 stall 위치 플롯 |

---

## 7. `surface_profiling/` (온라인, 분석 도구 포함)

### 7.1 수집 노드

| 파일 | 역할 |
|---|---|
| `surface_profiler.py` | 노드 본체. capture 구간이 열려 있는 동안 점군을 모으고, 종료 후 합친 점군 저장, 바닥 추출, 히트맵 생성을 수행 |
| `tf_sync_mixin.py` | 점군 stamp 시각의 TF를 기다려 `map` 프레임으로 변환하고, 순간 속도가 큰 프레임은 버림 |
| `capture_services_mixin.py` | `mission_executor`가 호출하는 capture 시작, 종료, 완료 서비스 |
| `utils/frame_recorder.py` | 프레임 단위 기록(`frames_<ts>.npz`): 점, intensity, ring, pose, 플래그 |
| `utils/floor_extractor.py` | PCD에서 바닥 높이 대역에 든 점만 남김 |
| `utils/heatmap_generator.py` | 격자 셀의 평균 z를 고정 스케일로 칠해 2D 맵 위에 겹침 |
| `utils/pcd_io.py` | PCD를 CSV로 내보냄 |
| `utils/config_paths.py` | `params.yaml`의 디렉터리 설정을 절대 경로로 조합. profiler와 재처리 스크립트가 이 모듈로 경로를 맞춤 |
| `utils/frame_video.py` | 프레임 기록으로 top-down 누적 영상을 만듦 |

### 7.2 후처리와 분석

| 파일 | 역할 |
|---|---|
| `local_plane_heatmap.py` | 프레임 기록에서 국소 평면 대비 편차 히트맵과 비교 지표를 만듦([가이드](local_plane_heatmap.md)) |
| `reprocess_pcd.py` | 저장된 `combined_*.pcd`로 바닥 추출과 히트맵 생성을 다른 파라미터로 다시 실행. 재주행이 필요 없음 |
| `analyze_coverage_comparison.py` | path 변형 비교용 실행별 지표(완전성, gap, 거리, 시간, 회전량, 순주행시간) |
| `analyze_frame_log.py` | 셀별 점 수와 서로 다른 통과 횟수를 세어 repass가 더하는 효과를 확인 |
| `analyze_ring_coverage.py` | 실제 주행 궤적에서 LiDAR ring이 각 셀을 몇 번 스치는지 계산 |
| `analyze_min_detectable_height.py` | 프레임 기록에 합성한 돔으로, 돔 지름과 추정 방식별 최소 검출 높이를 구함 |
| `analyze_z_bias.py` | 바닥 z의 체계 오차(센서 기울기)와 무작위 노이즈를 분리 |
| `fit_heading_z_bias.py` | 프레임 기록으로 잔차 IMU pitch, roll 바이어스를 추정([IMU 보정](imu_calibration.md)) |
| `auto_calibration_drive.py`, `scan_room_for_calibration.py` | 4방향 라이다 장착 보정 데이터 수집([라이다 보정](lidar_calibration.md)) |
| `make_accumulation_video.py` | 프레임 기록으로 누적 영상을 다시 만듦 |
| `view_pcd.py` | PCD를 점 하나씩 z로 칠해 봄 |
| `utils/coverage_metrics.py` | coverage 지표를 계산하는 순수 함수 |
| `utils/ring_coverage.py` | 바닥 셀별 ring 스침 횟수 |
| `utils/local_plane.py` | 국소 평면 대비 잔차 |
| `utils/floor_adjustment.py` | 겹치는 관측으로 바닥 높이장과 프레임별 평면을 함께 추정 |
| `utils/wall_alignment.py` | 원시 점군의 정면 벽에서 heading 오차를 추정 |
| `utils/stall_report_analyzer.py` | stall 보고서를 주행 path와 대조해 분석, 시각화 |
| `utils/ssh_robot.py` | Jetson에 `cmd_vel` 같은 명령을 보내는 얇은 SSH 래퍼 |

### 7.3 `test/`

합성 데이터로 정답을 아는 상태에서 모듈을 확인하거나, 기록된 데이터의 정합성을 보는 진단 스크립트임. 직접 실행함(`python3 surface_profiling/test/<파일>.py`).

| 파일 | 확인하는 것 |
|---|---|
| `check_local_plane.py`, `check_floor_adjustment.py`, `test_wall_alignment.py` | `utils/local_plane.py`, `utils/floor_adjustment.py`, `utils/wall_alignment.py`를 합성 바닥과 돔으로 검증 |
| `check_frame_recorder.py` | 프레임 기록에서 intensity, ring이 점과 계속 정렬되어 있는지 |
| `check_chassis_state.py` | 돔 위를 지날 때의 차체 상태가 후처리 모드별로 미치는 영향 |
| `check_map_alignment.py` | 프레임 기록의 점군이 2D 맵 위에 맞게 놓였는지 |
| `check_imu_noise.py`, `check_imu_dynamics.py`, `check_imu_yaw_scan.py` | IMU 노이즈, 정지와 주행 중 roll / pitch 바이어스, 2D scan matching 대비 yaw 오차 |

---

## 8. 확장하는 곳

- **다른 건물**: 파이프라인용으로 `<name>.dae`를 `<workspace_root>/assets/`에 두고, 시뮬레이션용으로 `models/<name>/`(`meshes/<name>.dae`, `model.sdf`, `model.config`)를 추가함. 그 뒤 `params.yaml`의 `dae_file`을 바꾸고 `colcon build`를 실행, `maps/grid/map_preprocessed_data.npz`를 지우고 `run_generation_pipeline.py`를 다시 돌리면 됨.
- **새 path 최적화 토글**: `params.yaml`의 `mission_planner` 아래에 키를 추가하고 `mission_planner.py`에서 읽음. `final_path_meta.json`에 남으려면 `path_exporter.py`가 쓰는 항목에도 넣을 것.
- **측정 구간이나 격자 변경**: `params.yaml`의 `surface_profiling` 섹션. 저장된 데이터에 미치는 영향은 `reprocess_pcd.py`로 먼저 볼 수 있음.
