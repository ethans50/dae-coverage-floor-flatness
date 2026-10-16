# 설정 가이드

[English](../en/configuration.md) · [한국어](configuration.md) · [← README](../../README.kr.md)

파이프라인 전체 설정은 [`config/params.yaml`](../../config/params.yaml) 하나에 단계별 섹션으로 묶여 있음. 수정한 뒤에는 `colcon build`로 install 트리에 반영해야 적용됨.

---

## 주요 파라미터

| 키 | 섹션 | 의미 |
|---|---|---|
| `dae_file` | `environment_modeling` | 처리할 모델. 시뮬레이션 world의 건물도 이 값을 따라감. |
| `target_resolution` | `environment_modeling` | 맵 해상도(m/px). 작을수록 정밀하고 느림. |
| `max_door_radius` | `environment_modeling` | 문으로 간주할 개구부 최대 폭의 절반. |
| `max_aspect_ratio` | `environment_modeling` | 이 값을 넘으면 노드를 분할함(Step 5). |
| `robot_width` | `environment_modeling` | 로봇 **반경**(m). 주행 가능 영역과 planning 경로의 벽 여유 계산에 쓰임. |
| `path_safety_margin` | `mission_planner` | `robot_width`에 더해 planning 경로가 벽에서 추가로 유지할 여유. |
| `lidar_range`, `overlap`, `lidar_vertical_fov_deg`, `lidar_mount_height` | `mission_planner` | 센서 기하. Swath 폭(`lidar_range` × (1 − `overlap`))과, 노드 버킷을 가르는 Blind Radius를 결정함. `lidar_range`는 분석에 쓰는 가장 먼 범위로, `floor_ring_max`가 가리키는 링의 상한에 맞춰 둠(마운트 높이 0.338 m에서 링 4는 2.44 m, 링 5는 3.0 m). `python3 run_generation_pipeline.py --ring 4`(또는 `--ring 5`)를 주면 `params.yaml`을 고치지 않고 그 실행에서만 두 값을 함께 지정함. |
| `floor_ring_max` | `mission_planner` | repass 건너뛰기 판정과 부족 셀 예측이 세는 하향 빔 링 수(1~5). 경로 메타에 기록되고, executor는 메타 값을 따라 같은 판정을 함(메타에 없으면 4). |
| `turn_weight`, `wall_weight` | `mission_planner` | A\* Transit 경로 페널티. |
| `enable_transit_capture` | `mission_planner` | `true`면 경로 생성 시 planning된 경로만으로 링 히트가 모자랄 바닥 셀(Blind Radius의 2배보다 짧은 직선 구간은 중앙이 빔)을 예측하고, 노드 뒤 transit을 측정하면 그 셀이 메워질 때만 그 transit의 캡처를 켬. 못 메우는 셀은 개수와 위치만 보고하고 경로는 바꾸지 않음. |
| `tsp_solver` | `mission_planner` | 방문 순서 풀이 방법. `christofides`(기본값)는 근사 해이고 입력 노드를 정렬해 동점 처리를 고정함. `exact`는 같은 노드 간 비용으로 모든 열린 경로 순서를 비교해 가장 싼 순서를 고르고, 비용이 같으면 노드 id 사전순으로 골라 입력 순서나 부동소수점 반올림에 영향받지 않게 함. 노드 9개 이하만 지원하고 넘으면 `christofides`로 되돌아감. |
| `enable_corner_reach` | `mission_planner` | `true`면 측정 구간의 코너 바로 앞에 보조 웨이포인트를 둠. 실행기가 직선 sub-segment를 목표 목록으로 Nav2에 보내는데, Nav2는 로봇이 0.7 m(transit BT는 0.3 m) 안에 들어온 중간 목표를 지우므로 보조점이 없으면 코너를 잘라 그 주변 셀의 측정이 부족해짐. |
| `coverage_speed_limit_mps` / `transit_speed_limit_mps` | `mission_execution` | 구간 종류별 주행 속도. |
| `direction_change_threshold_deg` | `mission_execution` | 새 직선 sub-segment를 시작할 heading 변화량. |
| `subsegment_merge_max_turn_deg` / `subsegment_merge_min_leg_m` | `mission_execution` | 고립된 코너점은 보통 다음 점과 함께 Nav2에 보냄. 코너에서 경로가 이 각도보다 크게 되돌아 나가고 앞뒤 구간이 모두 이 길이(m) 이상이면 코너를 단독 목표로 먼저 도달함. |
| `boundary_repass_distance_m` | `mission_execution` | Coverage 경계에서 재통과할 거리. Blind Radius의 2배를 넘겨야 함. |
| `z_min`, `z_max` | `surface_profiling` | 바닥 추출 z-window(m). 설계 바닥 높이의 위아래를 모두 포함해야 함. |
| `grid_size` | `surface_profiling` | Completeness와 Heatmap의 분석 셀 크기. |
| `save_raw_pcd`, `save_combined_csv`, `save_waypoint_pcd` | `surface_profiling` | 용량이 큰 선택적 산출물. 기본 off. |
| `save_frame_log`, `save_accumulation_video` | `surface_profiling` | 프레임 단위 기록(`pointclouds/frames/frames_*.npz`, 점별 반사 강도 포함)과 바닥 점 top-down 누적 영상. 점군 커버리지 디버깅용으로 기본 on. `frame_log_z_min/max`, `video_*`로 조정함. |

하드웨어에 묶인 상수(`lidar_mount_height`, `robot_width`, `boundary_repass_distance_m`)는 센서를 재장착하거나 로봇을 교체하면 **반드시 다시 실측**해야 함.

## 파라미터 설정 시 주의할 점

[`config/params.yaml`](../../config/params.yaml)에서 이름이나 형식 때문에 잘못 넣기 쉬운 값들임. `오류 표시`가 **없음**인 항목은 잘못 넣어도 주행이 정상 종료되고 결과만 달라지므로 특히 주의할 것.

| 키 | 주의할 점 | 오류 표시 |
|---|---|---|
| `robot_width` | 이름과 달리 **반경**(m)임. 차체 전폭을 넣으면 벽 여유가 두 배로 잡혀, 좁은 구역이 주행 대상에서 빠질 수 있음. | 없음 |
| `robot_width`, `path_safety_margin` | **planning 경로**의 벽 여유만 정함. Nav2 Costmap의 `footprint`·`inflation_radius`는 [`config/tb3_waffle_nav2_params.yaml`](../../config/tb3_waffle_nav2_params.yaml)에서 따로 맞춰야 함. | 없음 |
| `robot_width`, `path_safety_margin`, `boundary_repass_distance_m`, `enable_boundary_repass` | 경로 좌표가 이 값들에 의존함. 경로 생성 후 바꾸면 `mission_executor`가 시작을 거부하므로, 바꿨다면 경로 생성부터 다시 실행함. | 시작 거부 |
| `enable_boundary_repass` | `mission_execution` 섹션에 있지만 **경로 생성도** 이 값을 읽음. | 시작 거부 |
| `enable_*` 토글 7개 | `false`는 ablation 비교용 조건임. 실험 후 `true`로 되돌리지 않으면 이후 경로가 해당 최적화 없이 생성됨. | 없음 |
| `coverage_mode` | 문자열 `"full"` / `"centroid_only"`만 허용함. `centroid_only`는 Baseline 비교용이므로 실제 검사에는 `"full"`을 씀. | 경로 생성 시 예외 |
| `only_capture_at_waypoints` | 이름과 달리 "정지 지점에서만 측정"이 아님. `true`는 **Capture Window(Coverage 주행 + Boundary Repass + planner가 측정으로 표시한 Transit) 안의 점만** 남기며, 주행 중 연속 측정은 그대로임. `false`로 두면 노드 간 Transit 구간의 점까지 섞임. | 없음 |
| `z_min`, `z_max` | 단위는 **m**임(mm 아님). 설계 바닥 높이(z = 0)의 **위아래를 모두** 포함해야 함. `z_min: 0.0`이면 바닥 점의 아래쪽 절반이 빠짐. | 없음 |
| `voxel_size`, `grid_size` | `voxel_size`는 저장 시 Voxel Downsampling 간격, `grid_size`는 Completeness·Heatmap의 분석 격자임. `voxel_size`가 `grid_size`보다 크면 점이 들어갈 수 없는 셀이 생겨 Completeness가 낮게 나옴. | 없음 |
| `boundary_repass_distance_m` | Blind Radius의 **2배 이상**이어야 함. 세그먼트보다 길면 자동으로 줄어들지만(clamp), 설정값 자체가 작으면 경계 부근이 채워지지 않은 채 끝남. | 없음 |
| `lidar_mount_height` | 실측값을 넣어야 함. Blind Radius와 노드 폭 분류가 이 값에서 계산됨. | 없음 |
| `dae_file` | 시뮬레이션 world의 건물이 이 값을 따라감. `models/` 아래에 확장자를 뗀 이름과 같은 Gazebo 모델이 있어야 하고, 없으면 시뮬 launch가 중단됨. | 시작 거부 |
| `save_raw_pcd` | 셀당 z 표준편차(노이즈) 지표를 볼 때만 켬. 주행당 저장 용량이 크게 늘어남. | — |
| `is_sim` (launch 인자) | 기본값이 `false`이고 `use_sim_time`도 이 값을 따름. 시뮬레이션에서 빠뜨리면 노드가 Gazebo 시계 대신 시스템 시계를 씀. `surface_profiler`와 `mission_executor`에 같은 값을 줘야 함. | 없음 |

`params.yaml`을 고친 뒤에는 `colcon build`로 install 트리에 반영하고 새 터미널에서 실행함. 실제로 어떤 값이 적용 중인지는 `reprocess_pcd.py`가 시작할 때 출력해주므로 그것으로 확인하면 됨.

ROS 2·Nav2·Velodyne 자체의 설정, Topic, 네트워크(DDS)는 각 공식 문서를 참고할 것: [ROS 2 Humble](https://docs.ros.org/en/humble/), [Nav2](https://docs.nav2.org/), [Velodyne ROS 2 Driver](https://github.com/ros-drivers/velodyne/tree/humble-devel). 설치 과정에서 생기는 문제는 [설치 문서](installation.md#문제-해결)에 정리해둠.

---

[← README](../../README.kr.md)
