# 설정 가이드

[English](../en/configuration.md) · [한국어](configuration.md) · [← README](../../README.kr.md)

파이프라인 전체 설정은 [`config/params.yaml`](../../config/params.yaml) 하나에 단계별 섹션으로 묶여 있음. 수정 후에는 `colcon build`로 install 트리에 반영해야 함.

---

## 주요 파라미터

| 키 | 섹션 | 의미 |
|---|---|---|
| `dae_file` | `environment_modeling` | 처리할 모델. 시뮬레이션에서는 `worlds/coverage_flatness_env.world`도 같이 바꿔야 함. |
| `target_resolution` | `environment_modeling` | 맵 해상도(m/px). 작을수록 정밀하고 느림. |
| `max_door_radius` | `environment_modeling` | 문으로 간주할 개구부 최대 폭의 절반. |
| `max_aspect_ratio` | `environment_modeling` | 이 값을 넘으면 노드를 분할함(Step 5). |
| `robot_width` | `environment_modeling` | 로봇 **반경**(m). 주행 가능 영역과 계획 경로의 벽 여유 계산에 쓰임. |
| `path_safety_margin` | `mission_planner` | `robot_width`에 더해 계획 경로가 벽에서 추가로 유지할 여유. |
| `lidar_range`, `overlap`, `lidar_vertical_fov_deg`, `lidar_mount_height` | `mission_planner` | 센서 기하. Swath 폭과, 노드 버킷을 가르는 Blind Radius를 결정함. |
| `turn_weight`, `wall_weight` | `mission_planner` | A\* Transit 경로 페널티. |
| `coverage_speed_limit_mps` / `transit_speed_limit_mps` | `mission_execution` | 구간 종류별 주행 속도. |
| `direction_change_threshold_deg` | `mission_execution` | 새 직선 sub-segment를 시작할 heading 변화량. |
| `boundary_repass_distance_m` | `mission_execution` | Coverage 경계에서 재통과할 거리. Blind Radius의 2배를 넘겨야 함. |
| `z_min`, `z_max` | `surface_profiling` | 바닥 추출 z-window(m). 설계 바닥 높이의 위아래를 모두 포함해야 함. |
| `grid_size` | `surface_profiling` | Completeness와 Heatmap의 분석 셀 크기. |
| `save_raw_pcd`, `save_combined_csv`, `save_waypoint_pcd` | `surface_profiling` | 용량이 큰 선택적 산출물. 기본 off. |

하드웨어에 묶인 상수(`lidar_mount_height`, `robot_width`, `boundary_repass_distance_m`)는 센서를 재장착하거나 로봇을 교체하면 **반드시 다시 실측**해야 함.

## 설정 시 주의할 점

[`config/params.yaml`](../../config/params.yaml)에서 이름이나 형식 때문에 잘못 넣기 쉬운 값들임. `오류 표시`가 **없음**인 항목은 잘못 넣어도 주행이 정상 종료되고 결과만 달라지므로 특히 주의함.

| 키 | 주의할 점 | 오류 표시 |
|---|---|---|
| `robot_width` | 이름과 달리 **반경**(m)임. 차체 전폭을 넣으면 벽 여유가 두 배로 잡혀, 좁은 구역이 주행 대상에서 빠질 수 있음. | 없음 |
| `robot_width`, `path_safety_margin` | **계획 경로**의 벽 여유만 정함. Nav2 Costmap의 `footprint`·`inflation_radius`는 [`config/tb3_waffle_nav2_params.yaml`](../../config/tb3_waffle_nav2_params.yaml)에서 따로 맞춰야 함. | 없음 |
| `robot_width`, `path_safety_margin`, `boundary_repass_distance_m`, `enable_boundary_repass` | 경로 좌표가 이 값들에 의존함. 경로 생성 후 바꾸면 `mission_executor`가 시작을 거부하므로, 바꿨다면 경로 생성부터 다시 실행함. | 시작 거부 |
| `enable_boundary_repass` | `mission_execution` 섹션에 있지만 **경로 생성도** 이 값을 읽음. | 시작 거부 |
| `enable_*` 토글 5개 | `false`는 ablation 비교용 조건임. 실험 후 `true`로 되돌리지 않으면 이후 경로가 해당 최적화 없이 생성됨. | 없음 |
| `coverage_mode` | 문자열 `"full"` / `"centroid_only"`만 허용함. `centroid_only`는 Baseline 비교용이므로 실제 검사에는 `"full"`을 씀. | 경로 생성 시 예외 |
| `only_capture_at_waypoints` | 이름과 달리 "정지 지점에서만 측정"이 아님. `true`는 **Capture Window(Coverage 주행 + Boundary Repass) 안의 점만** 남기며, 주행 중 연속 측정은 그대로임. `false`로 두면 노드 간 Transit 구간의 점까지 섞임. | 없음 |
| `z_min`, `z_max` | 단위는 **m**임(mm 아님). 설계 바닥 높이(z = 0)의 **위아래를 모두** 포함해야 함 — `z_min: 0.0`이면 바닥 점의 아래쪽 절반이 빠짐. | 없음 |
| `voxel_size`, `grid_size` | `voxel_size`는 저장 시 Voxel Downsampling 간격, `grid_size`는 Completeness·Heatmap의 분석 격자임. `voxel_size`가 `grid_size`보다 크면 점이 들어갈 수 없는 셀이 생겨 Completeness가 낮게 나옴. | 없음 |
| `boundary_repass_distance_m` | Blind Radius의 **2배 이상**이어야 함. 세그먼트보다 길면 자동으로 줄어들지만(clamp), 설정값 자체가 작으면 경계 부근이 채워지지 않은 채 끝남. | 없음 |
| `lidar_mount_height` | 실측값을 넣어야 함. Blind Radius와 노드 폭 분류가 이 값에서 계산됨. | 없음 |
| `dae_file` | 시뮬레이션에서는 `worlds/coverage_flatness_env.world`의 모델도 같은 파일로 바꿔야 함. | 없음 |
| `save_raw_pcd` | 셀당 z 표준편차(노이즈) 지표를 볼 때만 켬. 주행당 저장 용량이 크게 늘어남. | — |
| `is_sim` (launch 인자) | 기본값이 `false`이고 `use_sim_time`도 이 값을 따름. 시뮬레이션에서 빠뜨리면 노드가 Gazebo 시계 대신 시스템 시계를 씀. `surface_profiler`와 `mission_executor`에 같은 값을 줘야 함. | 없음 |

`params.yaml`을 고친 뒤에는 `colcon build`로 install 트리에 반영하고 새 터미널에서 실행함. 실제로 적용 중인 값은 `reprocess_pcd.py`가 시작할 때 출력함.

ROS 2·Nav2·Velodyne 자체의 설정, Topic, 네트워크(DDS)는 각 공식 문서를 참고함 — [ROS 2 Humble](https://docs.ros.org/en/humble/), [Nav2](https://docs.nav2.org/), [Velodyne ROS 2 Driver](https://github.com/ros-drivers/velodyne/tree/humble-devel). 설치 과정에서 생기는 문제는 [설치 문서](installation.md#문제-해결)에 정리함.

---

[← README](../../README.kr.md)
