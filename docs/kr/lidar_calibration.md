# 3D 라이다 장착 자세 보정 · 고정 속도 주행 측정 가이드

미션 없이 **측정 때와 같은 고정 속도로 로봇을 직접 움직이며** 바닥 점군을 수집하고, 누적 영상으로 확인하고, 라이다 장착 자세(roll/pitch) 보정값을 구해 `params.yaml`에 넣는 절차임. Simulation과 Real-world에서 동일하게 씀.

## 1. 무엇을 보정하는가

TF가 알려주는 `velodyne_link` 자세와 센서의 실제 자세 사이에 고정 오프셋(roll/pitch)이 있으면, 평평한 바닥의 z가 센서에서 멀어질수록 한쪽으로 기움(`z ≈ a·전방거리 + b·좌측거리`). 오차는 거리에 비례함(1° 기울면 3m에서 약 5cm).

`surface_profiling` 노드는 센서 좌표계에 추가 회전을 곱해 이를 상쇄함.

```yaml
# config/params.yaml - surface_profiling
lidar_mount_correction_rpy_deg_sim:  [0.0, 0.0, 0.0]   # [roll, pitch, yaw] (deg), URDF joint rpy와 같은 규약
lidar_mount_correction_rpy_deg_real: [0.0, 0.0, 0.0]
```

- 보정은 수집 시점의 점 좌표에 바로 적용되며, 이후의 `.pcd`/`frames_*.npz`/히트맵/영상에 모두 반영됨.
- yaw는 이 절차로 구하지 않으므로 0으로 둠.
- `params.yaml`은 `colcon build --symlink-install`로 설치 경로에 링크되므로 값을 바꾼 뒤 재빌드 없이 노드만 다시 켜면 됨.

## 2. 키보드 주행 + 점군 수집

미션 실행(`mission_execution`)을 켜지 않고, 측정 노드의 캡처 서비스를 직접 호출해 수집 구간을 정함.

### 2-1. 실행 순서

**Simulation**

```bash
ros2 launch dae_coverage_floor_flatness sim_env.launch.py                                    # 1
ros2 launch dae_coverage_floor_flatness tb3_waffle_nav2.launch.py use_sim_time:=true          # 2
ros2 launch dae_coverage_floor_flatness surface_profiling.launch.py is_sim:=true              # 3
# 5. 아래 2-3절의 속도 명령으로 주행 (키보드 조작은 실측 속도와 달라 쓰지 않음)
```

**Real-world** (README의 Run order와 같은 순서에서 마지막 미션 실행 대신 키보드 사용)

```bash
ros2 launch dae_coverage_floor_flatness real_bringup.launch.py                                # [Jetson]
ros2 launch dae_coverage_floor_flatness tb3_waffle_nav2.launch.py use_sim_time:=false         # [Jetson]
ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py                                       # [Laptop]
ros2 launch dae_coverage_floor_flatness surface_profiling.launch.py is_sim:=false             # [Laptop]
# 아래 2-3절의 속도 명령으로 주행 (로봇에 명령이 닿는 머신에서)
```

- `map → velodyne_link` TF가 있어야 점이 기록됨. 미션 없이 AMCL만 쓰므로 RViz 등에서 초기 위치를 지정해 수렴시킨 뒤 시작함.
- 점군은 도면 기준 좌표(map)로 기록되므로, 이후 분석의 벽 제외(`--map`)와 영상의 벽 배경이 도면과 맞으려면 AMCL 위치가 정확해야 함.

### 2-2. 캡처 구간 제어

```bash
# 캡처 시작 (이 시점부터 들어오는 점만 결과에 반영됨)
ros2 service call /surface_profiling/start_waypoint_capture std_srvs/srv/Trigger

# ... 2-3절의 속도 명령으로 주행하거나 정지 유지 ...

# 캡처 종료
ros2 service call /surface_profiling/stop_waypoint_capture std_srvs/srv/Trigger

# 전체 수집 종료 -> 바닥 추출, 히트맵, 누적 영상까지 자동 생성
ros2 service call /surface_profiling/stop_collection_success std_srvs/srv/Trigger
```

- 캡처 구간을 여러 번 열고 닫아도 됨. 시작/종료는 항상 쌍으로 호출함.
- 미션 산출물(stall report 등)이 없으므로 종료 시 stall 분석 단계는 건너뛰거나 경고를 낼 수 있으며, 이는 앞 단계 결과에 영향을 주지 않음.

### 2-3. 고정 속도 주행

측정 속도는 `mission_execution.coverage_speed_limit_mps`(기본 0.16 m/s)임. 키보드 조작은 이 속도가 아니므로, 같은 값으로 `/cmd_vel`에 속도 명령을 일정 횟수만큼 발행해 주행함.

```bash
# 전진: 0.16 m/s x 20 Hz x 375회 = 약 3 m (거리 = 속도 x 횟수 / 주파수)
ros2 topic pub --times 375 -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.16}}"

# 후진
ros2 topic pub --times 375 -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.16}}"
```

- 이 명령은 Nav2 목표 없이 직접 발행하는 개루프 명령이므로 활성 Nav2 목표가 없는 상태에서 씀. 로봇 앞 공간을 확인한 뒤 실행함.
- 계단형 속도 명령이라 시작/정지 직후에 가감속이 있음. 캡처는 시작 1초 뒤에 열고 종료 1초 전에 닫아 정속 구간만 담음.
- 실제 미션의 속도 프로파일(경로 추종 컨트롤러의 가감속)과 완전히 같지는 않으므로, 정속 구간의 편향 확인용으로 씀.

| 항목 | 값 | 이유 |
|---|---|---|
| 선속도 | `coverage_speed_limit_mps`와 같은 값 | 측정 조건과 같은 점 간격을 얻기 위함 |
| 회전 | 캡처를 끈 상태에서 천천히 | 저역통과 필터 기본 임계(0.3 m/s, 20 deg/s)를 넘으면 그 프레임이 통째로 버려짐 |
| 바닥 | 벽에서 0.4m 이상 떨어진 평평한 구역 | 벽면 점이 z 통계를 오염시킴 |
| 라이다 회전수 | 600 rpm 고정 | 프레임당 점 밀도와 이동 간격이 이 값을 전제로 설계됨 |

기각된 프레임 수는 영상 상단 문구(`rejected_frames`)와 수집 로그로 확인함.

### 2-4. 권장 주행 패턴

| 목적 | 패턴 |
|---|---|
| 진행 방향 편향 확인 | 같은 직선(3~4m)을 **전진 → 후진** 각각 한 번씩 캡처. 색 편향이 로봇 앞/뒤에 고정되면 센서 자세, 주행 방향에 고정되면 동특성(자세 변화·지연) 문제임 |
| 장착 자세 보정값 산출 | 같은 자리에서 **0°, 90°, 180°, 270°** 방향으로 각각 정지해 **같은 시간(예: 10초)** 캡처. 방향별 프레임 수를 같게 해야 바닥 자체의 경사가 평균에서 상쇄됨 |

## 3. 누적 영상으로 확인

수집 종료 시 `visualization_dir`(기본 `~/dae_floor_maps/visualization/surface_profiling/`)에 `accumulation_<timestamp>.mp4`가 자동 생성됨. 도면 벽과 로봇은 검정, 점은 z 높이 색임.

| 색 | 의미 |
|---|---|
| jet (파랑→빨강) | 셀에 누적된 z 창 안 점의 **평균 z**. 히트맵과 같은 컬러맵·같은 집계·같은 색 범위 |
| 회색 | 셀에 z 창 아래 점만 있음 |
| 자홍 | 셀에 z 창 위 점만 있음 (벽면 점이 주로 여기에 해당) |

최종 프레임은 히트맵과 같은 값으로 수렴함(히트맵은 1cm voxel 다운샘플 후 평균이라 미세한 차이만 있음). 히트맵 컬러바는 실제 z[cm] 눈금임.

저장된 기록에서 옵션을 바꿔 다시 만들 수 있음.

```bash
python3 surface_profiling/make_accumulation_video.py \
  ~/dae_floor_maps/analytics/pointclouds/frames/frames_<timestamp>.npz \
  --z-min -0.01 --z-max 0.01 --z-margin 0.05
```

- 색 범위를 좁힐수록(예: ±1 cm) 미세한 차이가 크게 보임. 관측 횟수가 적은 셀(로봇 앞쪽에 처음 보이는 영역)은 평균이 덜 수렴해 잡음(sim 기준 표준편차 약 0.2 cm)이 색으로 드러나므로, 색만으로 편향을 판단하지 말고 4절 수치로 확인함. 벽 근처는 벽면 점이 섞여 z가 높게 나옴.
- 옵션: `--fps`, `--max-frames`(영상 프레임 수 상한), `--max-dim`(해상도), `--out`.

## 4. 수치로 검증하고 보정값 구하기

```bash
python3 surface_profiling/analyze_z_bias.py \
  ~/dae_floor_maps/analytics/pointclouds/frames/frames_<timestamp>.npz \
  --map ~/dae_floor_maps/maps/grid/map_from_dae.yaml
```

출력 예:

```
z(cm): mean 0.007  std 0.222  p1/p99 -0.52/0.54
pitch(deg, +=전방이 높음): mean 0.000  std(프레임 간) 0.007
roll (deg, +=좌측이 높음): mean 0.003  std(프레임 간) 0.009
제안값: lidar_mount_correction_rpy_deg = [-0.003, 0.000, 0.0]
```

- 벽에서 `--wall-margin`(기본 0.4m) 이내 점은 제외하고, 센서 거리 1.2~3.5m(`--r-min/--r-max`)의 바닥 점(|z| ≤ `--z-band`, 기본 0.1m)만 씀. 프레임마다 평면 `z = a·f + b·l + c`를 최소제곱으로 맞춰 pitch=atan(a), roll=atan(b)를 구함.
- **프레임 간 표준편차**가 작으면 고정 오프셋(보정 대상), 크면 주행 중 변하는 기울기임. 고정 오프셋이 아니면 이 절차로는 보정되지 않음.
- 방향 편향 확인: 전진/후진 데이터를 각각 분석해 pitch 부호가 로봇 기준으로 유지되는지 봄.

### 4-1. 보정값 적용

1. 보정을 끈 상태(`[0, 0, 0]`)로 2-4의 4방향 데이터를 수집함.
2. 4방향 데이터를 한 파일로 수집해(캡처 구간을 열고 닫으며 한 번의 실행에서) `analyze_z_bias.py`를 실행함. 프레임을 방향별로 같은 수만큼 모았다면 평균 pitch/roll이 마운트 기울기임.
3. 출력된 `제안값`을 `params.yaml`의 해당 키(`_sim` 또는 `_real`)에 넣음.
4. `surface_profiling` 노드를 다시 켜고 같은 방식으로 재수집해 pitch/roll이 0에 가까워졌는지 확인함.
5. 보정을 켠 채 수집한 데이터로 다시 분석했다면, 출력의 제안값은 **현재 값에 더해야** 하는 잔여분임.

부호 규약: `pitch`는 URDF joint rpy와 같은 방향(양수면 센서 좌표계를 y축 기준으로 회전)이며, 분석이 출력하는 제안값을 그대로 쓰면 부호 변환을 직접 할 필요 없음.

### 4-2. 판정 기준

| 지표 | 기준(잠정, 현장에서 확정) |
|---|---|
| \|pitch\|, \|roll\| 평균 | 목표 결함 높이/최대 사용 거리보다 충분히 작을 것(예: 1cm/3.5m ≈ 0.16°의 1/3 이하) |
| 전방/후방 평균 z 차이 | 결함 높이의 1/10 이하 |
| 프레임 간 pitch 표준편차 | 평균 오프셋보다 충분히 작을 것(작아야 고정 오프셋으로 다룰 수 있음) |

## 5. 한계

- 바닥이 평평하다는 가정으로 평면을 맞추므로, **평탄도를 재려는 바로 그 바닥**에서 보정하면 순환임. 4방향 평균은 바닥 경사가 로봇과 함께 돌지 않는다는 점을 이용해 이를 줄이는 방법임.
- 이 보정은 **고정** 오프셋만 다룸. 결함 위를 지날 때 차체가 기우는 것 같은 동적 기울기는 보정되지 않음(현재 TF는 평면이라 roll/pitch가 반영되지 않음).
- 사용 거리 범위(1.2~3.5m)와 프레임당 최소 점 수(300)는 기본값이며, 다른 센서/높이에서는 `--r-min`, `--r-max`, `--min-points`를 조정함.
