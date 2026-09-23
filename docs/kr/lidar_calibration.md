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

### 2-1. 실행 순서 (총 6개 터미널)

**Simulation**

```bash
# T1: Gazebo 환경
ros2 launch dae_coverage_floor_flatness sim_env.launch.py

# T2: Nav2 + AMCL
ros2 launch dae_coverage_floor_flatness tb3_waffle_nav2.launch.py use_sim_time:=true

# T3: 측정 노드
ros2 launch dae_coverage_floor_flatness surface_profiling.launch.py is_sim:=true

# T4: 2-2절 캡처 제어 (서비스 호출)
# T5: 2-3절 속도 명령 발행
```

**Real-world** (총 6개 터미널)

```bash
# === Jetson ===
# T1: 로봇 드라이버 + TF 퍼블리셔
ros2 launch dae_coverage_floor_flatness real_bringup.launch.py

# T2: Nav2 + AMCL (Jetson의 IMU/wheel odometry로 초기 위치 수렴)
ros2 launch dae_coverage_floor_flatness tb3_waffle_nav2.launch.py use_sim_time:=false

# T3: 속도 명령 발행 (Jetson에서 cmd_vel 퍼블리시)
# → 2-3절의 속도 명령을 여기서 실행

# === Laptop (3D 라이다 직결) ===
# T4: Velodyne 드라이버
ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py

# T5: 측정 노드 (laptop에서 실행되어 시간 동기화)
ros2 launch dae_coverage_floor_flatness surface_profiling.launch.py is_sim:=false

# T6: 캡처 제어 (laptop에서 서비스 호출)
# → 2-2절의 start/stop 호출을 여기서 실행
```

**실행 전 점검**

- `map → velodyne_link` TF가 있어야 점이 기록됨. 미션 없이 AMCL만 쓰므로 RViz에서 초기 위치를 지정해 수렴시킨 뒤 시작함.
- 점군은 도면 기준 좌표(map)로 기록되므로, 이후 분석의 벽 제외(`--map`)와 영상의 벽 배경이 도면과 맞으려면 AMCL 위치가 정확해야 함.

### 2-2. 캡처 구간 제어 (Laptop T6 터미널)

워크플로우:

```bash
# 1단계: 캡처 시작 (이 시점부터 들어오는 점만 결과에 반영됨)
ros2 service call /surface_profiling/start_waypoint_capture std_srvs/srv/Trigger

# 2단계: Jetson T3에서 속도 명령 발행 (2-3절 참조)
#        예: ros2 topic pub --times 375 -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.16}}"

# 3단계: 주행 후 정지 명령 (로봇을 즉시 정지)
#        → Jetson T3에서 실행 (또는 Laptop에서 Jetson으로 ssh해 실행)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}}"

# 4단계: 캡처 종료 (정지 후 2-3절의 가감속 시간과 유사하게 약 1초 대기 추천)
ros2 service call /surface_profiling/stop_waypoint_capture std_srvs/srv/Trigger

# 5단계: 전체 수집 종료 → 바닥 추출, 히트맵, 누적 영상까지 자동 생성
ros2 service call /surface_profiling/stop_collection_success std_srvs/srv/Trigger
```

**각 단계별 실행 위치:**

| 단계 | 터미널 | 머신 | 명령 |
|------|--------|------|------|
| 1 | Laptop T6 | 측정 노트북 | `start_waypoint_capture` 호출 |
| 2 | Jetson T3 | 로봇 | 속도 명령 발행 (전진/후진) |
| 3 | Jetson T3 | 로봇 | 정지 명령 발행 (`{linear: {x: 0.0}}`) |
| 4 | Laptop T6 | 측정 노트북 | `stop_waypoint_capture` 호출 |
| 5 | Laptop T6 | 측정 노트북 | `stop_collection_success` 호출 |

**주의:**

- 정지 명령(3단계)은 **Jetson T3에서** 속도를 0으로 발행해야 로봇이 정지함.
- 캡처 구간을 여러 번 열고 닫아도 됨 (1~4단계 반복 가능).
- 정지 → 캡처 종료 사이에 1초 정도 대기하면 가감속 오버슈트 제거 가능.

### 2-3. 고정 속도 주행 (Jetson T3 터미널)

측정 속도는 `mission_execution.coverage_speed_limit_mps`(기본 0.16 m/s)임. 키보드 조작은 이 속도가 아니므로, 같은 값으로 `/cmd_vel`에 속도 명령을 발행함.

**명령어 (Jetson T3에서 실행):**

```bash
# 전진: 0.16 m/s x 20 Hz x 375회 = 약 3 m (거리 = 속도 x 횟수 / 주파수)
ros2 topic pub --times 375 -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.16}}"

# 후진
ros2 topic pub --times 375 -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.16}}"

# 정지 (주행 완료 후 즉시 실행할 것, 2-2절 3단계)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}}"
```

**워크플로우:**

1. Laptop T6에서 `start_waypoint_capture` 호출
2. Jetson T3에서 위의 전진/후진 명령 실행 (자동으로 계산된 시간 동안 주행)
3. Jetson T3에서 정지 명령 실행 (로봇 멈춤)
4. 약 1초 대기 (가감속 오버슈트 소멸)
5. Laptop T6에서 `stop_waypoint_capture` 호출

**주의:**

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

### 2-5. 정지 캡처 중 자세(pose) 드리프트 확인

4방향 평균이 바닥 실제 경사를 상쇄하려면, 각 방향에서 로봇이 **그 방향으로 완전히 고정**돼 있어야 함. AMCL/오도메트리 잡음으로 정지 중에도 위치·yaw가 미세하게 흔들릴 수 있으므로, 캡처 후 아래처럼 확인함.

```bash
python3 -c "
from surface_profiling.utils.frame_recorder import load_frame_log, used_frame_mask
import numpy as np
log = load_frame_log('frames_<timestamp>.npz')
poses = log['poses']
mask = used_frame_mask(log)
yaw_deg = np.degrees(poses[mask, 3])
print(f'yaw std: {yaw_deg.std():.2f} deg, range: {yaw_deg.max() - yaw_deg.min():.2f} deg')
"
```

- 한 방향 구간 안에서 yaw 표준편차가 1° 이상이면 그 구간 동안 로봇이 흔들린 것이므로, 해당 방향은 재촬영을 권장.
- 다만 이동 중 데이터(2-3절 고정 속도 주행)에서는 yaw가 원래 계속 변하므로 이 확인이 의미 없음 — 정지 캡처(4방향 산출용)에만 적용.

### 2-6. 자동화 도구 (노트북 한 대로 전체 절차 실행)

2-1~2-5절의 수동 절차를 노트북 한 대에서 명령 두 개로 대신 실행하는 스크립트임. 로봇을 매 방향마다 완전히 정지시키는 대신, 각 방향에서 **후진→전진→후진(원위치 복귀)** 왕복 주행으로 더 넓은 바닥 영역을 사용하고, 방향 전환은 벽면을 라이다로 재검출해 자동 정렬함.

**1단계: 방 스캔 → 배치 가이드**

```bash
python3 surface_profiling/scan_room_for_calibration.py --duration 6 --out room_placement_guide.png
```

라이다를 방 아무 곳에나 세워두고 실행하면, 그 자리에서 몇 초간 모은 원시 포인트(맵/AMCL 불필요)로 방 외곽을 검출해 "여기서 전방 X m, 좌측 Y m 이동 + Z도 회전"하면 되는 배치 지점과 4벽까지의 거리를 이미지로 저장함.

**2단계: 자동 4방향 캘리브레이션 주행**

`--detect-only` 외의 모드는 캡처 서비스를 실제로 호출하므로(`--dry-run`도 마찬가지), 2-1절의 Real-world 6터미널(T1 Jetson bringup, T2 Nav2/AMCL, T4 Velodyne, T5 `surface_profiling.launch.py`)이 먼저 다 떠 있어야 함.

```bash
# 먼저 벽 각도 검출기가 실제 방에서 타당한 부호로 나오는지 확인 (로봇을 직접 살짝 돌려보기)
# --wall-r-max는 1단계에서 출력된 벽까지 거리보다 크게 잡을 것 (기본값 5m로는 부족한 넓은 방이면 상향)
python3 surface_profiling/auto_calibration_drive.py --detect-only --wall-r-max 6.0

# 전체 절차를 실제 이동 없이 훑어보기
python3 surface_profiling/auto_calibration_drive.py --reverse-m 1.5 --forward-m 3.0 --dry-run

# 실제 실행
python3 surface_profiling/auto_calibration_drive.py --reverse-m 1.5 --forward-m 3.0
```

`--host`/`--password`는 매번 입력하는 대신 터미널에서 한 번만 export해두면 이후 생략 가능함:

```bash
export ROBOT_HOST=192.168.0.0
export SSH_PASSWORD=0000
```

`--reverse-m`/`--forward-m`은 방 크기에 맞춰 조정함(방이 정사각형이 아니면 한 변 기준으로 왕복 거리를 다르게 줌). 나머지 옵션과 안전장치(회전 최대 스텝, 정렬 타임아웃 등)는 스크립트 내 `--help` 참고.

## 3. 누적 영상으로 확인

수집 종료 시 `visualization_dir`(기본 `~/dae_floor_maps/visualization/surface_profiling/`)에 `accumulation_<timestamp>.mp4`가 자동 생성됨. 도면 벽과 로봇은 검정, 점은 z 높이 색임.

| 색 | 의미 |
|---|---|
| jet (파랑→빨강) | 셀에 누적된 z 창 안 점의 **평균 z**. 히트맵과 같은 컬러맵·같은 집계·같은 색 범위 |
| 회색 | 셀에 z 창 아래 점만 있음 |
| 자홍 | 셀에 z 창 위 점만 있음 (벽면 점이 주로 여기에 해당) |

최종 프레임은 히트맵과 같은 값으로 수렴함(히트맵은 1cm voxel 다운샘플 후 평균이라 미세한 차이만 있음). 히트맵 컬러바는 실제 z[cm] 눈금임.

### 3-1. 영상 옵션과 재생성

저장된 기록에서 옵션을 바꿔 다시 만들 수 있음.

```bash
python3 surface_profiling/make_accumulation_video.py \
  ~/dae_floor_maps/analytics/pointclouds/frames/frames_2026-09-22_13-11-03.npz \
  --z-min -0.02 --z-max 0.02 --z-margin 0.05
```

- 색 범위를 좁힐수록(예: ±1 cm) 미세한 차이가 더 크게 보임. 색 범위를 벗어나는 값은 회색과 핑크색으로 일괄 표시함. 관측 횟수가 적은 셀(로봇 앞쪽에 처음 보이는 영역)은 평균이 덜 수렴해 잡음(sim 기준 표준편차 약 0.2 cm)이 색으로 드러나므로, 색만으로 편향을 판단하지 말고 4절 수치로 확인함. 벽 근처는 벽면 점이 섞여 z가 높게 나옴.
- 옵션: `--fps`, `--max-frames`(영상 프레임 수 상한), `--max-dim`(해상도), `--out`.

### 3-2. 다른 공간에서 캘리브레이션 (맵 없이)

맵이 없거나 맵 좌표와 맞지 않으면 영상이 왜곡될 수 있음. **4.의 수치 분석은 맵 없이도 진행 가능**함.

```bash
# 맵 없이: 센서 범위 1.2~3.5m, z 범위 ±10cm 내의 모든 점 사용
python3 surface_profiling/analyze_z_bias.py \
  ~/dae_floor_maps/analytics/pointclouds/frames/frames_2026-09-22_13-11-03.npz
```

벽 제외 대신 센서 거리 범위로 자동 필터링함. 더 많은 점이 포함되므로 잡음이 늘 수 있으나, 순수 바닥 기울기(pitch/roll)를 찾기에는 충분함.

## 4. 수치로 검증하고 보정값 구하기

### 4-0. 명령어 (두 가지 모드)

**맵이 있을 때 (권장: 벽 제외)**

```bash
python3 surface_profiling/analyze_z_bias.py \
  ~/dae_floor_maps/analytics/pointclouds/frames/frames_<timestamp>.npz \
  --map ~/dae_floor_maps/maps/grid/map_from_dae.yaml
```

**맵이 없을 때 (센서 거리만 이용)**

```bash
python3 surface_profiling/analyze_z_bias.py \
  ~/dae_floor_maps/analytics/pointclouds/frames/frames_2026-09-22_13-11-03.npz
```

두 모드 모두 같은 원리로 pitch/roll을 계산함. 맵 모드가 벽면 점을 제외해 더 깔끔함.

### 4-1. 출력 읽기

```
z(cm): mean 0.007  std 0.222  p1/p99 -0.52/0.54
pitch(deg, +=전방이 높음): mean 0.000  std(프레임 간) 0.007
roll (deg, +=좌측이 높음): mean 0.003  std(프레임 간) 0.009
제안값: lidar_mount_correction_rpy_deg = [-0.003, 0.000, 0.0]
```

**각 항목 해석:**

| 항목 | 의미 | 판단 기준 |
|------|------|----------|
| `z mean/std` | 바닥 높이 편향의 평균과 표준편차(cm) | 작을수록 좋음. ±0.2cm 이하 목표 |
| `pitch mean` | 전방 기울기(도). 양수면 센서 앞쪽이 높음 | 이것이 센서의 고정 오프셋 |
| `pitch std(프레임 간)` | 프레임 간 pitch 변화. 작을수록 고정 오프셋 | 평균의 1/5 이하이면 고정 오프셋으로 봄 |
| `roll mean` | 좌측 기울기(도). 양수면 센서 왼쪽이 높음 | 이것이 센서의 고정 오프셋 |
| `roll std(프레임 간)` | 프레임 간 roll 변화 | pitch와 동일 기준 |
| `제안값` | 바로 `params.yaml`에 넣을 수 있는 값 | 다음 4-3절 참조 |

### 4-2. 계산 원리

**거리 범위 선택 (중요)**

```
센서 거리: 1.2~3.5m (기본값, --r-min/--r-max로 조정 가능)
```

- **1.2m 이상**: 센서 바로 아래(0~1.2m)는 강한 반사·노이즈가 많으므로 제외.
- **3.5m 이하**: 너무 먼 점은 신호 감쇠로 신뢰도 낮음.
- **실제 측정 거리와 맞춰야 함**: 로봇이 실제로 바닥을 스캔하는 거리가 다르면 조정 필요.
  - 예: 로봇 바로 아래 0.5m부터 측정하면 → `--r-min 0.5`
  - 예: 더 먼 거리 4m까지 포함하면 → `--r-max 4.0`

**필터링**

- 벽에서 `--wall-margin`(기본 0.4m) 이내 점은 제외 (벽면 점이 z 통계를 오염).
- 바닥 높이 범위: |z| ≤ `--z-band`(기본 0.1m = ±10cm).
  - 이 범위 밖의 점(천장, 벽)은 제외.

**계산**

- 프레임마다 평면 `z = a·f + b·l + c`를 최소제곱으로 맞춰 pitch=atan(a), roll=atan(b)를 구함. f, l은 **그 프레임 자신의 yaw**로 투영한 로봇 좌표계임.
- **프레임 간 표준편차**가 작으면 고정 오프셋(보정 대상), 크면 주행 중 변하는 기울기임. 고정 오프셋이 아니면 이 절차로는 보정되지 않음.
- 방향 편향 확인: 전진/후진 데이터를 각각 분석해 pitch 부호가 로봇 기준으로 유지되는지 봄.
- **yaw 범위 주의**: f, l이 프레임별 yaw로 투영되므로, 데이터의 yaw가 좁은 범위(예: 방 하나를 한 방향으로만 통과)에서만 움직이면 바닥 자체의 실제 경사가 상쇄되지 않고 pitch/roll에 그대로 섞임. 이동 중 수집한 데이터로 나온 제안값은 "센서 편향 + 그 구간 바닥의 실제 국소 경사"가 합쳐진 값일 수 있으므로, 정지 4방향(2-4, 2-5절) 데이터로 검증해야 함.

### 4-3. 보정값 적용 및 재검증 절차

**목표:** 첫 측정에서 pitch/roll을 구하고, 보정값을 넣은 뒤 재측정해 수치 개선 확인.

**단계 1: 초기 측정 (보정 비활성)**

```yaml
# config/params.yaml - 현재 상태 확인
lidar_mount_correction_rpy_deg_sim:  [0.0, 0.0, 0.0]
lidar_mount_correction_rpy_deg_real: [0.0, 0.0, 0.0]
```

2-4의 4방향 데이터 수집 (0°, 90°, 180°, 270°):
- 각 방향에서 정지 상태로 같은 시간(예: 10초) 캡처
- 모든 방향 데이터를 한 파일로 누적 (총 4회 반복)

분석:
```bash
python3 surface_profiling/analyze_z_bias.py \
  ~/dae_floor_maps/analytics/pointclouds/frames/frames_<timestamp>.npz
```

출력 예:
```
pitch(deg, +=전방이 높음): mean 0.47  std(프레임 간) 0.03
roll (deg, +=좌측이 높음): mean -0.31  std(프레임 간) 0.02
제안값: lidar_mount_correction_rpy_deg = [-0.31, 0.47, 0.0]
```

**단계 2: 보정값 적용**

```yaml
# config/params.yaml 수정 (Simulation인 경우 _sim, Real인 경우 _real)
lidar_mount_correction_rpy_deg_real: [-0.31, 0.47, 0.0]  # [roll, pitch, yaw]
```

`colcon build --symlink-install`으로 설치했다면 재빌드 **불필요**. 노드만 다시 실행.

**단계 3: 재측정 (보정 활성)**

`surface_profiling` 노드를 다시 시작하고, 단계 1과 동일한 방식으로 4방향 데이터 수집.

분석:
```bash
python3 surface_profiling/analyze_z_bias.py \
  ~/dae_floor_maps/analytics/pointclouds/frames/frames_<timestamp_new>.npz
```

기대 출력:
```
pitch(deg, +=전방이 높음): mean -0.02  std(프레임 간) 0.02  ← 거의 0에 수렴
roll (deg, +=좌측이 높음): mean 0.01  std(프레임 간) 0.02   ← 거의 0에 수렴
제안값: lidar_mount_correction_rpy_deg = [0.01, -0.02, 0.0]  ← 이미 0에 가까움
```

**단계 4 (선택): 미세 조정**

재측정 후에도 pitch/roll이 0.1도 이상이면, 제안값의 잔여분을 더할 수 있음:

```yaml
# 현재: [-0.31, 0.47, 0.0]
# 단계 3 제안값: [0.01, -0.02, 0.0]
# 최종값: [-0.31 + 0.01, 0.47 - 0.02, 0.0] = [-0.30, 0.45, 0.0]
lidar_mount_correction_rpy_deg_real: [-0.30, 0.45, 0.0]
```

**부호 규약**

`pitch`와 `roll`의 부호:
- `pitch` (+): 센서의 전방부(X축)이 위쪽(+Z). URDF joint rpy에서 Y축 회전과 같은 방향.
- `roll` (+): 센서의 왼쪽(−Y축)이 위쪽(+Z). URDF joint rpy에서 X축 회전과 같은 방향.
- 분석이 출력하는 제안값을 그대로 쓰면 부호 변환 불필요.

### 4-4. 판정 기준 (목표치)

| 지표 | 기준 | 해석 |
|------|------|------|
| \|pitch mean\|, \|roll mean\| | 0.05° 이하 | 목표 결함 높이(1cm) / 최대 거리(3.5m) ≈ 0.16°의 1/3 |
| `pitch/roll std(프레임 간)` | 평균의 1/5 이하 | 고정 오프셋(변하지 않음) 확인 |
| `z mean` | ±0.2cm 이하 | 바닥 높이 편향 최소화 |

목표치에 미달하면 다시 측정하거나 환경 조건(바닥 평탄성, 센서 고정도) 확인 필요.

## 5. 한계

- 바닥이 평평하다는 가정으로 평면을 맞추므로, **평탄도를 재려는 바로 그 바닥**에서 보정하면 순환임. 4방향 측정 평균은, 바닥 경사가 로봇과 함께 돌지 않는다는 점을 이용해, 이 편향을 줄이는 방법임.
- 이 보정은 **고정** 오프셋만 다룸. 결함 위를 지날 때 차체가 기우는 것 같은 동적 기울기는 보정되지 않음(현재 TF는 평면이라 roll/pitch가 반영되지 않음).
- 사용 거리 범위(1.2~3.5m)와 프레임당 최소 점 수(300)는 기본값이며, 다른 센서/높이에서는 `--r-min`, `--r-max`, `--min-points`를 조정함.
