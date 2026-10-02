# IMU 차체 기울기 보정 · 주행 검증 가이드

로봇의 IMU(OpenCR)가 알려주는 roll/pitch를 TF에 넣어 쓰기 전에, (1) **정지 상태에서 고정 바이어스를 구하고**, (2) **주행 중에 IMU가 실제 차체 기울기를 따라가는지 라이다 바닥 평면과 비교해 검증**하는 절차임. 실물 로봇 기준이며, 라이다 장착 자세 보정은 [lidar_calibration.md](lidar_calibration.md)에서 다룸.

## 1. 무엇을 하는가

`imu_tilt_broadcaster`(`real_bringup.launch.py`, `sim_env.launch.py`가 실행)가 `/imu`의 roll/pitch에서 바이어스를 뺀 값을 `base_footprint → base_link` TF로 발행함. `base_link` 아래의 `velodyne_link`는 고정 조인트라, 이 기울기가 점군 좌표 변환에 그대로 반영됨. yaw는 odom/AMCL이 담당하므로 넣지 않음.

```yaml
# config/params.yaml - mission_execution
imu_mount_correction_rpy_deg: [0.0, 0.0]   # [roll, pitch] deg. /imu에서 이 값을 뺀 뒤 TF에 싣음
```

| 단계 | 구하는 것 | 방법 | 이유 |
|---|---|---|---|
| 3절 정지 | 고정 바이어스(장착 오차, 영점) | 같은 자리에서 90° 간격 4헤딩 정지 측정 후 평균 | 바이어스와 바닥 기울기는 한 방향 측정으로 분리 안 됨. 헤딩을 돌리면 바닥 성분이 평균에서 상쇄됨 |
| 4절 주행 | 가감속·진동·지연 같은 동적 오차 | 주행 중 IMU와 라이다 평면 적합 tilt를 같은 시각축에서 비교 | 정지에서는 드러나지 않으므로 보정이 아니라 **검증** 대상임 |

> **순서**: IMU(이 문서) → 라이다 보정. 라이다 보정값은 IMU TF가 켜진 상태에서 남는 오차이므로, IMU 설정이 바뀌면 라이다 보정을 다시 해야 함.
>
> **주의**: `imu_mount_correction_rpy_deg`는 Simulation/Real 구분 키가 없음. 실물 값을 넣은 채 Simulation을 실행하면 바이어스가 없는 시뮬레이션 IMU에서 그 값이 빠져 TF가 기울어짐. Simulation을 돌릴 때는 `[0.0, 0.0]`으로 되돌림.

스크립트: `surface_profiling/test/check_imu_dynamics.py` (아래 명령은 패키지 루트 기준, **Laptop**에서 실행). 서브커맨드는 `static`(정지 기록), `record`(주행 기록), `analyze`(비교).

## 2. 준비

- Jetson과 Laptop이 같은 `ROS_DOMAIN_ID`로 서로의 토픽을 볼 수 있어야 함. Laptop에서 `ros2 topic hz /imu`가 약 20 Hz로 나오면 됨.
- 4절 주행 검증은 Jetson과 Laptop의 **시계가 동기화**돼 있어야 정확함(IMU 시각은 Jetson, 프레임 시각은 Laptop 기준). 어긋나면 분석 결과의 지연 값에 섞여 나옴.
- 라이다 쪽 준비(Velodyne 드라이버, AMCL 초기 위치)는 [lidar_calibration.md](lidar_calibration.md) 2-1절과 같음.

## 3. 정지 4헤딩 바이어스 측정

**필요한 터미널은 2개**임. Nav2, 라이다, 측정 노드는 필요 없음.

| 순서 | 머신 | 명령 / 동작 |
|---|---|---|
| 1 | **Jetson** J1 | `ros2 launch dae_coverage_floor_flatness real_bringup.launch.py` |
| 2 | **Laptop** L1 | 로봇을 평평한 바닥에 두고 **완전히 정지**시킨 뒤: `python3 surface_profiling/test/check_imu_dynamics.py static --heading 0 --duration 30` |
| 3 | **Jetson** J2 | 로봇을 제자리에서 90° 회전: `python3 surface_profiling/test/check_imu_dynamics.py rotate --deg 90` (`/odom` yaw를 보며 목표 각도 근처에서 감속하고 **스스로 정지**함. 앞뒤 공간 확인) |
| 4 | | 정지 메시지가 나오면 **약 10초 대기** (직접 정지 명령은 필요 없음) |
| 5 | **Laptop** L1 | `python3 surface_profiling/test/check_imu_dynamics.py static --heading 90 --duration 30` |
| 6 | | 3~5를 반복해 `--heading 180`, `--heading 270` 측정 |
| 7 | **Laptop** L1 | `python3 surface_profiling/test/check_imu_dynamics.py static --report` |

- `rotate`는 odom 기준 회전량을 출력함(정지 후 관성분 포함, 목표 ±수 도 이내면 충분함). 시계 방향은 `--deg -90`. Jetson에 이 패키지 소스가 없으면 Laptop에서 실행해도 되며, 같은 `ROS_DOMAIN_ID`에서 `/cmd_vel`을 발행함.
- 모든 헤딩을 **같은 자리**(바퀴 중심 고정)에서 측정함. 스크립트는 헤딩 시작 후 5초(`--settle`)를 추가로 버려 회전 직후 필터 과도 응답을 제외함.
- 날짜나 바닥이 바뀌면 `--tag 이름`으로 세션을 분리함.

**결과 읽기** (4개가 모이면 `static`/`--report`가 출력. 아래 숫자는 형식을 보이기 위한 예시임)

```
[*] 4헤딩 평균 -> imu_mount_correction_rpy_deg: [1.200, -0.350]
    헤딩 간 표준편차 roll 0.052  pitch 0.031 deg
```

- 출력된 `[roll, pitch]`를 `config/params.yaml`의 `imu_mount_correction_rpy_deg`에 넣음(`--symlink-install`이면 재빌드 불필요, 노드 재시작).
- 헤딩 간 표준편차가 **0.5°를 넘으면** 경고가 나옴. 값이 몸체에 고정된 바이어스가 아니라 바닥 기울기, 필터 드리프트, 주변 자기장 등이 섞였다는 뜻이므로, 위치를 바꾸거나 시간을 두고 반복해 재현되는지 먼저 확인함. 재현되지 않으면 IMU TF를 쓰지 않는 쪽이 안전함.
- roll/pitch가 한 번에 수 도(°) 단위로 달라지는 바이어스는 정상적인 마운트 오차 범위를 넘으므로 원인 확인이 우선임.

## 4. 주행 중 IMU 검증

같은 주행 패턴을 **IMU TF를 끈 상태**와 **켠 상태**로 각각 한 번씩 수행함.

| 실행 | `--imu-tf` | 보려는 것 |
|---|---|---|
| A. TF off | `off` | IMU 변화가 라이다가 본 실제 기울기와 일치하는가 |
| B. TF on | `on` | IMU를 TF로 넣었을 때 라이다 점군의 기울기 잔차가 줄어드는가 |

### 4-1. 터미널 구성

| 이름 | 머신 | 역할 |
|---|---|---|
| J1 | **Jetson** | 로봇 드라이버 + TF (`real_bringup.launch.py`) |
| J2 | **Jetson** | Nav2 + AMCL |
| J3 | **Jetson** | 속도 명령 발행 |
| J4 | **Jetson** | IMU TF 전환 (A에서는 정지 TF 발행, B에서는 `imu_tilt_broadcaster`) |
| L1 | **Laptop** | Velodyne 드라이버 |
| L2 | **Laptop** | 측정 노드 (`surface_profiling.launch.py`) |
| L3 | **Laptop** | 캡처 서비스 호출 / 분석 |
| L4 | **Laptop** | IMU 기록 (`record`) |

### 4-2. 실행 A: IMU TF off

`base_footprint`는 URDF에 없고 이 노드가 발행하므로, 노드를 끄면 TF가 끊김. 같은 변환을 회전 0으로 대신 발행함.

| 순서 | 머신 | 명령 / 동작 |
|---|---|---|
| 1 | **Jetson** J1 | `ros2 launch dae_coverage_floor_flatness real_bringup.launch.py` |
| 2 | **Jetson** J4 | `pkill -f imu_tilt_broadcaster` |
| 3 | **Jetson** J4 | `ros2 run tf2_ros static_transform_publisher --z 0.01 --frame-id base_footprint --child-frame-id base_link` (**실행 중 유지**) |
| 4 | **Jetson** J3 | 확인: `ros2 run tf2_ros tf2_echo base_footprint base_link` → rotation이 0이면 정상, Ctrl-C |
| 5 | **Jetson** J2 | `ros2 launch dae_coverage_floor_flatness tb3_waffle_nav2.launch.py use_sim_time:=false` |
| 6 | **Laptop** L1 | `ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py` |
| 7 | **Laptop** L2 | `ros2 launch dae_coverage_floor_flatness surface_profiling.launch.py is_sim:=false` |
| 8 | | RViz에서 AMCL 초기 위치를 지정해 수렴시킴 |
| 9 | **Laptop** L4 | `python3 surface_profiling/test/check_imu_dynamics.py record --label tf_off` (Ctrl-C까지 기록) |
| 10 | **Laptop** L3 | 캡처 열기: `ros2 service call /surface_profiling/start_waypoint_capture std_srvs/srv/Trigger` (가감속 구간도 담아야 하므로 **이동 전에** 열고, 라이다 문서와 달리 1초 대기하지 않음) |
| 11 | | 로봇 정지 상태로 **10초 대기** (정지 기준선) |
| 12 | **Jetson** J3 | 전진: `ros2 topic pub --times 375 -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.16}}"` |
| 13 | **Jetson** J3 | 정지: `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}}"` 후 **5초 대기** |
| 14 | **Jetson** J3 | 후진: `ros2 topic pub --times 375 -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.16}}"` → 정지(13과 같은 명령) → 5초 대기 |
| 15 | | 12~14를 한 번 더 반복(총 2세트) |
| 16 | | 마지막 정지 후 **10초 대기** |
| 17 | **Laptop** L3 | `ros2 service call /surface_profiling/stop_waypoint_capture std_srvs/srv/Trigger` |
| 18 | **Laptop** L3 | `ros2 service call /surface_profiling/stop_collection_success std_srvs/srv/Trigger` (프레임 기록 `frames_<timestamp>.npz` 저장) |
| 19 | **Laptop** L4 | Ctrl-C → `imu_drive_tf_off_<timestamp>.npz` 저장 |

속도 0.16 m/s는 `mission_execution.coverage_speed_limit_mps`와 같은 값으로, 측정 조건을 맞추기 위함임. 앞 공간을 확인한 뒤 실행함(개루프 명령).

### 4-3. 실행 B: IMU TF on

3절에서 구한 바이어스를 `params.yaml`에 넣은 뒤 진행함.

| 순서 | 머신 | 명령 / 동작 |
|---|---|---|
| 1 | **Jetson** J4 | 정지 TF 발행(A의 3단계)을 Ctrl-C로 끔 |
| 2 | **Jetson** J4 | `ros2 run dae_coverage_floor_flatness imu_tilt_broadcaster` |
| 3 | **Laptop** L2 | 측정 노드를 Ctrl-C 후 다시 실행(`surface_profiling.launch.py is_sim:=false`). 필터 상태가 이전 실행에 남지 않게 함 |
| 4 | | A의 8~18단계를 같은 방식으로 반복하되, 9단계는 `record --label tf_on` |

### 4-4. 분석 실행

**Laptop** L3에서 실행함. `--imu`는 `~/dae_floor_maps/analytics/imu_check/`의 파일 이름, `--frames`는 `~/dae_floor_maps/analytics/pointclouds/frames/`의 파일 이름만 줘도 됨.

```bash
python3 surface_profiling/test/check_imu_dynamics.py analyze \
  --imu imu_drive_tf_off_<timestamp>.npz --frames frames_<timestamp>.npz \
  --map ~/dae_floor_maps/maps/grid/map_from_dae.yaml --imu-tf off

python3 surface_profiling/test/check_imu_dynamics.py analyze \
  --imu imu_drive_tf_on_<timestamp>.npz --frames frames_<timestamp>.npz \
  --map ~/dae_floor_maps/maps/grid/map_from_dae.yaml --imu-tf on

# 그래프 보기
xdg-open ~/dae_floor_maps/analytics/imu_check/imu_drive_tf_off_<timestamp>_vs_lidar.png
```

방금 수집한 프레임 파일은 `ls -t ~/dae_floor_maps/analytics/pointclouds/frames | head -1`로 찾음. `--map`을 주면 벽에서 0.4m 이내 점을 제외함(권장).

## 5. 결과 읽기

```
===== pitch =====
  상관(지연 0): +0.74   최적 지연 +0.20s 에서 +0.98   기울기 L~I: +1.00
  IMU 변동 std 0.267 deg   라이다 적합 std 0.289 deg   정지 프레임의 라이다 적합 잡음 std 0.291 deg
  구간별 평균 변화량(정지 구간 평균 대비, deg):   IMU / 라이다
    accel   +0.043 / +0.307
    ...
```

(위 숫자는 출력 형식을 보이기 위한 예시임)

| 출력 | 의미 | 판단 |
|---|---|---|
| 구간별 개수 | 정지/가속/정속/제동에 속한 IMU·프레임 수 | `accel`/`brake` 프레임이 거의 0이면 가감속 검증이 안 된 것임. `--acc-thr`를 낮추거나 속도 명령을 더 급하게 줌 |
| 상관, 최적 지연 | IMU tilt와 라이다 적합 tilt의 상관. 지연은 -1~+1초를 훑어 가장 잘 맞는 값 | 양수는 IMU가 라이다보다 늦다는 뜻. ±0.3초를 넘으면 IMU 지연과 시계 오프셋이 분리되지 않음 |
| 기울기 `L~I` | 라이다 tilt를 IMU tilt로 회귀한 기울기 | 아래 표 참고. 부호는 좌표 관례에 따라 반대일 수 있어 절댓값으로 봄 |
| 정지 프레임 라이다 적합 잡음 | 정지 중 프레임별 평면 적합의 흩어짐 | 판별 가능한 최소 tilt 크기의 기준선임 |
| 구간별 평균 변화량 | 가속·정속·제동 때 정지 구간 대비 tilt 변화(IMU / 라이다) | TF off에서 IMU만 변하고 라이다는 안 변하면 IMU 아티팩트(가감속의 선가속도가 자세 추정에 섞임)임 |
| 해석 | 위 기준을 자동 판정한 문장 | |

| `--imu-tf` | 결과 | 의미 |
|---|---|---|
| `off` | 상관 높음, \|기울기\|≈1 | IMU가 실제 차체 기울기를 따라감 → TF 주입 가능 |
| `off` | 상관 낮음, IMU 변동 < 정지 잡음 | **판별 불가**(동적 틸트가 검출 한계 아래). "문제없음"이 아님 |
| `off` | 상관 낮음, IMU 변동 > 정지 잡음 | IMU 아티팩트이거나 라이다가 못 잡는 크기 |
| `on` | \|기울기\|≈0 | 잔차가 IMU와 무상관 → 보상이 동작함 |
| `on` | \|기울기\|≈2 | 부호 반대 또는 과보상 의심 → TF를 켠 쪽이 더 나쁨 |

보조로 A와 B의 `라이다 적합 std`를 비교함. B가 더 작으면 IMU TF가 점군 기울기를 줄이는 것임.

## 6. 한계

- 라이다 프레임당 평면 적합에는 정지 중에도 수 십분의 1도(°)의 잡음이 있어, 그보다 작은 동적 틸트는 이 방법으로 검출되지 않음. 이때 결과는 "판별 불가"로 읽어야 함.
- 가속·제동 구간은 컨트롤러 가감속이 작으면 판정 임계(`--acc-thr`, 기본 0.03 m/s²)에 걸리는 프레임이 적을 수 있음.
- 같은 주행에서 바닥 자체의 국소 경사가 달라지면 IMU와 라이다에 똑같이 나타나므로 상관에는 영향이 없으나, 정지 기준선과 주행 구간의 평균 비교에는 섞임.
- 이 절차의 검증 대상은 roll/pitch뿐이며, 높이(z) 오프셋은 다루지 않음.
