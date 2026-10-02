# 실기체 스택 실행기 (`stack.py`)

로봇(Jetson)과 라이다 노트북에 걸친 실행 순서를 **노트북 터미널 하나**에서 올리고, 단계마다 게이트(자동 확인)를 통과해야 다음 단계로 넘어가는 도구임. 터미널 로그를 눈으로 보고 다음 명령을 치던 절차를 수치 조건으로 바꾼 것이며, 실기체 전용임(Simulation은 [README](../../README.kr.md)의 실행 순서를 따름).

## 1. 무엇을 올리는가

| 단계 | 실행 위치 | 실행하는 것 | 게이트(통과 조건) |
|---|---|---|---|
| `preflight` | 양쪽 | 없음(점검만) | 이전 실행의 잔여 프로세스가 없음 |
| `bringup` | Jetson | `real_bringup.launch.py use_imu_tilt:=<on/off>` | `/scan` 3-20 Hz, `/imu` 5-500 Hz, `/odom` 10-100 Hz, 세 토픽의 발행자가 각각 1개, TF `odom→base_footprint`와 `base_footprint→base_link`, IMU TF 모드가 요청과 일치 |
| `nav2` | Jetson | `tb3_waffle_nav2.launch.py use_rviz:=false` | `/map_server`, `/amcl` lifecycle이 `active` |
| `velodyne` | 노트북 | `velodyne-all-nodes-VLP16-launch.py` | `/velodyne_points` 8-12 Hz(600 rpm = 10 Hz), 발행자 1개 |
| `pose` | 노트북 | `/initialpose` 발행(`--init`을 줬을 때) | TF `map→odom` 생성, `/amcl_pose` 수신 |
| `profiler` | 노트북 | `surface_profiling.launch.py is_sim:=false` | TF `map→velodyne_link` |

미션 실행(`mission_execution.launch.py`)은 포함하지 않음. 시작 지점 정렬과 시작 시점은 사람이 정하므로 [README](../../README.kr.md)의 마지막 단계를 직접 실행함.

Jetson 프로세스는 `bash -ic`(`~/.bashrc`의 `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION` 등이 적용되는 셸)로 띄우고, 이 스크립트가 끝나도 계속 실행됨. 로그는 각 기기의 `~/stack_logs/<단계>.log`에 남음.

## 2. 사전 준비 (최초 1회)

- **Jetson**: 이 패키지를 최신으로 받아 빌드함(`git pull` 후 `colcon build --symlink-install`). `imu_tilt_broadcaster`가 Jetson에서 실행되므로 `config/params.yaml`의 IMU 바이어스도 **Jetson의 파일**이 쓰임. 지도(`~/dae_floor_maps`)도 Jetson에 있어야 함.
- **노트북**: ROS 환경과 이 패키지를 source한 터미널에서 실행함(게이트가 DDS로 직접 확인하므로 두 기기의 `ROS_DOMAIN_ID`가 같아야 함).
- 두 기기의 시계를 chrony 등으로 동기화함(점군 변환이 스탬프 시각의 TF를 쓰기 때문임).
- 접속 정보를 환경변수로 줌(비밀번호를 명령줄에 남기지 않기 위함).

```bash
export ROBOT_HOST=<Jetson IP>
export SSH_PASSWORD=<비밀번호>      # 사용자는 기본 waffle, 다르면 --user
```

저장소 폴더 이름이 `dae-coverage-floor-flatness`가 아니거나 워크스페이스가 `~/ros2_ws`가 아니면 `--ws <워크스페이스 경로>`, `--repo <저장소 경로>`를 줌. Jetson의 경로가 노트북과 다르면 `--jetson-ws`, `--jetson-repo`로 따로 지정함. 워크스페이스에 `install/setup.bash`가 없으면 실행 전에 바로 중단하고 알려줌.

## 3. 명령

모든 명령은 패키지 루트에서 **노트북**에 입력함.

```bash
# 전체 올리기: IMU TF 끔, 초기 위치를 인자로 지정
python3 scripts/stack.py up --imu-tf off --init 1.20 0.80 0.0

# 한 단계씩 확인하며 올리기(단계가 끝날 때마다 Enter 대기)
python3 scripts/stack.py up --imu-tf off --step

# 특정 단계까지만(예: 로봇 bringup만)
python3 scripts/stack.py up --until bringup

# 상태 점검(올라와 있는 스택을 건드리지 않고 읽기만 함)
python3 scripts/stack.py check

# 양쪽 모두 종료
python3 scripts/stack.py down
```

| 옵션 | 기본값 | 설명 |
|---|---|---|
| `--imu-tf on\|off` | `off` | `imu_tilt_broadcaster`의 모드를 launch 인자로 고정함. 모드는 측정 기록에도 저장됨 |
| `--init X Y YAW` | 없음 | map 프레임 좌표(m)와 yaw(rad)로 AMCL 초기 위치를 지정함. 없으면 RViz에서 직접 지정함(4절) |
| `--map <yaml>` | Nav2 기본값 | Nav2에 넘길 지도 |
| `--until <단계>` | `profiler` | 이 단계까지만 실행함 |
| `--step` | 꺼짐 | 단계마다 Enter로 확인한 뒤 진행함 |
| `--clean` | 꺼짐 | 잔여 프로세스가 있으면 먼저 종료함. 없으면 중단하고 알려줌 |
| `--pose-wait <초>` | 180 | RViz 수동 초기 위치를 기다리는 시간 |
| `--jetson-ws <경로>` | `--ws`와 같음 | Jetson의 워크스페이스 경로 |
| `--local` | 꺼짐 | Jetson 쪽 명령도 이 컴퓨터에서 실행함(점검용) |

게이트에 실패하면 해당 단계 로그의 마지막 15줄을 출력하고 멈춤. 그 뒤에는 `check`로 상태를 보고, 원인을 고친 다음 `down`(또는 `--clean`)부터 다시 시작함.

## 4. AMCL 초기 위치

- **`--init` 사용(권장)**: 스크립트가 `/initialpose`를 map 프레임으로 발행하고 `map→odom`이 생길 때까지 확인함. 로봇을 놓은 자리의 map 좌표와 heading을 줌. heading 오차는 `map→odom`에 그대로 남아 경로 전체가 기울어 보이므로 정확히 맞춤.
- **RViz에서 지정**: `--init` 없이 실행하면 `pose` 단계에서 대기함. 노트북에서 RViz를 열고 **Global Options의 Fixed Frame을 `map`으로 바꾼 뒤** 2D Pose Estimate를 지정함. 발행되는 `/initialpose`의 `frame_id`가 Fixed Frame 값이라, `map`이 아니면 AMCL이 받지 않아 아무 일도 일어나지 않음.

```bash
rviz2 -d $(ros2 pkg prefix dae-coverage-floor-flatness)/share/dae-coverage-floor-flatness/rviz/tb3_navigation2.rviz
```

## 5. 상태 점검 `check`

실행 중인 스택에서 다음을 한 번에 출력함. 초기 위치를 줬는데도 `map→odom`이 안 생길 때 원인을 좁히는 용도임.

| 출력 | 보는 것 |
|---|---|
| 토픽 수신율과 발행자 수 | `/scan`, `/imu`, `/odom`, `/velodyne_points`. 발행자가 2개 이상이면 드라이버가 중복 실행 중임 |
| TF 존재 여부 | `odom→base_footprint`, `base_footprint→base_link`, `odom→base_scan`, `map→odom`, `map→velodyne_link` |
| IMU TF | 현재 모드, 바이어스와 그 값이 들어 있는 키 |
| Nav2 | `/map_server`, `/amcl`의 lifecycle 상태, AMCL의 `set_initial_pose`와 `initial_pose.*` 값, `/amcl_pose` 수신 여부 |
| AMCL 로그 | `stack.py up`으로 띄운 경우, 초기 위치 설정과 TF 변환 실패 관련 줄 |

`--try-pose X Y YAW`를 주면 map 프레임 `/initialpose`를 발행해 `map→odom`이 생기는지까지 시험함.

## 6. 문제가 생겼을 때

| 증상 | 조치 |
|---|---|
| `이미 실행 중인 프로세스가 있음` | `stack.py down` 후 다시 실행하거나 `--clean`을 줌 |
| `NO_SCRIPT ...` | 해당 기기에서 `git pull`을 하지 않았거나 `--repo` 경로가 다름 |
| 발행자 수 게이트 실패 | 드라이버가 이미 떠 있음. `down` 후 다시 실행함 |
| `map→odom` 게이트 실패 | `check`로 AMCL 상태를 보고, RViz로 지정했다면 Fixed Frame이 `map`인지 확인함 |
| IMU TF 모드 게이트 실패 | Jetson이 이전 버전이거나 `bringup`이 다른 모드로 이미 떠 있음. `down` 후 다시 실행함 |
| 접속 실패 | `ROBOT_HOST`, `SSH_PASSWORD`, Jetson의 SSH 서버를 확인함 |

종료 때 `down`은 각 기기에서 `scripts/stop_all.sh`를 실행함. 이 스크립트는 단독으로도 쓸 수 있으며([installation.md](installation.md#3-셸-환경-설정) 참고), Gazebo와 RViz까지 정리함.

## 7. 한계

- 게이트의 수신율 범위는 기본값이며, 센서 기종이나 설정이 다르면 `scripts/stack.py`의 `rate_gate` 호출부 값을 조정함.
- SSH는 비밀번호 인증만 지원함(`auto_calibration_drive.py`와 같은 방식).
- 실행 중인 프로세스를 이름으로 식별하므로, 같은 기기에서 이 패키지와 무관한 ROS 프로세스를 돌리고 있으면 `down`이 함께 종료함.
