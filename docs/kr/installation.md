# 설치 가이드

[English](../en/installation.md) · [한국어](installation.md) · [← README](../../README.kr.md)

이 문서는 시스템이 돌아갈 수 있는 모든 기기를 다룸. **대부분의 경우 전부 필요하지는 않으니** [어떤 단계가 필요한가](#어떤-단계가-필요한가)부터 볼 것.

**Ubuntu 22.04 + ROS 2 Humble + Python 3.10** 기준으로 검증함.

---

## 어떤 단계가 필요한가

시스템은 역할 3개로 나뉨. 물리적으로는 한 기기가 여러 역할을 겸할 수 있음.

| 역할 | 하는 일 | 필요한 단계 |
|---|---|---|
| **planning** (워크스테이션/노트북, x86_64) | `.dae` 모델을 `final_path.json`으로 바꿈, 오프라인 | 0–7 |
| **주행** (로봇의 Jetson Orin Nano, arm64) | Nav2와 `mission_executor` 실행 | 0–4, 9, 10 |
| **측정** (VLP-16이 직결된 노트북, x86_64) | `surface_profiler` 실행 | 0–4, 7, 8, 10 |

**시뮬레이션만, 기기 1대로 하려면?** 그 기기에 0–8단계만 하면 되고 9·10단계는 통째로 건너뛰어도 됨.

---

## 0. 사전 준비

- Ubuntu 22.04
- ROS 2 Humble ([설치 문서](https://docs.ros.org/en/humble/Installation.html)), `source /opt/ros/humble/setup.bash`가 정상 동작할 것
- `colcon`, `rosdep` 초기화 (`sudo rosdep init && rosdep update`)

이 패키지가 `rosdep`으로 끌어오는 ROS 패키지: `rclpy`, `nav2_simple_commander`, `nav2_msgs`, `gazebo_msgs`, `sensor_msgs_py`, `tf_transformations`, `laser_filters`.

특정 launch 파일이 추가로 요구하는 ROS 패키지:

| 패키지 | 사용하는 launch | 필요한 경우 |
|---|---|---|
| `nav2_bringup` | `tb3_waffle_nav2.launch.py` | 주행, 시뮬레이션 |
| `gazebo_ros`, `turtlebot3_description` | `sim_env.launch.py` | 시뮬레이션 |
| `turtlebot3_bringup`, `turtlebot3_node`, 그리고 기종에 맞는 LDS 드라이버 (`hls_lfcd_lds_driver` / `ld08_driver` / `coin_d4_driver`) | `real_bringup.launch.py` | 실기체만 |
| `velodyne` (드라이버) | VLP-16 데이터 수집 | 실기체 측정 |

---

## 1. 외부 데이터 저장소

입력과 출력은 전부 저장소 **바깥**의 `~/dae_floor_maps`에 둠. 어느 단계든 실행하는 기기라면 모두 만들어야 함:

```bash
mkdir -p ~/dae_floor_maps/{assets,maps/{debug_image,grid,topology},\
analytics/{metrics,paths,pointclouds,logs},\
visualization/{mission_generation/{environment_modeling,mission_planning},mission_execution,surface_profiling}}
```

3D 모델을 `~/dae_floor_maps/assets/`에 넣고(예: `Apt.dae`), `config/params.yaml`의 `environment_modeling.dae_file`을 그 파일명으로 맞출 것.

> 경로 자체는 `config/params.yaml`의 `global.workspace_root`로 바꿀 수 있으나, 모든 기기가 같은 값을 써야 함.

---

## 2. 워크스페이스와 빌드

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/ChanggonSong/dae-coverage-floor-flatness.git

cd ~/ros2_ws
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
pip install --user "setuptools<80,>=30.3.0"

colcon build --symlink-install
source install/setup.bash
```

> **빌드할 때마다 다시 source할 것.** 새 터미널을 열거나 `source ~/ros2_ws/install/setup.bash`를 실행하면 됨.
>
> **`--symlink-install`은 반드시 붙일 것.** 이 옵션으로 빌드해야 `install/` 아래의 설정·launch·world·모델 파일이 소스를 그대로 가리키는 심볼릭 링크가 됨. 덕분에 `params.yaml`이나 behavior tree를 고치면 재빌드 없이 바로 반영됨.
>
> 옵션 없이 빌드하면 전부 복사본이 되어, 값 하나를 고칠 때마다 `colcon build`를 다시 돌려야 함. 파일을 새로 추가하거나 이름을 바꿨을 때는 옵션과 무관하게 한 번 빌드해야 함. 설치 목록 자체가 달라지기 때문임.

---

## 3. 셸 환경 설정

`~/.bashrc` 맨 아래에 추가한 뒤 `source ~/.bashrc`:

```bash
# --- ROS 2 Humble & TurtleBot3 ---
source /opt/ros/humble/setup.bash
# 워크스페이스는 필요할 때 수동으로:
# source ~/ros2_ws/install/setup.bash

export TURTLEBOT3_MODEL=waffle
export LDS_MODEL=LDS-02          # 로봇의 2D 라이다 기종에 맞출 것
export ROS_DOMAIN_ID=30          # 모든 기기에서 동일해야 함
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Gazebo (시뮬레이션에서만)
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/opt/ros/humble/lib

# OR-Tools는 /usr/local/lib에 설치됨
export LD_LIBRARY_PATH=/usr/local/lib:/opt/ros/humble/lib:$LD_LIBRARY_PATH
```

`TURTLEBOT3_MODEL`과 `LDS_MODEL`은 launch 파일을 **import하는 시점**에 읽히므로, launch 인자로 넘기는 게 아니라 `ros2 launch` 실행 전에 export되어 있어야 함.

`GAZEBO_MODEL_PATH`는 `sim_env.launch.py`가 이 패키지의 `models/` 디렉토리를 기준으로 직접 설정하므로 따로 export할 필요 없음.

필수는 아니지만 초기화 alias를 권장함. 이전 실행에서 남은 Gazebo/Nav2 프로세스가 이상 동작의 가장 흔한 원인임:

```bash
alias rrr='ros2 daemon stop; \
killall -9 gzserver gzclient; \
pkill -9 -f ros2; pkill -9 -f fastdds; \
pkill -9 -f robot_state_publisher; pkill -9 -f ekf; \
pkill -9 -f nav2; pkill -9 -f rviz; pkill -9 -f spawn_entity.py'
```

---

## 4. 시스템 패키지 (모든 기기)

```bash
sudo apt update
sudo apt install build-essential libgdal-dev libgeos-dev libeigen3-dev libboost-dev \
     libtbb-dev libtinyxml2-dev nlohmann-json3-dev libpython3-dev gnuplot
sudo apt install ros-humble-rmw-cyclonedds-cpp

python3 -m pip install --upgrade pip wheel
pip install --user "setuptools<80,>=30.3.0"
```

> **OpenCV:** `package.xml`에는 apt 패키지 `python3-opencv`가 선언되어 있지만, 실제로는 유저 site-packages의 pip 휠이 항상 우선순위를 가져가 그쪽이 import됨. `requirements-*.txt`가 `opencv-python-headless`를 고정하는 이유가 이것임. 7단계에서 설치하고, apt 쪽은 그냥 안 쓰이게 두면 됨.

---

## 5. Fields2Cover — 커밋 고정 소스 빌드

*사전 planning 역할에서만 필요함.*

**Fields2Cover를 PyPI로 설치하지 말 것.** 공개된 릴리스는 이 프로젝트가 검증에 사용한 버전과 스와스 형상이 달라서, 노드 방문 순서까지 바뀌고 기기 간 결과 비교가 불가능해짐. 고정 커밋에서 소스 빌드할 것:

```bash
cd ~
git clone https://github.com/Fields2Cover/Fields2Cover.git
cd ~/Fields2Cover
git checkout 85d6cf7      # 더 최신 태그/커밋으로 임의로 바꾸지 말 것
mkdir -p build && cd build
cmake .. -DBUILD_PYTHON=ON
make -j$(nproc)
sudo make install
sudo ldconfig
```

확인:

```bash
python3 -c "import fields2cover as f2c; print(f2c.DECOMP_Boustrophedon)"
```

> 파이썬 바인딩은 C++ API의 일부만 노출함. `help(f2c.X)`로 시그니처가 안 나오면 C++ 헤더나 `~/Fields2Cover/tutorials/python/`를 찾아볼 것.

로봇에서는 Fields2Cover가 런타임에 실제로 쓰이지 않음. 경로는 planning 기기에서 미리 계산해 파일로 넘겨받기 때문임. 그래도 설치해두는 쪽이 안전함. `mission_planner.py`를 import하는 코드 경로가 있을 때 import 에러를 막아줌.

---

## 6. OR-Tools 공유 라이브러리

TSP solver가 OR-Tools의 C++ 공유 라이브러리를 필요로 함. **아카이브는 아키텍처별로 다르니** 맞는 쪽을 쓸 것.

**x86_64 (노트북 / 워크스테이션):**

```bash
wget https://github.com/google/or-tools/releases/download/v9.9/or-tools_amd64_ubuntu-22.04_cpp_v9.9.3963.tar.gz \
     -O ~/or-tools_v9.9.3963.tar.gz
tar tzf ~/or-tools_v9.9.3963.tar.gz | head -3     # 실제 최상위 폴더명 확인
tar xzf ~/or-tools_v9.9.3963.tar.gz -C ~
sudo cp -P ~/or-tools_x86_64_Ubuntu-22.04_cpp_v9.9.3963/lib/*.so* /usr/local/lib/
sudo ldconfig
```

**arm64 (Jetson Orin Nano):**

```bash
wget https://github.com/google/or-tools/releases/download/v9.9/or-tools_arm64_debian-11_cpp_v9.9.3963.tar.gz \
     -O ~/or-tools_v9.9.3963.tar.gz
tar tzf ~/or-tools_v9.9.3963.tar.gz | head -3     # 실제 최상위 폴더명 확인
tar xzf ~/or-tools_v9.9.3963.tar.gz -C ~
sudo cp -P ~/or-tools_aarch64_Debian-11_cpp_v9.9.3963/lib/*.so* /usr/local/lib/
sudo ldconfig
```

압축을 푼 폴더명이 아카이브 이름과 항상 일치하지는 않음. `tar tzf` 줄을 먼저 돌려보고 거기 나온 이름을 쓸 것.

---

## 7. 역할별 파이썬 의존성

버전을 고정해뒀음. 버전을 고정하지 않고 설치하면 기기마다 경로 생성 결과가 달라질 수 있으므로, 패키지 이름으로 직접 설치하지 말고 requirements 파일을 쓸 것.

**planning 역할:**

```bash
cd ~/ros2_ws/src/dae-coverage-floor-flatness
pip install -r requirements-mission_generation.txt
```

**측정 역할:**

```bash
cd ~/ros2_ws/src/dae-coverage-floor-flatness
pip install -r requirements-surface_profiling.txt

# torch는 CUDA 빌드라 기본 PyPI 인덱스에 없어서 따로 설치함
pip install torch==2.11.0+cu128 --index-url https://download.pytorch.org/whl/cu128
```

> GPU가 없거나 CUDA 버전이 다르면 [pytorch.org](https://pytorch.org/get-started/locally/)에서 해당 기기에 맞는 커맨드로 대체할 것. torch는 `surface_profiler.py`의 PointCloud2 좌표 변환에만 쓰이고 경로 생성에는 전혀 안 쓰이므로, CPU 빌드로 깔아도 기능상 문제 없음(속도만 느려짐).

의존성 버전을 바꿔야 한다면 해당 `requirements-*.txt`도 같이 갱신하고 재검증할 것. 기기 간 결과 비교가 가능한 건 이 고정 덕분임.

---

## 8. 측정 기기 — VLP-16

### 8.1 NumPy 2.x용 `transforms3d` 패치

NumPy 2.0에서 `np.maximum_sctype`이 제거돼 `transforms3d`가 import 시점에 실패함:

```bash
sudo sed -i \
  -e 's/_MAX_FLOAT = np.maximum_sctype(np.float)/_MAX_FLOAT = np.float64/' \
  -e 's/_EPS = np.finfo(_MAX_FLOAT).eps \* 4.0/_FLOAT_EPS = np.finfo(np.float64).eps/' \
  /usr/lib/python3/dist-packages/transforms3d/quaternions.py
```

### 8.2 Velodyne 드라이버

```bash
sudo apt update
sudo apt install ros-humble-velodyne
ros2 pkg list | grep velodyne        # velodyne, velodyne_driver, velodyne_pointcloud 등이 나와야 함
```

apt에 해당 바이너리가 없으면 소스 빌드로 전환:

```bash
cd ~/ros2_ws/src
git clone -b humble-devel https://github.com/ros-drivers/velodyne.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to velodyne
source install/setup.bash
```

### 8.3 네트워크

VLP-16은 로봇이 아니라 **이 기기에 이더넷으로 직결**함. 센서의 데이터량이 Jetson이 Nav2에 써야 할 대역폭과 경합하기 때문임.

유선 인터페이스에 센서와 같은 서브넷의 고정 IPv4 주소를 주고(센서의 공장 초기 주소는 VLP-16 매뉴얼 참고), 드라이버가 실제로 발행하는지 확인:

```bash
ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py
# 다른 터미널에서:
ros2 topic hz /velodyne_points
```

실기체에서는 `surface_profiling.launch.py`를 띄우기 **전에** `/velodyne_points`가 발행 중이어야 함. 시뮬레이션에서는 Gazebo가 이 토픽을 대신 발행하므로 드라이버가 필요 없음.

---

## 9. 로봇 — TurtleBot3 bringup 의존성

*실기체에서만 필요하고, 시뮬레이션이면 건너뛸 것.*

`real_bringup.launch.py`와 `tb3_waffle_nav2.launch.py`는 **이 패키지**가 제공하므로, 원본 `turtlebot3_navigation2`/`turtlebot3_gazebo`의 launch 파일은 쓰지 않음. 다만 모터 제어 노드와 LDS 드라이버 때문에 원본 패키지 자체는 여전히 필요함:

```bash
# 2D 라이다 기종에 맞는 LDS 드라이버 (apt. rosdep으로도 같이 깔림)
sudo apt install ros-humble-ld08-driver           # LDS-02
# sudo apt install ros-humble-hls-lfcd-lds-driver # LDS-01
# sudo apt install ros-humble-coin-d4-driver      # LDS-03 (COIN-D4)

# turtlebot3_node(모터 제어)와 메시지는 소스에서
cd ~/ros2_ws/src
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

`real_bringup.launch.py`가 셋 중 어느 드라이버를 띄울지는 3단계의 `LDS_MODEL`로 정해지므로, 설치한 드라이버와 값이 일치해야 함.

로봇에서 추가로 해야 할 것:

- **planning 기기의 `~/dae_floor_maps`를 복사해 올 것** — 최소한 `maps/grid/`, `maps/topology/`, `analytics/metrics/`가 필요함. 트리 전체를 복사하는 게 가장 간단함.
- **OpenCR / USB 권한 설정** — 표준 TurtleBot3 bringup 문서대로 진행할 것.
- **테스트 세션마다 시간 동기화.** 로봇과 노트북이 각자 자기 시계로 산출물 시각을 찍으므로, `chrony`(또는 `ntpdate`)로 맞춰둬야 나중에 짝을 지을 수 있음.

```bash
sudo apt install chrony
# 세션 시작 전마다 로봇을 노트북에, 또는 양쪽을 같은 소스에 동기화할 것
```

---

## 10. 기기 간 네트워크 (CycloneDDS)

*로봇과 노트북이 별개 기기일 때만 해당함.*

`ROS_DOMAIN_ID`와 `RMW_IMPLEMENTATION`을 맞추는 것만으로는 부족함. `docker0`, `tailscale0`, `can0` 같은 가상 인터페이스가 떠 있는 기기는 CycloneDDS가 엉뚱한 쪽을 잡아버려서 **어떤 네트워크에 붙어도** discovery가 실패함. 인터페이스를 명시적으로 지정하면 해결됨.

**네트워크를 바꿀 때마다 다시 해줘야 하지만**, 바뀌는 건 상대방 IP뿐임. 인터페이스 이름은 기기에 고정된 값이라 한 번만 정하면 됨.

**1. 자기 와이파이 인터페이스 이름과 상대 기기의 현재 IP 확인:**

```bash
ip addr
```

`UP` 상태이면서 실제로 공유기/핫스팟에 붙어 있는 인터페이스를 찾음(로봇은 보통 `wlP1p1s0`류, 노트북은 `wlp0s20f3`류). `lo`, `docker0`, `tailscale0`, `can0`은 무시.

**2. 각 기기에 `~/cyclonedds.xml` 작성** (내용이 서로 다름: `NetworkInterfaceAddress`는 자기 인터페이스, `Peer address`는 **상대** 기기의 IP). 복사·붙여넣기 과정에서 따옴표가 깨지는 일이 있으므로 `printf`로 한 줄씩 쓰는 걸 권장함:

```bash
printf '%s\n' \
'<?xml version="1.0" encoding="UTF-8" ?>' \
'<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">' \
'    <Domain id="any">' \
'        <General>' \
'            <NetworkInterfaceAddress>자기_와이파이_인터페이스명</NetworkInterfaceAddress>' \
'        </General>' \
'        <Discovery>' \
'            <Peers>' \
'                <Peer address="상대_기기_IP"/>' \
'            </Peers>' \
'            <ParticipantIndex>auto</ParticipantIndex>' \
'            <MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>' \
'        </Discovery>' \
'    </Domain>' \
'</CycloneDDS>' \
> ~/cyclonedds.xml
```

**3. XML이 제대로 작성됐는지 확인 (양쪽 다):**

```bash
python3 -c "import xml.dom.minidom,os; xml.dom.minidom.parse(os.path.expanduser('~/cyclonedds.xml')); print('XML OK')"
```

**4. 등록 (기기당 최초 1회):**

```bash
echo 'export CYCLONEDDS_URI=file://'$HOME'/cyclonedds.xml' >> ~/.bashrc
source ~/.bashrc
echo $CYCLONEDDS_URI    # file:///home/<사용자명>/cyclonedds.xml
```

**5. 떠 있는 모든 ROS 2 터미널 재시작.** `CYCLONEDDS_URI`는 프로세스 시작 시 한 번만 읽히므로, 이미 떠 있던 노드는 `cyclonedds.xml`을 새로 고쳐도 반영되지 않음.

**6. 노트북에서 새 터미널로 최종 검증:**

```bash
ros2 topic info /tf --verbose              # Publisher count가 1 이상
ros2 topic hz /tf                          # 실제 데이터가 흐르는지
ros2 run tf2_ros tf2_echo map base_footprint   # 좌표가 찍히면 정상
```

> 방화벽이 켜져 있으면(`sudo ufw status`) DDS의 UDP 패킷이 막힐 수 있음. 끄거나 해당 포트를 허용할 것.

---

## 11. 설치 확인

**planning 기기** — 경로를 끝까지 한 번 생성해볼 것:

```bash
cd ~/ros2_ws/src/dae-coverage-floor-flatness/mission_generation
python3 run_generation_pipeline.py
```

생성되어야 하는 것:

- `~/dae_floor_maps/maps/grid/map_from_dae.{pgm,yaml}`
- `~/dae_floor_maps/maps/topology/final_topological_map.npz`
- `~/dae_floor_maps/analytics/metrics/{final_path.json,raw_path.json,final_path_meta.json}`
- `~/dae_floor_maps/visualization/mission_generation/` 아래 디버그 이미지들

주행을 시작하기 전에 `visualization/mission_generation/mission_planning/full_mission_path.png`를 열어 공간 분할과 경로가 의도한 대로 나왔는지 먼저 확인할 것.

**시뮬레이션** — 터미널 4개:

```bash
ros2 launch dae_coverage_floor_flatness sim_env.launch.py
ros2 launch dae_coverage_floor_flatness tb3_waffle_nav2.launch.py use_sim_time:=true
ros2 launch dae_coverage_floor_flatness surface_profiling.launch.py is_sim:=true
ros2 launch dae_coverage_floor_flatness mission_execution.launch.py is_sim:=true
```

**실제로 적용 중인 설정값 확인** — 실제 파이프라인과 완전히 동일한 방식으로 `params.yaml`을 읽어서 출력해줌:

```bash
cd ~/ros2_ws/src/dae-coverage-floor-flatness/surface_profiling
python3 reprocess_pcd.py <아무_combined_파일>.pcd     # 맨 처음에 로드한 설정을 출력함
```

---

## 문제 해결

<details>
<summary><b><code>ImportError: libortools.so...</code> / OR-Tools를 못 찾음</b></summary>

`/usr/local/lib`가 라이브러리 경로에 없거나, 아키텍처가 다른 아카이브를 설치한 경우임. `echo $LD_LIBRARY_PATH`(3단계)를 확인하고 `sudo ldconfig`를 다시 돌린 뒤, 6단계에서 복사한 파일이 `uname -m` 결과와 맞는 아카이브에서 나온 것인지 확인할 것.
</details>

<details>
<summary><b><code>import fields2cover</code> 실패, 또는 기기마다 생성 경로가 다름</b></summary>

거의 항상 버전 불일치임. 모든 planning 기기에서 `git -C ~/Fields2Cover rev-parse --short HEAD`가 `85d6cf7`을 출력하는지, 그리고 PyPI의 `fields2cover`가 소스 빌드를 가리고 있지 않은지(`pip uninstall fields2cover`) 확인할 것.
</details>

<details>
<summary><b><code>transforms3d</code>가 <code>np.maximum_sctype</code>에서 죽음</b></summary>

8.1의 NumPy 2.x 패치를 적용하지 않았거나, 다른 파이썬 설치본에 적용한 경우임. 실제로 import되는 파일을 확인할 것: `python3 -c "import transforms3d, os; print(transforms3d.__file__)"`.
</details>

<details>
<summary><b>launch 실행 시 <code>Unable to parse parameter as yaml</code></b></summary>

`TURTLEBOT3_MODEL` 또는 `LDS_MODEL`이 셸에 export되지 않으면 이 오류가 남. launch 파일을 import하는 시점에 읽히므로 `ros2 launch` 전에 셸에서 export되어 있어야 함.
</details>

<details>
<summary><b><code>params.yaml</code>이나 behavior tree를 고쳤는데 반영이 안 됨</b></summary>

`install/config`와 `install/behavior_trees`는 `build/`를 가리키는 심볼릭 링크임. `colcon build` 후 다시 source할 것. `reprocess_pcd.py`가 로드한 설정을 출력해주므로 실제로 무엇이 적용 중인지 확인할 수 있음.
</details>

<details>
<summary><b>로봇과 노트북이 서로의 토픽을 못 봄</b></summary>

10단계 참고. 가장 흔한 원인은 CycloneDDS가 와이파이 대신 `docker0`/`tailscale0`을 잡은 것이고, 그다음은 네트워크를 바꾼 뒤 peer IP가 낡은 것임.
</details>

<details>
<summary><b>시작 직후 파라미터 불일치 오류로 미션이 중단됨</b></summary>

`final_path_meta.json`에 그 경로를 planning할 때 쓴 파라미터가 기록되어 있고, 실행기는 설정이 다르면 실행을 거부함. `run_generation_pipeline.py`를 다시 돌리거나 `params.yaml`을 되돌릴 것. planning된 transit 시작점이 이 값들에 의존하므로 의도적으로 막아둔 것임.
</details>

---

다음: [실행 순서](../../README.kr.md#실행-순서) · [설정 가이드](configuration.md) · [평가 프로토콜](evaluation.md)
