# Installation Guide

[English](installation.md) · [한국어](../kr/installation.md) · [← README](../../README.md)

This guide covers every machine the system can run on. **You almost certainly do not need all of it** — jump to [Which steps do I need?](#which-steps-do-i-need) first.

Tested on **Ubuntu 22.04 + ROS 2 Humble + Python 3.10**.

---

## Which steps do I need?

The system splits into three roles. One physical machine may take several roles.

| Role | What it does | Required steps |
|---|---|---|
| **Planning** (workstation/laptop, x86_64) | Turns the `.dae` model into `final_path.json`, offline | 0–7 |
| **Driving** (Jetson Orin Nano on the robot, arm64) | Runs Nav2 and `mission_executor` | 0–4, 6, 9, 10 |
| **Sensing** (laptop with the VLP-16 attached, x86_64) | Runs `surface_profiler` | 0–4, 7, 8, 10 |

**Simulation only, single machine?** You need steps 0–8 on that one machine, and you can skip 9 and 10 entirely.

---

## 0. Prerequisites

- Ubuntu 22.04
- ROS 2 Humble ([installation](https://docs.ros.org/en/humble/Installation.html)), with `source /opt/ros/humble/setup.bash` working
- `colcon`, `rosdep` initialised (`sudo rosdep init && rosdep update`)

ROS packages pulled in by `rosdep` for this package: `rclpy`, `nav2_simple_commander`, `nav2_msgs`, `gazebo_msgs`, `sensor_msgs_py`, `tf_transformations`, `laser_filters`.

Additional ROS packages that specific launch files expect:

| Package | Needed by | Required for |
|---|---|---|
| `nav2_bringup` | `tb3_waffle_nav2.launch.py` | Driving, Simulation |
| `gazebo_ros`, `turtlebot3_description` | `sim_env.launch.py` | Simulation |
| `turtlebot3_bringup`, `turtlebot3_node`, and the LDS driver for your model (`hls_lfcd_lds_driver` / `ld08_driver` / `coin_d4_driver`) | `real_bringup.launch.py` | Real robot only |
| `velodyne` (driver) | VLP-16 data acquisition | Sensing on real hardware |

---

## 1. External data store

All inputs and outputs live **outside** the repository, in `~/dae_floor_maps`. Create it on every machine that runs any stage:

```bash
mkdir -p ~/dae_floor_maps/{assets,maps/{debug_image,grid,topology},\
analytics/{metrics,paths,pointclouds,logs},\
visualization/{mission_generation/{environment_modeling,mission_planning},mission_execution,surface_profiling}}
```

Put your 3D model in `~/dae_floor_maps/assets/` (e.g. `Apt.dae`), then set `environment_modeling.dae_file` in `config/params.yaml` to that filename.

> The path is configurable via `global.workspace_root` in `config/params.yaml`, but every machine must agree on it.

---

## 2. Workspace and build

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

> **Re-source after every build.** Open a new terminal, or run `source ~/ros2_ws/install/setup.bash`.
>
> `install/config` and `install/behavior_trees` are symlinks into `build/`. Editing `params.yaml` or a behavior tree still requires a `colcon build` before it takes effect at runtime.

---

## 3. Shell environment

Append to `~/.bashrc`, then `source ~/.bashrc`:

```bash
# --- ROS 2 Humble & TurtleBot3 ---
source /opt/ros/humble/setup.bash
# Load the workspace manually when needed:
# source ~/ros2_ws/install/setup.bash

export TURTLEBOT3_MODEL=waffle
export LDS_MODEL=LDS-02          # match your robot's 2D LiDAR
export ROS_DOMAIN_ID=30          # identical on every machine
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Gazebo (simulation only)
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/opt/ros/humble/lib

# OR-Tools installs into /usr/local/lib
export LD_LIBRARY_PATH=/usr/local/lib:/opt/ros/humble/lib:$LD_LIBRARY_PATH
```

`TURTLEBOT3_MODEL` and `LDS_MODEL` are read at launch-file *import* time, so they must be exported before `ros2 launch`, not passed as arguments.

`sim_env.launch.py` sets `GAZEBO_MODEL_PATH` itself from this package's `models/` directory, so you do not need to export it.

Optional but recommended — a hard reset alias, since a leftover Gazebo or Nav2 process from a previous run is the most common cause of confusing behaviour:

```bash
alias rrr='ros2 daemon stop; \
killall -9 gzserver gzclient; \
pkill -9 -f ros2; pkill -9 -f fastdds; \
pkill -9 -f robot_state_publisher; pkill -9 -f ekf; \
pkill -9 -f nav2; pkill -9 -f rviz; pkill -9 -f spawn_entity.py'
```

---

## 4. System packages (all machines)

```bash
sudo apt update
sudo apt install build-essential libgdal-dev libgeos-dev libeigen3-dev libboost-dev \
     libtbb-dev libtinyxml2-dev nlohmann-json3-dev libpython3-dev gnuplot
sudo apt install ros-humble-rmw-cyclonedds-cpp

python3 -m pip install --upgrade pip wheel
pip install --user "setuptools<80,>=30.3.0"
```

> **OpenCV:** `package.xml` declares the apt package `python3-opencv`, but the pip wheel in the user site-packages always takes precedence and is what actually gets imported. The `requirements-*.txt` files pin `opencv-python-headless` for exactly this reason — install them (step 7) and let apt's copy sit unused.

---

## 5. Fields2Cover — pinned source build

*Planning role only.*

**Do not install Fields2Cover from PyPI.** The published release generates different swath geometry than the version this project was validated against, which changes the node visit order and makes results non-comparable between machines. Build from source at the pinned commit:

```bash
cd ~
git clone https://github.com/Fields2Cover/Fields2Cover.git
cd ~/Fields2Cover
git checkout 85d6cf7      # do not substitute a newer tag or commit
mkdir -p build && cd build
cmake .. -DBUILD_PYTHON=ON
make -j$(nproc)
sudo make install
sudo ldconfig
```

Verify:

```bash
python3 -c "import fields2cover as f2c; print(f2c.DECOMP_Boustrophedon)"
```

> The Python bindings expose only part of the C++ API. If `help(f2c.X)` shows no signature, check the C++ headers or `~/Fields2Cover/tutorials/python/`.

On the robot, Fields2Cover is not used at runtime — paths are computed on the planning machine and transferred as a file. Installing it there anyway is harmless and avoids import errors if any code path reaches `mission_planner.py`.

---

## 6. OR-Tools shared libraries

The TSP solver needs OR-Tools' C++ shared libraries. **The archive is architecture-specific** — use the matching one.

**x86_64 (laptop / workstation):**

```bash
wget https://github.com/google/or-tools/releases/download/v9.9/or-tools_amd64_ubuntu-22.04_cpp_v9.9.3963.tar.gz \
     -O ~/or-tools_v9.9.3963.tar.gz
tar tzf ~/or-tools_v9.9.3963.tar.gz | head -3     # check the actual top-level directory name
tar xzf ~/or-tools_v9.9.3963.tar.gz -C ~
sudo cp -P ~/or-tools_x86_64_Ubuntu-22.04_cpp_v9.9.3963/lib/*.so* /usr/local/lib/
sudo ldconfig
```

**arm64 (Jetson Orin Nano):**

```bash
wget https://github.com/google/or-tools/releases/download/v9.9/or-tools_arm64_debian-11_cpp_v9.9.3963.tar.gz \
     -O ~/or-tools_v9.9.3963.tar.gz
tar tzf ~/or-tools_v9.9.3963.tar.gz | head -3     # check the actual top-level directory name
tar xzf ~/or-tools_v9.9.3963.tar.gz -C ~
sudo cp -P ~/or-tools_aarch64_Debian-11_cpp_v9.9.3963/lib/*.so* /usr/local/lib/
sudo ldconfig
```

The extracted directory name does not always match the archive name — run the `tar tzf` line and use whatever it prints.

---

## 7. Python dependencies per role

Versions are pinned. Installing these packages unpinned has been observed to change path-generation output between machines, so use the requirements files rather than installing by name.

**Planning role:**

```bash
cd ~/ros2_ws/src/dae-coverage-floor-flatness
pip install -r requirements-mission_generation.txt
```

**Sensing role:**

```bash
cd ~/ros2_ws/src/dae-coverage-floor-flatness
pip install -r requirements-surface_profiling.txt

# torch is a CUDA build and is not on the default PyPI index, so it is installed separately
pip install torch==2.11.0+cu128 --index-url https://download.pytorch.org/whl/cu128
```

> No GPU, or a different CUDA version? Pick the matching command from [pytorch.org](https://pytorch.org/get-started/locally/). Torch is used only for the PointCloud2 coordinate transform in `surface_profiler.py`, never for path generation — a CPU build works correctly, just more slowly.

If you change a dependency version, update the corresponding `requirements-*.txt` and re-validate; the pinning is what makes runs comparable across machines.

---

## 8. Sensing machine — VLP-16

### 8.1 Patch `transforms3d` for NumPy 2.x

`np.maximum_sctype` was removed in NumPy 2.0, so `transforms3d` crashes on import:

```bash
sudo sed -i \
  -e 's/_MAX_FLOAT = np.maximum_sctype(np.float)/_MAX_FLOAT = np.float64/' \
  -e 's/_EPS = np.finfo(_MAX_FLOAT).eps \* 4.0/_FLOAT_EPS = np.finfo(np.float64).eps/' \
  /usr/lib/python3/dist-packages/transforms3d/quaternions.py
```

### 8.2 Velodyne driver

```bash
sudo apt update
sudo apt install ros-humble-velodyne
ros2 pkg list | grep velodyne        # expect velodyne, velodyne_driver, velodyne_pointcloud, ...
```

If apt has no binary for your setup, build from source:

```bash
cd ~/ros2_ws/src
git clone -b humble-devel https://github.com/ros-drivers/velodyne.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to velodyne
source install/setup.bash
```

### 8.3 Network

The VLP-16 is wired directly to this machine over Ethernet, not to the robot — the sensor's data rate would otherwise compete with the Nav2 traffic the Jetson needs.

Give the wired interface a static IPv4 address in the sensor's subnet (see the VLP-16 manual for its factory default address), then confirm the driver publishes:

```bash
ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py
# in another terminal:
ros2 topic hz /velodyne_points
```

`/velodyne_points` must be publishing before you start `surface_profiling.launch.py` on real hardware. In simulation, Gazebo publishes this topic instead and no driver is needed.

---

## 9. Robot — TurtleBot3 bringup dependencies

*Real robot only; skip for simulation.*

`real_bringup.launch.py` and `tb3_waffle_nav2.launch.py` are provided by **this** package, so the original `turtlebot3_navigation2` and `turtlebot3_gazebo` launch files are not used. The upstream packages are still needed for the motor-control node and the LDS driver:

```bash
# LDS driver for your 2D LiDAR model (apt; rosdep also pulls these in)
sudo apt install ros-humble-ld08-driver           # LDS-02
# sudo apt install ros-humble-hls-lfcd-lds-driver # LDS-01
# sudo apt install ros-humble-coin-d4-driver      # LDS-03 (COIN-D4)

# turtlebot3_node (motor control) and its messages, from source
cd ~/ros2_ws/src
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

`LDS_MODEL` (step 3) selects which of these `real_bringup.launch.py` starts, so it must match the driver you installed.

Also on the robot:

- **Copy `~/dae_floor_maps` over from the planning machine** — at minimum `maps/grid/`, `maps/topology/` and `analytics/metrics/`. Copying the whole tree is simplest.
- **Set up OpenCR / USB permissions** as per the standard TurtleBot3 bringup instructions.
- **Synchronise clocks before every test session.** The robot and the laptop timestamp their outputs independently; `chrony` (or `ntpdate`) keeps them consistent enough to pair.

```bash
sudo apt install chrony
# then sync the robot to the laptop, or both to the same source, before each session
```

---

## 10. Multi-machine networking (CycloneDDS)

*Only when the robot and the laptop are separate machines.*

Matching `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` is not sufficient. A machine with virtual interfaces up (`docker0`, `tailscale0`, `can0`) will let CycloneDDS bind to the wrong one and discovery then fails on **every** network. Pin the interface explicitly.

**This must be redone whenever the network changes** — but only the peer IP changes; the interface name is fixed per machine.

**1. Find your Wi-Fi interface name and the other machine's current IP:**

```bash
ip addr
```

Look for the interface that is `UP` and actually connected (robot: often `wlP1p1s0`-style; laptop: often `wlp0s20f3`-style). Ignore `lo`, `docker0`, `tailscale0`, `can0`.

**2. Write `~/cyclonedds.xml` on each machine** (different content on each: `NetworkInterfaceAddress` is that machine's own interface, `Peer address` is the *other* machine's IP). Using `printf` avoids quote mangling from copy-paste:

```bash
printf '%s\n' \
'<?xml version="1.0" encoding="UTF-8" ?>' \
'<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">' \
'    <Domain id="any">' \
'        <General>' \
'            <NetworkInterfaceAddress>YOUR_WIFI_INTERFACE</NetworkInterfaceAddress>' \
'        </General>' \
'        <Discovery>' \
'            <Peers>' \
'                <Peer address="OTHER_MACHINE_IP"/>' \
'            </Peers>' \
'            <ParticipantIndex>auto</ParticipantIndex>' \
'            <MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>' \
'        </Discovery>' \
'    </Domain>' \
'</CycloneDDS>' \
> ~/cyclonedds.xml
```

**3. Check the XML parses (both machines):**

```bash
python3 -c "import xml.dom.minidom,os; xml.dom.minidom.parse(os.path.expanduser('~/cyclonedds.xml')); print('XML OK')"
```

**4. Register it (once per machine):**

```bash
echo 'export CYCLONEDDS_URI=file://'$HOME'/cyclonedds.xml' >> ~/.bashrc
source ~/.bashrc
echo $CYCLONEDDS_URI    # file:///home/<user>/cyclonedds.xml
```

**5. Restart every ROS 2 terminal.** `CYCLONEDDS_URI` is read once at process start, so already-running nodes will not pick up a new or edited `cyclonedds.xml`.

**6. Verify from the laptop, in a fresh terminal:**

```bash
ros2 topic info /tf --verbose              # Publisher count >= 1
ros2 topic hz /tf                          # data actually flowing
ros2 run tf2_ros tf2_echo map base_footprint   # real coordinates = fully working
```

> If a firewall is enabled (`sudo ufw status`), it can block DDS UDP traffic. Disable it or allow the relevant ports.

---

## 11. Verify the installation

**Planning machine** — generate a path end to end:

```bash
cd ~/ros2_ws/src/dae-coverage-floor-flatness/mission_generation
python3 run_generation_pipeline.py
```

Expect:

- `~/dae_floor_maps/maps/grid/map_from_dae.{pgm,yaml}`
- `~/dae_floor_maps/maps/topology/final_topological_map.npz`
- `~/dae_floor_maps/analytics/metrics/{final_path.json,raw_path.json,final_path_meta.json}`
- Debug images under `~/dae_floor_maps/visualization/mission_generation/`

Open `visualization/mission_generation/mission_planning/full_mission_path.png` and check the decomposition and path look sensible before driving anything.

**Simulation** — four terminals:

```bash
ros2 launch dae_coverage_floor_flatness sim_env.launch.py
ros2 launch dae_coverage_floor_flatness tb3_waffle_nav2.launch.py use_sim_time:=true
ros2 launch dae_coverage_floor_flatness surface_profiling.launch.py is_sim:=true
ros2 launch dae_coverage_floor_flatness mission_execution.launch.py is_sim:=true
```

**Confirm the configuration actually in effect** — this reads `params.yaml` through exactly the same resolution path as the live pipeline:

```bash
cd ~/ros2_ws/src/dae-coverage-floor-flatness/surface_profiling
python3 reprocess_pcd.py <some_combined_file>.pcd     # prints the loaded config first
```

---

## Troubleshooting

<details>
<summary><b><code>ImportError: libortools.so...</code> / OR-Tools not found</b></summary>

`/usr/local/lib` is not on the library path, or the wrong architecture's archive was installed. Check `echo $LD_LIBRARY_PATH` (step 3), re-run `sudo ldconfig`, and confirm the files you copied in step 6 came from the archive matching `uname -m`.
</details>

<details>
<summary><b><code>import fields2cover</code> fails, or generated paths differ between machines</b></summary>

Almost always a version mismatch. Confirm `git -C ~/Fields2Cover rev-parse --short HEAD` prints `85d6cf7` on every planning machine, and that no PyPI `fields2cover` is shadowing the source build (`pip uninstall fields2cover`).
</details>

<details>
<summary><b><code>transforms3d</code> crashes with <code>np.maximum_sctype</code></b></summary>

The NumPy 2.x patch in step 8.1 has not been applied, or was applied to a different Python installation. Check which file is imported: `python3 -c "import transforms3d, os; print(transforms3d.__file__)"`.
</details>

<details>
<summary><b><code>Unable to parse parameter as yaml</code> on launch</b></summary>

`TURTLEBOT3_MODEL` or `LDS_MODEL` is unset. They are read when the launch file is imported, so export them in the shell before `ros2 launch`.
</details>

<details>
<summary><b>Edits to <code>params.yaml</code> or a behavior tree have no effect</b></summary>

`install/config` and `install/behavior_trees` are symlinks into `build/`. Run `colcon build` and re-source. `reprocess_pcd.py` prints the configuration it loaded, so you can confirm what is actually live.
</details>

<details>
<summary><b>Robot and laptop cannot see each other's topics</b></summary>

See step 10. The usual cause is CycloneDDS binding to `docker0` or `tailscale0` instead of the Wi-Fi interface; the second most common is a stale peer IP after changing networks.
</details>

<details>
<summary><b>Mission aborts at startup with a parameter-mismatch error</b></summary>

`final_path_meta.json` records the parameters the path was planned with, and the executor refuses to run against a different configuration. Either re-run `run_generation_pipeline.py` or restore `params.yaml` to match. The planned transit start points depend on those values, so this check is deliberate.
</details>

---

Next: [Run order](../../README.md#run-order) · [Configuration guide](configuration.md) · [Evaluation protocol](evaluation.md)
