# Real-Robot Stack Launcher (`stack.py`)

A tool that brings up the start-up sequence spanning the robot (Jetson) and the LiDAR laptop from **a single laptop terminal**. Each stage must pass a gate (an automatic check) before the next one starts, which replaces watching terminal logs by eye before typing the next command. It targets the real robot only; for simulation follow the run order in the [README](../../README.md).

## 1. What it starts

| Stage | Runs on | What it runs | Gate (pass condition) |
|---|---|---|---|
| `preflight` | both | nothing (check only) | no leftover processes from a previous run |
| `bringup` | Jetson | `real_bringup.launch.py use_imu_tilt:=<on/off>` | `/scan` 3-20 Hz, `/imu` 5-500 Hz, `/odom` 10-100 Hz, exactly one publisher on each, TF `odom→base_footprint` and `base_footprint→base_link`, IMU TF mode matches the request |
| `nav2` | Jetson | `tb3_waffle_nav2.launch.py use_rviz:=false` | `/map_server` and `/amcl` lifecycle are `active` |
| `velodyne` | laptop | `velodyne-all-nodes-VLP16-launch.py` | `/velodyne_points` 8-12 Hz (600 rpm = 10 Hz), exactly one publisher |
| `pose` | laptop | publishes `/initialpose` (when `--init` is given) | TF `map→odom` exists, `/amcl_pose` received |
| `profiler` | laptop | `surface_profiling.launch.py is_sim:=false` | TF `map→velodyne_link` |

Mission execution (`mission_execution.launch.py`) is not included. The start-pose alignment and the moment to start are the operator's decision, so run the last step of the [README](../../README.md) yourself.

Jetson processes are started through `bash -ic` (a shell that applies `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION` and the rest of `~/.bashrc`) and keep running after the script exits. Logs go to `~/stack_logs/<stage>.log` on each machine.

## 2. Prerequisites (once)

- **Jetson**: update and build this package (`git pull`, then `colcon build --symlink-install`). `imu_tilt_broadcaster` runs on the Jetson, so the IMU bias is read from the **Jetson's** `config/params.yaml`. The maps (`~/dae_floor_maps`) must also be on the Jetson.
- **Laptop**: run from a terminal with ROS and this package sourced. The gates check over DDS directly, so both machines need the same `ROS_DOMAIN_ID`.
- Synchronise the two clocks (chrony or similar), because point clouds are transformed with the TF at their stamp time.
- Pass the connection details as environment variables, which keeps the password off the command line.

```bash
export ROBOT_HOST=<Jetson IP>
export SSH_PASSWORD=<password>      # user defaults to waffle, override with --user
```

If the repository folder is not named `dae-coverage-floor-flatness` or the workspace is not `~/ros2_ws`, pass `--ws <workspace path>` and `--repo <repository path>`. If the Jetson's paths differ from the laptop's, set them separately with `--jetson-ws` and `--jetson-repo`. If the workspace has no `install/setup.bash`, the run stops immediately and says so.

## 3. Commands

Run every command on the **laptop**, from the package root.

```bash
# Bring everything up: IMU TF off, AMCL initial pose given as arguments
python3 scripts/stack.py up --imu-tf off --init 1.20 0.80 0.0

# Bring up one stage at a time (waits for Enter after each stage)
python3 scripts/stack.py up --imu-tf off --step

# Stop at a given stage (for example, robot bringup only)
python3 scripts/stack.py up --until bringup

# Status check (read-only, does not touch a running stack)
python3 scripts/stack.py check

# Stop everything on both machines
python3 scripts/stack.py down
```

| Option | Default | Description |
|---|---|---|
| `--imu-tf on\|off` | `off` | Fixes the `imu_tilt_broadcaster` mode through a launch argument. The mode is also stored in the measurement records |
| `--init X Y YAW` | none | AMCL initial pose in the map frame (metres, radians). Without it, set the pose in RViz (section 4) |
| `--map <yaml>` | Nav2 launch default | Map passed to Nav2 |
| `--until <stage>` | `profiler` | Run up to and including this stage |
| `--step` | off | Confirm with Enter after each stage |
| `--clean` | off | Stop leftover processes first. Without it, the run aborts and reports them |
| `--pose-wait <s>` | 180 | How long to wait for a manual RViz initial pose |
| `--jetson-ws <path>` | same as `--ws` | Workspace path on the Jetson |
| `--local` | off | Run the Jetson-side commands on this computer as well (for testing) |

When a gate fails, the last 15 lines of that stage's log are printed and the run stops. Use `check` to inspect the state, fix the cause, then start again from `down` (or with `--clean`).

## 4. AMCL initial pose

- **With `--init` (recommended)**: the script publishes `/initialpose` in the map frame and waits until `map→odom` appears. Give the map coordinates and heading of where the robot was placed. A heading error is baked into `map→odom` and makes the whole path look skewed, so set it accurately.
- **In RViz**: without `--init`, the `pose` stage waits. Open RViz on the laptop, **set Global Options → Fixed Frame to `map`**, then use 2D Pose Estimate. The published `/initialpose` carries the Fixed Frame as its `frame_id`; if that is not `map`, AMCL rejects it and nothing happens.

```bash
rviz2 -d $(ros2 pkg prefix dae-coverage-floor-flatness)/share/dae-coverage-floor-flatness/rviz/tb3_navigation2.rviz
```

## 5. Status check: `check`

Prints the following for a running stack in one pass. Use it to narrow down why `map→odom` does not appear even though an initial pose was given.

| Output | What it shows |
|---|---|
| Topic rates and publisher counts | `/scan`, `/imu`, `/odom`, `/velodyne_points`. Two or more publishers means a driver is running twice |
| TF presence | `odom→base_footprint`, `base_footprint→base_link`, `odom→base_scan`, `map→odom`, `map→velodyne_link` |
| IMU TF | Current mode, the bias, and the key it came from |
| Nav2 | Lifecycle state of `/map_server` and `/amcl`, AMCL's `set_initial_pose` and `initial_pose.*` values, whether `/amcl_pose` was received |
| AMCL log | When started with `stack.py up`, the lines about initial-pose handling and TF transform failures |

With `--try-pose X Y YAW`, it also publishes a map-frame `/initialpose` and reports whether `map→odom` appears.

## 6. Troubleshooting

| Symptom | Action |
|---|---|
| A process is already running | Run `stack.py down` and retry, or pass `--clean` |
| `NO_SCRIPT ...` | That machine has not pulled the update, or `--repo` points to the wrong path |
| Publisher-count gate fails | A driver is already running. Run `down` and retry |
| `map→odom` gate fails | Run `check` to see the AMCL state. If the pose was set in RViz, confirm the Fixed Frame is `map` |
| IMU TF mode gate fails | The Jetson is on an older version, or `bringup` is already running in another mode. Run `down` and retry |
| Connection fails | Check `ROBOT_HOST`, `SSH_PASSWORD` and the Jetson's SSH server |

`down` runs `scripts/stop_all.sh` on each machine. It can also be used on its own (see [installation.md](installation.md#3-shell-environment)) and it cleans up Gazebo and RViz as well.

## 7. Limitations

- The rate ranges in the gates are defaults. For a different sensor model or configuration, adjust the values at the `rate_gate` calls in `scripts/stack.py`.
- SSH uses password authentication only (the same approach as `auto_calibration_drive.py`).
- Processes are matched by name, so `down` also stops unrelated ROS processes running on the same machine.
