# 3D LiDAR Mount Correction and Fixed-Speed Drive Measurement Guide

Procedure for driving the robot **at the same fixed speed used in measurement** (no mission), collecting floor point clouds, inspecting the accumulation video, and deriving a roll/pitch mount correction for `params.yaml`. It applies to both simulation and the real robot.

## 1. What is corrected

If the TF pose of `velodyne_link` differs from the sensor's physical pose by a fixed roll/pitch offset, the z of a flat floor tilts with distance from the sensor (`z ≈ a·forward + b·left`). The error grows linearly with range: a 1° tilt gives about 5 cm at 3 m.

The `surface_profiling` node multiplies an extra rotation into the sensor frame to cancel it.

```yaml
# config/params.yaml - surface_profiling
lidar_mount_correction_rpy_deg_sim:  [0.0, 0.0, 0.0]   # [roll, pitch, yaw] in deg, same convention as URDF joint rpy
lidar_mount_correction_rpy_deg_real: [0.0, 0.0, 0.0]
```

- The correction is applied to point coordinates at collection time, so every later `.pcd`, `frames_*.npz`, heatmap and video reflects it.
- Yaw is not derived by this procedure; leave it at 0.
- With `colcon build --symlink-install`, `params.yaml` is linked into the install tree: after editing, restart the node without rebuilding.

## 2. Keyboard drive and point-cloud collection

`mission_execution` is not started; the capture window is opened and closed by calling the capture services directly.

### 2-1. Launch order

**Simulation**

```bash
ros2 launch dae_coverage_floor_flatness sim_env.launch.py                                    # 1
ros2 launch dae_coverage_floor_flatness tb3_waffle_nav2.launch.py use_sim_time:=true          # 2
ros2 launch dae_coverage_floor_flatness surface_profiling.launch.py is_sim:=true              # 3
# 5. Drive with the speed command in section 2-3 (keyboard teleop is not used: its speed differs from the measurement speed)
```

**Real robot** (same order as the README run order, with the fixed-speed command instead of mission execution)

```bash
ros2 launch dae_coverage_floor_flatness real_bringup.launch.py                                # [Jetson]
ros2 launch dae_coverage_floor_flatness tb3_waffle_nav2.launch.py use_sim_time:=false         # [Jetson]
ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py                                       # [Laptop]
ros2 launch dae_coverage_floor_flatness surface_profiling.launch.py is_sim:=false             # [Laptop]
# Drive with the speed command in section 2-3 (from a machine that reaches the robot)
```

- Points are recorded only while a `map → velodyne_link` TF exists. Without a mission, set an initial pose (e.g. in RViz) and let AMCL converge before starting.
- Points are stored in map coordinates, so the wall exclusion in the analysis (`--map`) and the wall overlay in the video line up with the floor plan only if the AMCL pose is accurate.

### 2-2. Capture control

```bash
# Start capture (only points arriving after this are used in the result)
ros2 service call /surface_profiling/start_waypoint_capture std_srvs/srv/Trigger

# ... drive with the speed command of section 2-3, or hold still ...

# Stop capture
ros2 service call /surface_profiling/stop_waypoint_capture std_srvs/srv/Trigger

# End collection -> floor extraction, heatmap and accumulation video are generated
ros2 service call /surface_profiling/stop_collection_success std_srvs/srv/Trigger
```

- Capture windows can be opened and closed repeatedly; always call start and stop as a pair.
- Without mission artefacts the stall-analysis stage may be skipped or warn at the end; earlier outputs are unaffected.

### 2-3. Fixed-speed driving

The measurement speed is `mission_execution.coverage_speed_limit_mps` (default 0.16 m/s). Keyboard teleoperation does not run at that speed, so publish a constant velocity to `/cmd_vel` a fixed number of times instead.

```bash
# Forward: 0.16 m/s x 375 messages at 20 Hz = about 3 m (distance = speed x count / rate)
ros2 topic pub --times 375 -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.16}}"

# Reverse
ros2 topic pub --times 375 -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.16}}"
```

- This is an open-loop command published directly, so run it with no active Nav2 goal, and check that the space ahead is clear.
- A step velocity command produces acceleration right after start and stop. Open the capture window 1 s after the start and close it 1 s before the end so that only the constant-speed part is recorded.
- The speed profile is not identical to a mission's (whose path-following controller applies its own acceleration limits), so use this for checking bias in the constant-speed segment.

| Item | Value | Reason |
|---|---|---|
| Linear speed | Same as `coverage_speed_limit_mps` | Gives the same point spacing as measurement |
| Rotation | Slowly, with capture off | Frames exceeding the low-pass filter thresholds (default 0.3 m/s, 20 deg/s) are dropped entirely |
| Floor | Flat area at least 0.4 m from walls | Wall-surface points contaminate z statistics |
| LiDAR speed | Fixed at 600 rpm | Point density and travel per revolution are designed around it |

The number of rejected frames appears in the video header (`rejected_frames`) and in the collection log.

### 2-4. Recommended driving patterns

| Goal | Pattern |
|---|---|
| Check for a direction-dependent bias | Capture the same 3-4 m line **forward, then in reverse**. If the colour bias stays on the robot's front/back it is a sensor-pose effect; if it follows the travel direction it is a dynamic effect (attitude change, latency) |
| Derive the mount correction | Hold still at one spot facing **0°, 90°, 180°, 270°** and capture **the same duration (e.g. 10 s)** at each. Equal frame counts per heading make the floor's own slope cancel in the average |

## 3. Inspecting the accumulation video

At the end of collection, `accumulation_<timestamp>.mp4` is written to `visualization_dir` (default `~/dae_floor_maps/visualization/surface_profiling/`). Walls and the robot are black; points are coloured by z.

| Colour | Meaning |
|---|---|
| jet (blue to red) | **Mean z** of the in-window points accumulated in the cell. Same colormap, aggregation and colour range as the heatmap |
| gray | The cell holds only points below the z window |
| magenta | The cell holds only points above the z window (mostly wall-surface points) |

The final frame converges to the heatmap values (the heatmap averages after a 1 cm voxel downsample, so only small differences remain). The heatmap colour bar shows actual z in cm.

The video can be re-rendered from the stored log with different options:

```bash
python3 surface_profiling/make_accumulation_video.py \
  ~/dae_floor_maps/analytics/pointclouds/frames/frames_<timestamp>.npz \
  --z-min -0.01 --z-max 0.01 --z-margin 0.05
```

- A narrower colour range (e.g. ±1 cm) magnifies small differences. Cells observed only a few times (the area first seen ahead of the robot) have not converged to their mean, so sensor noise (about 0.2 cm standard deviation in simulation) shows up as colour; confirm any apparent bias with the numbers in section 4. Near walls, wall-surface points raise z.
- Options: `--fps`, `--max-frames` (video frame limit), `--max-dim` (resolution), `--out`.

## 4. Verify numerically and derive the correction

```bash
python3 surface_profiling/analyze_z_bias.py \
  ~/dae_floor_maps/analytics/pointclouds/frames/frames_<timestamp>.npz \
  --map ~/dae_floor_maps/maps/grid/map_from_dae.yaml
```

Example output:

```
z(cm): mean 0.007  std 0.222  p1/p99 -0.52/0.54
pitch(deg, +=front higher): mean 0.000  std(between frames) 0.007
roll (deg, +=left higher): mean 0.003  std(between frames) 0.009
suggested lidar_mount_correction_rpy_deg = [-0.003, 0.000, 0.0]
```

- Points within `--wall-margin` (default 0.4 m) of a wall are excluded, and only floor points at 1.2-3.5 m from the sensor (`--r-min/--r-max`, |z| ≤ `--z-band`, default 0.1 m) are used. For each frame the plane `z = a·f + b·l + c` is fitted by least squares, giving pitch = atan(a) and roll = atan(b).
- A small **between-frame standard deviation** means a fixed offset (correctable); a large one means the tilt varies while driving, which this procedure cannot correct.
- Direction check: analyse the forward and reverse data separately and see whether the pitch sign stays fixed in the robot frame.

### 4-1. Applying the correction

1. Collect the four-heading data of section 2-4 with the correction off (`[0, 0, 0]`).
2. Collect all four headings in one run (opening and closing capture windows) and run `analyze_z_bias.py`. With equal frame counts per heading, the mean pitch/roll is the mount tilt.
3. Put the suggested value into the matching key (`_sim` or `_real`) in `params.yaml`.
4. Restart `surface_profiling`, collect again the same way, and confirm pitch/roll are close to 0.
5. If the data was collected with a correction already enabled, the suggested value is a residual that must be **added to the current value**.

Sign convention: `pitch` follows the URDF joint rpy convention (positive rotates the sensor frame about y), and the suggested value can be used as printed without converting signs.

### 4-2. Acceptance criteria

| Metric | Criterion (provisional; fix per site) |
|---|---|
| Mean \|pitch\|, \|roll\| | Well below target defect height / maximum used range (e.g. under one third of 1 cm / 3.5 m ≈ 0.16°) |
| Front/back mean z difference | Under one tenth of the target defect height |
| Between-frame pitch std | Well below the mean offset (so it can be treated as fixed) |

## 5. Limitations

- The plane fit assumes a flat floor, so calibrating on the very floor whose flatness is to be measured is circular. The four-heading average reduces this because the floor's slope does not rotate with the robot.
- Only a **fixed** offset is corrected. Dynamic tilt, such as the chassis tilting while crossing a defect, is not corrected (the current TF is planar and carries no roll/pitch).
- The usable range (1.2-3.5 m) and the minimum points per frame (300) are defaults; adjust `--r-min`, `--r-max` and `--min-points` for other sensors or mount heights.
