# IMU Body-Tilt Correction and Drive Verification Guide

Procedure to run before feeding the robot's IMU (OpenCR) roll/pitch into TF: (1) **measure the fixed bias while stationary**, and (2) **verify while driving that the IMU follows the real chassis tilt, by comparing it with the floor plane seen by the LiDAR**. It targets the real robot. The LiDAR mount correction is covered in [lidar_calibration.md](lidar_calibration.md).

## 1. What this does

`imu_tilt_broadcaster` (started by `real_bringup.launch.py` and `sim_env.launch.py`) subtracts a bias from the roll/pitch of `/imu` and publishes the result as the `base_footprint → base_link` TF. `velodyne_link` hangs below `base_link` through a fixed joint, so this tilt propagates directly into the point-cloud transform. Yaw is not injected because odom/AMCL already provide it.

```yaml
# config/params.yaml - mission_execution
imu_mount_correction_rpy_deg: [0.0, 0.0]   # [roll, pitch] in deg, subtracted from /imu before it goes into TF
```

| Stage | What it finds | Method | Why |
|---|---|---|---|
| Section 3, static | Fixed bias (mounting error, zero offset) | Stationary measurement at 4 headings 90° apart, then average | Bias and floor slope cannot be separated from one heading. Rotating the robot cancels the floor component in the average |
| Section 4, driving | Dynamic error (acceleration, vibration, delay) | Compare IMU tilt with per-frame LiDAR plane-fit tilt on a common time axis | Not visible when stationary, so this is **verification**, not correction |

> **Order**: IMU (this document) → LiDAR correction. The LiDAR correction is the residual left when the IMU TF is active, so if the IMU setting changes, redo the LiDAR correction.
>
> **Caution**: `imu_mount_correction_rpy_deg` has no separate simulation/real keys. If the real-robot value is left in place when running Gazebo, it is subtracted from the bias-free simulated IMU and the TF ends up tilted. Reset it to `[0.0, 0.0]` for simulation.

Script: `surface_profiling/test/check_imu_dynamics.py` (commands below are relative to the package root and run on the **Laptop**). Subcommands: `static` (stationary logging), `record` (drive logging), `analyze` (comparison).

## 2. Prerequisites

- Jetson and Laptop must see each other's topics (same `ROS_DOMAIN_ID`). `ros2 topic hz /imu` on the Laptop should show about 20 Hz.
- The drive verification in section 4 needs **synchronized clocks** on the Jetson and Laptop (IMU stamps come from the Jetson, frame stamps from the Laptop). Otherwise the offset is mixed into the reported delay.
- LiDAR-side preparation (Velodyne driver, AMCL initial pose) is the same as section 2-1 of [lidar_calibration.md](lidar_calibration.md).

## 3. Static 4-heading bias measurement

**Two terminals** are needed. Nav2, the LiDAR and the measurement node are not required.

| Step | Machine | Command / action |
|---|---|---|
| 1 | **Jetson** J1 | `ros2 launch dae_coverage_floor_flatness real_bringup.launch.py` |
| 2 | **Laptop** L1 | With the robot on a flat floor and **completely still**: `python3 surface_profiling/test/check_imu_dynamics.py static --heading 0 --duration 30` |
| 3 | **Jetson** J2 | Rotate the robot 90° in place: `python3 surface_profiling/test/check_imu_dynamics.py rotate --deg 90` (it watches the `/odom` yaw, slows down near the target and **stops by itself**; check the space around the robot) |
| 4 | | When the stop message appears, **wait about 10 s** (no manual stop command is needed) |
| 5 | **Laptop** L1 | `python3 surface_profiling/test/check_imu_dynamics.py static --heading 90 --duration 30` |
| 6 | | Repeat 3-5 for `--heading 180` and `--heading 270` |
| 7 | **Laptop** L1 | `python3 surface_profiling/test/check_imu_dynamics.py static --report` |

- `rotate` prints the odom-based rotation (including coasting after the stop; within a few degrees of the target is fine). Use `--deg -90` for clockwise. If the Jetson has no copy of this package's source, run it on the Laptop instead; it publishes `/cmd_vel` on the same `ROS_DOMAIN_ID`.
- Measure all headings at the **same spot** (wheel center fixed). The script additionally discards the first 10 s of each heading (`--settle`) to skip the filter transient after rotation.
- Use `--tag name` to separate sessions when the day or the floor changes.

**Reading the result** (printed by `static`/`--report` once all four headings exist; the numbers below are only an example of the format)

```
[*] 4-heading mean -> imu_mount_correction_rpy_deg: [1.200, -0.350]
    std across headings roll 0.052  pitch 0.031 deg
```

- Put the printed `[roll, pitch]` into `imu_mount_correction_rpy_deg` in `config/params.yaml` (no rebuild with `--symlink-install`; restart the node).
- If the std across headings **exceeds 0.5°**, a warning is printed. It means the value is not a body-fixed bias but contains floor slope, filter drift, nearby magnetic fields or similar. Change the location or repeat after some time and check whether it reproduces. If it does not, leaving the IMU TF unused is the safer choice.
- A bias of several degrees that changes between measurements is beyond a normal mounting error, so find the cause first.

### 3-1. Continuous rotation (sweep) method (recommended)

Instead of stopping at 4 headings, **record while rotating slowly** and fit roll/pitch as `b + a1·cos(yaw) + a2·sin(yaw) + c·t` to get the bias `b`. It has far more samples, needs no settling wait after a turn, and one counter-clockwise plus one clockwise turn also reveals a direction-dependent bias (for example filter lag at a given turn rate). It takes about 2 minutes in total.

| Step | Machine | Command / action |
|---|---|---|
| 1 | **Jetson** J1 | `ros2 launch dae_coverage_floor_flatness real_bringup.launch.py` |
| 2 | **Jetson** J2 | Clear at least 0.5 m around the robot, then: `python3 surface_profiling/test/check_imu_dynamics.py sweep` (one counter-clockwise turn → pause → one clockwise turn, 0.1 rad/s. If the Jetson has no copy of the source, run it on the Laptop) |

```
[leg 0] CCW  turned 343deg  n=1171
    roll  bias -6.012  heading-dependent amplitude 0.322  residual std 0.140  drift +0.073 deg/min
    pitch bias +4.008  heading-dependent amplitude 0.304  residual std 0.143  drift +0.308 deg/min
...
[*] mean -> imu_mount_correction_rpy_deg: [-6.006, 3.998]
    CCW-CW bias difference roll -0.011  pitch +0.021 deg
```

(The numbers are only an example of the format. The actual output is in Korean.)

| Output | Meaning | Judgement |
|---|---|---|
| Bias | Intercept after removing the heading-dependent part (value at the middle of the leg) | The mean of the two legs is the candidate for `imu_mount_correction_rpy_deg` |
| Heading-dependent amplitude | Size of the part that changes with heading | Mostly floor slope. For a rigid tilt the roll and pitch amplitudes are similar |
| Residual std | Scatter left after the fit | IMU noise level. The bias uncertainty is much smaller (shrinks with the square root of the sample count) |
| Drift | Time trend inside a leg | A warning appears above 0.3° within one leg. Check whether it also drifts when stationary with `static --duration 300` |
| CCW-CW difference | Bias difference by rotation direction | Above 0.2° there is an error that depends on the turn rate/direction. Lower `--speed` and measure again |

- The raw log is saved as `imu_sweep_<timestamp>.npz`. The heading angle comes from `/odom` yaw, so wheel slip does not disturb the fit much as long as the turn exceeds one revolution.
- If the IMU value shifts by several degrees between sessions, this method must also be **repeated every session**. Run `sweep` twice in the same session first and check that the bias reproduces within 0.2°.

## 4. Drive verification

Run the same driving pattern twice: once with the **IMU TF off** and once with it **on**.

| Run | `--imu-tf` | What it shows |
|---|---|---|
| A. TF off | `off` | Whether IMU changes match the real tilt seen by the LiDAR |
| B. TF on | `on` | Whether injecting the IMU into TF reduces the residual tilt of the point cloud |

### 4-1. Terminals

| Name | Machine | Role |
|---|---|---|
| J1 | **Jetson** | Robot driver + TF (`real_bringup.launch.py`) |
| J2 | **Jetson** | Nav2 + AMCL |
| J3 | **Jetson** | Velocity commands |
| J4 | **Jetson** | IMU TF switching (static TF in run A, `imu_tilt_broadcaster` in run B) |
| L1 | **Laptop** | Velodyne driver |
| L2 | **Laptop** | Measurement node (`surface_profiling.launch.py`) |
| L3 | **Laptop** | Capture service calls / analysis |
| L4 | **Laptop** | IMU recording (`record`) |

### 4-2. Run A: IMU TF off

`base_footprint` is not in the URDF and is published by this node, so stopping the node breaks TF. Publish the same transform with zero rotation instead.

| Step | Machine | Command / action |
|---|---|---|
| 1 | **Jetson** J1 | `ros2 launch dae_coverage_floor_flatness real_bringup.launch.py` |
| 2 | **Jetson** J4 | `pkill -f imu_tilt_broadcaster` |
| 3 | **Jetson** J4 | `ros2 run tf2_ros static_transform_publisher --z 0.01 --frame-id base_footprint --child-frame-id base_link` (**leave running**) |
| 4 | **Jetson** J3 | Check: `ros2 run tf2_ros tf2_echo base_footprint base_link` → rotation should be 0; Ctrl-C |
| 5 | **Jetson** J2 | `ros2 launch dae_coverage_floor_flatness tb3_waffle_nav2.launch.py use_sim_time:=false` |
| 6 | **Laptop** L1 | `ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py` |
| 7 | **Laptop** L2 | `ros2 launch dae_coverage_floor_flatness surface_profiling.launch.py is_sim:=false` |
| 8 | | Converge the AMCL initial pose in RViz |
| 9 | **Laptop** L4 | `python3 surface_profiling/test/check_imu_dynamics.py record --label tf_off` (records until Ctrl-C) |
| 10 | **Laptop** L3 | Open the capture: `ros2 service call /surface_profiling/start_waypoint_capture std_srvs/srv/Trigger` (open it **before moving** so the acceleration phases are captured; unlike the LiDAR guide, do not wait 1 s) |
| 11 | | Keep the robot still for **10 s** (stationary baseline) |
| 12 | **Jetson** J3 | Forward: `ros2 topic pub --times 375 -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.16}}"` |
| 13 | **Jetson** J3 | Stop: `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}}"`, then **wait 5 s** |
| 14 | **Jetson** J3 | Backward: `ros2 topic pub --times 375 -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.16}}"` → stop (same command as 13) → wait 5 s |
| 15 | | Repeat 12-14 once more (2 sets total) |
| 16 | | After the last stop, **wait 10 s** |
| 17 | **Laptop** L3 | `ros2 service call /surface_profiling/stop_waypoint_capture std_srvs/srv/Trigger` |
| 18 | **Laptop** L3 | `ros2 service call /surface_profiling/stop_collection_success std_srvs/srv/Trigger` (saves the frame log `frames_<timestamp>.npz`) |
| 19 | **Laptop** L4 | Ctrl-C → saves `imu_drive_tf_off_<timestamp>.npz` |

0.16 m/s equals `mission_execution.coverage_speed_limit_mps`, to match the measurement condition. Check the space ahead before running (open-loop commands).

### 4-3. Run B: IMU TF on

Put the bias from section 3 into `params.yaml` first.

| Step | Machine | Command / action |
|---|---|---|
| 1 | **Jetson** J4 | Stop the static TF publisher (step 3 of run A) with Ctrl-C |
| 2 | **Jetson** J4 | `ros2 run dae_coverage_floor_flatness imu_tilt_broadcaster` |
| 3 | **Laptop** L2 | Ctrl-C the measurement node and start it again (`surface_profiling.launch.py is_sim:=false`) so filter state from the previous run does not carry over |
| 4 | | Repeat steps 8-18 of run A the same way, but use `record --label tf_on` in step 9 |

### 4-4. Running the analysis

Run on the **Laptop** L3. For `--imu` the file name inside `~/dae_floor_maps/analytics/imu_check/` is enough, and for `--frames` the file name inside `~/dae_floor_maps/analytics/pointclouds/frames/`.

```bash
python3 surface_profiling/test/check_imu_dynamics.py analyze \
  --imu imu_drive_tf_off_<timestamp>.npz --frames frames_<timestamp>.npz \
  --map ~/dae_floor_maps/maps/grid/map_from_dae.yaml --imu-tf off

python3 surface_profiling/test/check_imu_dynamics.py analyze \
  --imu imu_drive_tf_on_<timestamp>.npz --frames frames_<timestamp>.npz \
  --map ~/dae_floor_maps/maps/grid/map_from_dae.yaml --imu-tf on

# view the plot
xdg-open ~/dae_floor_maps/analytics/imu_check/imu_drive_tf_off_<timestamp>_vs_lidar.png
```

Find the frame file you just collected with `ls -t ~/dae_floor_maps/analytics/pointclouds/frames | head -1`. With `--map`, points within 0.4 m of a wall are excluded (recommended).

## 5. Reading the result

```
===== pitch =====
  correlation (lag 0): +0.74   best lag +0.20s gives +0.98   slope L~I: +1.00
  IMU std 0.267 deg   lidar-fit std 0.289 deg   lidar-fit noise on still frames std 0.291 deg
  mean change per phase (relative to still, deg):   IMU / lidar
    accel   +0.043 / +0.307
    ...
```

(The numbers are only an example of the output format. The actual output is in Korean.)

| Output | Meaning | Judgement |
|---|---|---|
| Phase counts | Number of IMU samples / frames in stop, accel, cruise, brake | If `accel`/`brake` frames are near 0, acceleration was not verified. Lower `--acc-thr` or command a sharper speed step |
| Correlation, best lag | Correlation between IMU tilt and LiDAR-fit tilt; lag is the best match found over -1 to +1 s | Positive means the IMU is later than the LiDAR. Beyond ±0.3 s, IMU delay and clock offset cannot be separated |
| Slope `L~I` | Slope of LiDAR tilt regressed on IMU tilt | See the table below. The sign may flip with the coordinate convention, so use the absolute value |
| LiDAR-fit noise on still frames | Scatter of the per-frame plane fit while stationary | Baseline for the smallest tilt that can be detected |
| Mean change per phase | Tilt change in accel/cruise/brake relative to still (IMU / LiDAR) | In run A, if only the IMU changes and the LiDAR does not, it is an IMU artifact (linear acceleration leaking into the attitude estimate) |
| Interpretation | Automatic verdict from the criteria above | |

| `--imu-tf` | Result | Meaning |
|---|---|---|
| `off` | High correlation, \|slope\| ≈ 1 | The IMU follows the real chassis tilt → injecting it is reasonable |
| `off` | Low correlation, IMU std < still noise | **Undecidable** (dynamic tilt is below the detection limit). This is not "no problem" |
| `off` | Low correlation, IMU std > still noise | IMU artifact, or a magnitude the LiDAR cannot capture |
| `on` | \|slope\| ≈ 0 | Residual is uncorrelated with the IMU → compensation works |
| `on` | \|slope\| ≈ 2 | Sign inverted or over-compensation suspected → worse with the TF on |

As a supplement, compare the `lidar-fit std` of runs A and B. If B is smaller, the IMU TF reduces the point-cloud tilt.

## 6. Limitations

- The per-frame plane fit from the LiDAR has noise of a few tenths of a degree even when stationary, so dynamic tilt smaller than that cannot be detected by this method. Read such a result as "undecidable".
- If the controller's acceleration is small, few frames may cross the phase threshold (`--acc-thr`, default 0.03 m/s²).
- Local floor slope that changes along the path appears equally in the IMU and the LiDAR, so it does not affect the correlation, but it does mix into the comparison of phase means against the stationary baseline.
- Only roll/pitch are verified; the height (z) offset is out of scope.

## 7. Verifying yaw with the 2D LiDAR (scan matching)

The 2D LiDAR (`/scan`) is, to first order, insensitive to chassis roll/pitch on vertical walls (the range changes only with the square of the angle), so it **cannot serve as a tilt reference**; the 3D LiDAR plane fit in section 4 does that job. The rotation between two scans (ICP), however, is an **independent reference for yaw change**. It is compared with the IMU gyro z integral, the IMU orientation yaw and the wheel-odometry yaw over the same interval to obtain a **scale error (%) and a bias (deg/min)**. Yaw error propagates through odometry into path skew.

Script: `surface_profiling/test/check_imu_yaw_scan.py` (subcommands `wiggle`, `record`, `analyze`). As in section 3, only Jetson J1 (`real_bringup.launch.py`, which includes the LDS driver) is needed; run the commands below on the **Laptop** L1 (or the Jetson).

| Purpose | Step | Command / action |
|---|---|---|
| Stationary drift | 1 | **Jetson** J1: `ros2 launch dae_coverage_floor_flatness real_bringup.launch.py` |
| | 2 | **Laptop** L1: keep the robot still and run `python3 surface_profiling/test/check_imu_yaw_scan.py record --label static`, then Ctrl-C after **3-5 minutes** |
| Many short rotations | 2 | **Laptop** L1: clear at least 1 m around the robot and run `python3 surface_profiling/test/check_imu_yaw_scan.py wiggle --label room` (±25°, ±50°, ±90° turns at 0.2/0.4/0.7 rad/s, 18 turns, about 90 s, net rotation 0) |
| While driving | 2 | **Laptop** L1: start `record --label drive`, run the forward/stop/backward commands of section 4-2 on **Jetson** J3, then Ctrl-C |
| Analysis | 3 | **Laptop** L1: `python3 surface_profiling/test/check_imu_yaw_scan.py analyze scan_imu_<label>_<timestamp>.npz` |

- Recordings go to `~/dae_floor_maps/analytics/imu_check/`; the analysis writes a `*_yaw.png` there (scatter of scan-matched yaw change against each source's yaw change).
- Angles, rates and directions are mixed to check whether the scale error depends on speed or direction. Repeating `wiggle` at different positions and headings in the room and seeing whether the result reproduces is the evidence for generalization.

**Featureless spaces**: every scan pair is checked for (point count, matching residual, consistency when the initial yaw is shifted by ±3°, deviation from the odometry initial guess) and rejected if it fails. The first output line shows the accepted pair count and the rejection reasons; a warning appears below 30% accepted, and no result is produced below 30 pairs. Measure where walls and furniture surround the robot within 3 m (the LDS-01 maximum range is 3.5 m).

```
Regression  dsrc = s*dscan + b*dt   (dscan: scan-matched yaw change, the reference)
  gyro z integral      scale error +2.34%  bias +12.215 deg/min  residual std 0.164 deg  (n=257)
  ...
```

(Only an example of the format; the actual output is in Korean.)

| Output | Interpretation |
|---|---|
| Scale error | By how many % the source mis-measures the real rotation. Within ±2% can be ignored |
| Bias | How fast yaw drifts while stationary. Above 0.5 deg/min the path skews on long drives |
| Per-rate table | Drift for the stationary bin, per-direction scale for rotation bins. If they differ strongly by rate or direction, a single correction constant is not enough |

**Limitations**
- The 2D LiDAR yaw is a reference, not ground truth. Scan matching itself has an error of a few tenths of a degree, so the scale in the slow-rotation bin (1-8 deg/s) is uncertain because the change per pair is small.
- Moving objects near the robot disturb the matching.
- This verifies yaw only. It is independent of the accuracy of the roll/pitch that go into the IMU TF.
