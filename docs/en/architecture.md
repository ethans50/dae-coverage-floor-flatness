# Code Architecture

[English](architecture.md) · [한국어](../kr/architecture.md) · [← README](../../README.md)

This page maps every tracked file to the stage of the pipeline it belongs to, and lists what each stage reads and writes. For the reasons behind the algorithmic choices see [Design choices](../../README.md#design-choices) in the README; for parameters see the [configuration guide](configuration.md).

---

## 1. Pipeline and where each stage runs

```text
 3D design file (.dae)
        │
        ▼
 [1] environment_modeling ──► 2D map, node decomposition           (workstation, offline)
        │
        ▼
 [2] mission_planning ──────► final_path.json (+ metadata sidecar) (workstation, offline)
        │
        ▼
 [3] mission_execution ─────► drives the path with Nav2 / AMCL     (robot, online)
        │  capture start / stop services
        ▼
 [4] surface_profiling ─────► point clouds, frame log, heatmap     (LiDAR laptop, online)
```

| Stage | Package directory | Entry point | Runs on |
|---|---|---|---|
| 1 + 2 | `mission_generation/` | `python3 mission_generation/run_generation_pipeline.py` | workstation |
| 3 | `mission_execution/` | `mission_executor` (`mission_execution.launch.py`) | robot (Jetson) or simulation |
| 4 | `surface_profiling/` | `surface_profiler` (`surface_profiling.launch.py`) | LiDAR laptop or simulation |
| support | `launch/`, `config/`, `models/`, `urdf/`, `worlds/`, `scripts/` | — | all |

`mission_executor`, `imu_tilt_broadcaster` and `surface_profiler` are the three console scripts registered in `setup.py`. Stage 3 and stage 4 communicate only through ROS: stage 3 opens and closes capture windows with the `/surface_profiling/{start_waypoint_capture,stop_waypoint_capture,stop_collection_success}` services, and stage 4 reads the transform tree and `/velodyne_points`.

## 2. Files each stage reads and writes

All data lives outside the package, under `workspace_root` in `config/params.yaml` (default `~/dae_floor_maps`). Directory names below are the defaults of the matching keys in the same file.

| Stage | Reads | Writes |
|---|---|---|
| Environment modeling | `assets/<dae_file>` | `maps/grid/map_from_dae.{pgm,yaml}`, `maps/grid/map_preprocessed_data.npz`, `maps/topology/final_topological_map.npz`, debug images under `visualization/` |
| Mission planning | topology map, 2D map, `params.yaml` | `analytics/metrics/final_path.json`, `analytics/metrics/final_path_meta.json`, debug images, and a dated copy under `analytics/metrics/archive/<timestamp>_ring<N>_<hash>/` (path, sidecar, raw path, the `params.yaml` used, and `config_summary.txt` listing the ring count and toggles) |
| Mission execution | `final_path.json`, `final_path_meta.json`, 2D map | `analytics/paths/robot_path_<ts>.csv`, `analytics/logs/drive_debug_<ts>.csv`, `analytics/logs/stall_report_<ts>.csv`, plots under `visualization/mission_execution/` |
| Surface profiling | `/velodyne_points`, TF, capture services | `analytics/pointclouds/combined_<ts>.pcd`, `combined_raw_<ts>.pcd`, `combined_filtered_<ts>.pcd`, `frames_<ts>.npz`, `floor_heatmap_<ts>.png`, optional accumulation video |

`final_path_meta.json` records the parameters the path was planned with. The executor compares the execution-relevant ones (robot width, wall margin, boundary-repass settings, LiDAR mount height, map resolution) with the current `params.yaml` and refuses to run on a mismatch; the remaining entries only record which optimisation toggles produced the path. The cached file `map_preprocessed_data.npz` is reused when present, so delete it when you change `dae_file`.

When `eval_run_label` is passed to the launches, the outputs of stages 3 and 4 are written under `eval_runs/<label>/` instead, and the executor reads the path snapshot stored there. `run_generation_pipeline.py --snapshot-label <label>` creates that snapshot.

---

## 3. Top-level files

| File | Purpose |
|---|---|
| `package.xml`, `setup.py`, `setup.cfg`, `resource/dae-coverage-floor-flatness` | ROS 2 (ament_python) package definition; `setup.py` installs launch files, configs, behavior trees, URDF, worlds and models into the share directory and registers the console scripts. |
| `requirements-mission_generation.txt`, `requirements-surface_profiling.txt` | Python dependencies for the workstation and for the LiDAR laptop. |
| `README.md`, `README.kr.md`, `CITATION.cff`, `LICENSES/Apache-2.0.txt` | Overview, citation metadata, license text for the files derived from ROBOTIS / Open Robotics. |
| `config/params.yaml` | All algorithm, execution and measurement parameters, one section per stage. |
| `config/tb3_waffle_nav2_params.yaml` | Nav2 stack: costmaps (footprint polygon, inflation), controllers, behavior-tree paths. |
| `behavior_trees/navigate_through_poses_transit.xml`, `navigate_to_pose_transit.xml` | Behavior trees for non-measuring (transit) segments; they select the Regulated Pure Pursuit controller. |
| `models/INU_9_211/`, `models/Apt/` | Building models (`.dae` mesh + `model.sdf`) used both as pipeline input and as the simulated building. The model follows `dae_file`. |
| `urdf/turtlebot3_waffle.urdf.xacro`, `turtlebot3_waffle.gazebo.xacro` | Robot description and Gazebo plugins (vendored from TurtleBot3). |
| `worlds/coverage_flatness_env.world.xacro` | Simulation world template, expanded with the selected building model. |
| `rviz/tb3_navigation2.rviz` | RViz layout for Nav2. |
| `scripts/stack.py`, `scripts/stop_all.sh` | Start the robot-side and laptop-side stack in order with checks between stages ([guide](stack.md)); stop all related processes in stages. |

## 4. `launch/`

| File | Purpose |
|---|---|
| `sim_env.launch.py` | Whole simulation environment: Gazebo world, robot spawn, robot state publisher, `imu_tilt_broadcaster`. Arguments: `world`, `x_pose`, `y_pose`, `use_imu_tilt`, `use_sim_time`. |
| `sim_robot_state_publisher.launch.py`, `sim_spawn_robot.launch.py` | Simulation robot description and spawning (spawn file adapted from TurtleBot3). |
| `real_bringup.launch.py`, `real_robot_state_publisher.launch.py` | Real-robot bring-up and robot description. |
| `tb3_waffle_nav2.launch.py` | Nav2, AMCL and the map server (adapted from TurtleBot3). |
| `mission_execution.launch.py` | `mission_executor` node; accepts `is_sim` and `eval_run_label`. |
| `surface_profiling.launch.py` | `surface_profiler` node; accepts `is_sim` and `eval_run_label`. |

---

## 5. `mission_generation/` (offline)

### 5.1 Entry point

| File | Purpose |
|---|---|
| `run_generation_pipeline.py` | Runs environment modeling, then mission planning. Asks whether to rebuild the map and whether to regenerate the path when earlier outputs exist. `--snapshot-label` copies the maps, topology, path and debug images into `eval_runs/<label>/` for experiment bookkeeping. |

### 5.2 `environment_modeling/`

| File | Purpose |
|---|---|
| `environment_modeler.py` | Orchestrates the stage: 2D map generation, preprocessing, segmentation, saving the topology. |
| `map_generator.py` | Slices the `.dae` obstacle triangles into a 2D occupancy grid at `target_resolution`. |
| `map_preprocessor.py` | Removes the exterior and enclosed non-floor regions and keeps the largest connected floor area. |
| `space_segmenter.py` | Runs the five segmentation steps below and saves `final_topological_map.npz`. |
| `algorithms/limits.py` | Step 1: splits the floor into traversable and non-traversable (but still to be sensed) areas from the robot radius. |
| `algorithms/doors.py` | Step 2: splits spaces at doors and decides which doors the robot can pass. |
| `algorithms/holes.py` | Step 3: detects holes (pillars, enclosed obstacles) and cuts nodes around them. |
| `algorithms/convexity.py` | Step 4: recursively splits L- and U-shaped spaces by solidity. |
| `algorithms/subdivider.py` | Step 5: subdivides nodes whose aspect ratio is too large. |
| `utils/visualizer.py` | Debug images for each segmentation step. |
| `__main__.py` | Standalone runner for this stage. |

### 5.3 `mission_planning/`

| File | Purpose |
|---|---|
| `mission_planner.py` | Orchestrates planning: node visiting order, per-node coverage, transit paths, boundary-repass preview, path assembly. |
| `algorithms/tsp.py` | Connection waypoints between adjacent nodes, the planning mask that lets A\* cross narrow connections, and the node visiting order (approximate travelling-salesman tour over a graph of connection costs, using NetworkX). The planner sorts nodes by centroid before solving, so the order does not depend on how nodes are stored in the topology file. |
| `algorithms/pendant_reorder.py` | Reorders the visits to leaf nodes attached to a hub, based on where the hub's coverage actually ends. |
| `algorithms/coverage.py` | Converts a node mask into Fields2Cover cells and generates the swaths, optionally after boustrophedon decomposition and with a search over swath angles (`enable_optimal_swath_angle`). |
| `algorithms/transit.py` | A\* between nodes with turn and wall-proximity penalties. |
| `utils/sampler.py` | Resamples the path at a fixed spacing and adds the helper waypoint before each corner of a measured segment. |
| `utils/coverage_check.py` | Predicts, from the planned path alone, which floor cells will get too few LiDAR ring hits; used to decide which transits to measure. |
| `utils/ring_coverage.py` | VLP-16 ring geometry used by the prediction and by the repass decision. |
| `utils/repass_preview.py` | Approximates, in pixel space, the points that boundary repass will add. |
| `utils/geometry.py` | Centroids, distance matrices and other geometric helpers. |
| `utils/visualizer.py` | Debug images of the planned path. |
| `translator.py` | Converts pixel paths into metric Nav2 poses (position and heading). |
| `path_exporter.py` | Resamples, writes `raw_path.json`, `final_path.json` and `final_path_meta.json`, and overlays waypoints on the debug images. |
| `__main__.py` | Standalone runner for this stage. |

### 5.4 `test/`

| File | Purpose |
|---|---|
| `check_executed_paths.py` | Self-check of the corner-cut model in `utils/coverage_check.py`. |

---

## 6. `mission_execution/` (online)

`MissionExecutor` is a ROS node assembled from three mixins plus helpers in `utils/`.

| File | Purpose |
|---|---|
| `mission_executor.py` | The node: loads the path, splits it into straight sub-segments (a new one starts where the heading changes by more than `direction_change_threshold_deg` or where the capture flag changes), runs each one, and opens and closes capture windows. A corner that reverses direction sharply (`subsegment_merge_max_turn_deg`) is sent as its own goal. |
| `localization_mixin.py` | AMCL initialisation with position and heading checks, jump detection, `map→base_link` lookup. TF is received on a dedicated thread. |
| `nav2_drive_mixin.py` | Nav2 action wrappers (`NavigateToPose`, `NavigateThroughPoses`, spin, back-up) and escalation when a stall persists. |
| `run_context_mixin.py` | Reads the path, metadata and parameters; saves the driven path, stall report and plots at the end of a run. |
| `imu_tilt_broadcaster.py` | Publishes `base_footprint→base_link`. With `use_imu_tilt:=true` it carries IMU roll and pitch minus the configured bias; otherwise a zero rotation. |
| `utils/boundary_repass.py` | Start prepass and exit repass around coverage boundaries, so that the blind zone of the LiDAR near a boundary is measured on the way out. |
| `utils/controller_switch.py` | Switches the Nav2 controller settings (rotation threshold, speed limit, behavior tree) between coverage and transit segments. |
| `utils/ring_coverage.py` | Decides whether the segment after a coverage exit already has enough ring hits, so a repass can be skipped. |
| `utils/mission_logger.py` | Periodic record of where the robot is and what it is doing (`drive_debug_<ts>.csv`). |
| `utils/stall_logger.py` | Detects stalls from Nav2 feedback and writes `stall_report_<ts>.csv`. |
| `utils/nav2_utils.py` | Patches `BasicNavigator` initial-pose publishing (live timestamp, throttled re-publish). |
| `utils/ros_utils.py` | Builds `PoseStamped` messages clamped to the map; teleports the robot in Gazebo (simulation start). |
| `utils/map_utils.py` | Physical bounds of the map from the YAML and PGM. |
| `utils/visualizer.py` | Plots the planned path's wall proximity and the stall locations. |

---

## 7. `surface_profiling/` (online, plus analysis tools)

### 7.1 Collection node

| File | Purpose |
|---|---|
| `surface_profiler.py` | The node: collects point clouds while a capture window is open, then saves the combined clouds, extracts the floor and writes the heatmap. |
| `tf_sync_mixin.py` | Waits for the transform at each cloud's stamp, transforms to the `map` frame and rejects frames whose instantaneous velocity is too high. |
| `capture_services_mixin.py` | The capture start / stop / finish services that `mission_executor` calls. |
| `utils/frame_recorder.py` | Per-frame log (`frames_<ts>.npz`): points, intensity, ring, pose and flags. |
| `utils/floor_extractor.py` | Keeps points inside the floor-height band of a PCD. |
| `utils/heatmap_generator.py` | Colours the mean z per grid cell on a fixed scale and overlays it on the 2D map. |
| `utils/pcd_io.py` | PCD → CSV export. |
| `utils/config_paths.py` | Resolves the directory settings of `params.yaml` into absolute paths; the profiler and the reprocessing script resolve paths through it. |
| `utils/frame_video.py` | Builds the top-down accumulation video from a frame log. |

### 7.2 Post-processing and analysis

| File | Purpose |
|---|---|
| `local_plane_heatmap.py` | Heatmap of deviation from a local plane computed from the frame log, with comparison metrics ([guide](local_plane_heatmap.md)). |
| `reprocess_pcd.py` | Re-runs floor extraction and heatmap generation on a stored `combined_*.pcd` with different parameters, without driving again. |
| `analyze_coverage_comparison.py` | Per-run metrics for comparing path variants: completeness, gaps, distance, time, rotation, net driving time. |
| `analyze_frame_log.py` | Per cell, point count and number of distinct passes, to assess what repass adds. |
| `analyze_ring_coverage.py` | How often each LiDAR ring grazes each cell along the driven trajectory. |
| `analyze_min_detectable_height.py` | Smallest detectable bump height per bump diameter and estimator, from synthetic bumps added to a frame log. |
| `analyze_z_bias.py` | Separates systematic floor-height error (sensor tilt) from random noise. |
| `fit_heading_z_bias.py` | Estimates residual IMU pitch / roll bias from a frame log ([IMU calibration](imu_calibration.md)). |
| `auto_calibration_drive.py`, `scan_room_for_calibration.py` | Four-heading LiDAR mount calibration data collection ([LiDAR calibration](lidar_calibration.md)). |
| `make_accumulation_video.py` | Re-creates the accumulation video from a frame log. |
| `view_pcd.py` | Shows a PCD point by point, coloured by z. |
| `utils/coverage_metrics.py` | Pure functions behind the coverage metrics. |
| `utils/ring_coverage.py` | Ring-graze counts per floor cell. |
| `utils/local_plane.py` | Residual from a local plane. |
| `utils/floor_adjustment.py` | Joint estimate of the floor height field and per-frame planes from overlapping observations. |
| `utils/wall_alignment.py` | Estimates the heading error from the front wall in a raw cloud. |
| `utils/stall_report_analyzer.py` | Analyses and plots stall reports against the driven path. |
| `utils/ssh_robot.py` | Thin SSH wrapper for sending commands (such as `cmd_vel`) to the Jetson. |

### 7.3 `test/`

Diagnostic scripts that check a module against synthetic data with a known answer, or check recorded data for consistency. They are run by hand (`python3 surface_profiling/test/<file>.py`).

| File | Checks |
|---|---|
| `check_local_plane.py`, `check_floor_adjustment.py`, `test_wall_alignment.py` | `utils/local_plane.py`, `utils/floor_adjustment.py`, `utils/wall_alignment.py` against synthetic floors and bumps. |
| `check_frame_recorder.py` | That intensity and ring stay aligned with the points in the frame log. |
| `check_chassis_state.py` | The effect of the chassis state while crossing a bump on each post-processing mode. |
| `check_map_alignment.py` | That the point clouds in a frame log sit correctly on the 2D map. |
| `check_imu_noise.py`, `check_imu_dynamics.py`, `check_imu_yaw_scan.py` | IMU noise, roll / pitch bias at rest and while driving, and yaw error against 2D scan matching. |

---

## 8. Where to extend

- **A different building**: put `<name>.dae` in `<workspace_root>/assets/` for the pipeline, add `models/<name>/` (`meshes/<name>.dae`, `model.sdf`, `model.config`) for the simulation, set `dae_file` in `params.yaml`, run `colcon build`, delete `maps/grid/map_preprocessed_data.npz`, and re-run `run_generation_pipeline.py`.
- **A new path optimisation toggle**: add the key under `mission_planner` in `params.yaml`, read it in `mission_planner.py`, and add it to the entries written by `path_exporter.py` so that `final_path_meta.json` records it.
- **A different measurement window or grid**: `surface_profiling` section of `params.yaml`; `reprocess_pcd.py` previews the effect on stored data.
