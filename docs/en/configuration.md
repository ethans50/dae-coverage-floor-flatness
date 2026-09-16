# Configuration Guide

[English](configuration.md) · [한국어](../kr/configuration.md) · [← README](../../README.md)

All pipeline settings live in a single file, [`config/params.yaml`](../../config/params.yaml), grouped into one section per stage. After editing it, run `colcon build` to update the install tree.

---

## Configuration reference

| Key | Section | Meaning |
|---|---|---|
| `dae_file` | `environment_modeling` | Which model to process. Change `worlds/coverage_flatness_env.world` to match if you simulate. |
| `target_resolution` | `environment_modeling` | Map resolution, m/px. Smaller is finer and slower. |
| `max_door_radius` | `environment_modeling` | Half the widest opening still treated as a door. |
| `max_aspect_ratio` | `environment_modeling` | Above this, a node is subdivided (Step 5). |
| `robot_width` | `environment_modeling` | Robot **radius** (m). Used for the traversable area and the planned path's wall clearance. |
| `path_safety_margin` | `mission_planner` | Additional clearance the planned path keeps from walls on top of `robot_width`. |
| `lidar_range`, `overlap`, `lidar_vertical_fov_deg`, `lidar_mount_height` | `mission_planner` | Sensor geometry; determines swath width and the blind radius that drives node bucketing. |
| `turn_weight`, `wall_weight` | `mission_planner` | A\* transit penalties. |
| `coverage_speed_limit_mps` / `transit_speed_limit_mps` | `mission_execution` | Runtime speed per segment type. |
| `direction_change_threshold_deg` | `mission_execution` | Heading change that starts a new straight sub-segment. |
| `boundary_repass_distance_m` | `mission_execution` | Retrace distance at coverage boundaries; must exceed twice the blind radius. |
| `z_min`, `z_max` | `surface_profiling` | Floor extraction window (m). Must span both sides of the design floor level. |
| `grid_size` | `surface_profiling` | Analysis cell size for completeness and the heatmap. |
| `save_raw_pcd`, `save_combined_csv`, `save_waypoint_pcd` | `surface_profiling` | Optional bulky artefacts, off by default. |

Hardware-coupled constants (`lidar_mount_height`, `robot_width`, `boundary_repass_distance_m`) **must be re-measured** if the sensor is remounted or the robot is replaced.

## Configuration pitfalls

Values in [`config/params.yaml`](../../config/params.yaml) that are easy to get wrong because of their name or format. Entries whose error column reads **None** are the ones to watch most closely: a wrong value lets the run finish normally and only changes the result.

| Key | What to watch for | Error on a wrong value |
|---|---|---|
| `robot_width` | Despite the name, this is a **radius** (m). Entering the full chassis width doubles the wall clearance and can drop narrow areas from the drivable region. | None |
| `robot_width`, `path_safety_margin` | Set only the **planned path's** wall clearance. Nav2's costmap `footprint` and `inflation_radius` are configured separately in [`config/tb3_waffle_nav2_params.yaml`](../../config/tb3_waffle_nav2_params.yaml). | None |
| `robot_width`, `path_safety_margin`, `boundary_repass_distance_m`, `enable_boundary_repass` | The path coordinates depend on these. Changing them after planning makes the executor refuse to start; re-run path generation after any change. | Refuses to start |
| `enable_boundary_repass` | Lives in the `mission_execution` section but is **also read by path generation**. | Refuses to start |
| The five `enable_*` toggles | `false` is an ablation condition. If not restored to `true` after an experiment, every later path is generated without that optimization. | None |
| `coverage_mode` | Accepts only the strings `"full"` / `"centroid_only"` (anything else raises during path generation). `centroid_only` is the evaluation baseline; use `"full"` for real inspections. | Raises an exception |
| `only_capture_at_waypoints` | Despite the name, this does not mean "measure only while stopped". `true` keeps **only points inside capture windows (coverage driving + boundary repass)**; continuous capture while driving is unchanged. `false` also mixes in points from transit between nodes. | None |
| `z_min`, `z_max` | Units are **metres**, not millimetres. The window must include **both sides** of the design floor level (z = 0) — `z_min: 0.0` discards the lower half of the floor points. | None |
| `voxel_size`, `grid_size` | `voxel_size` is the downsampling pitch at save time; `grid_size` is the analysis cell for completeness and the heatmap. A `voxel_size` larger than `grid_size` leaves cells that can never receive a point, lowering completeness. | None |
| `boundary_repass_distance_m` | Must be **at least twice** the blind radius. It is clamped automatically when a segment is shorter, but a value that is too small leaves the boundary region unfilled. | None |
| `lidar_mount_height` | Enter the measured value. The blind radius and node width classification are derived from it. | None |
| `dae_file` | For simulation, `worlds/coverage_flatness_env.world` must reference the same model. | None |
| `save_raw_pcd` | Enable only for the per-cell z standard deviation (noise) metric; it substantially increases per-run storage. | — |
| `is_sim` (launch argument) | Defaults to `false`, and `use_sim_time` follows it. Omitting it in simulation makes the nodes use the system clock instead of Gazebo's. Give the measurement node and the executor the same value. | None |

After editing `params.yaml`, run `colcon build` to update the install tree and launch from a new terminal. `reprocess_pcd.py` prints the values actually in effect when it starts.

For the configuration, topics and networking (DDS) of ROS 2, Nav2 and the Velodyne driver themselves, refer to the upstream documentation — [ROS 2 Humble](https://docs.ros.org/en/humble/), [Nav2](https://docs.nav2.org/), [Velodyne ROS 2 driver](https://github.com/ros-drivers/velodyne/tree/humble-devel). Installation issues are covered in the [installation guide](installation.md#troubleshooting).

---

[← README](../../README.md)
