# How the Heatmap Is Generated

This document describes, stage by stage, how `surface_profiler` turns LiDAR point clouds into the flatness heatmap (`floor_heatmap_<ts>.png`). The parameters are under `surface_profiling` in `config/params.yaml`; defaults are given in parentheses. Each stage also states **why** it is set that way, marking the kind of evidence (observation, design judgement, empirical value) and what was not verified. It covers the default heatmap produced during a run; the post-processing that recomputes it as deviation from a local plane is in the [local-plane heatmap guide](local_plane_heatmap.md).

## 1. Overview

```text
/velodyne_points ──► (1) map-frame transform ──► (2) frame rejection ──► (3) capture-window selection
                                                                                 │
floor_heatmap_<ts>.png ◄── (6) cell mean and colour mapping ◄── (5) z window ◄── (4) accumulate, downsample
                                                                                 │
                                                 frames_<ts>.npz ◄── (per-frame log, saved in parallel)
```

| Stage | What it does | Code |
|---|---|---|
| 1. Transform | transforms the cloud from the sensor frame to the `map` frame | `surface_profiling/tf_sync_mixin.py` (`_wait_and_process`, `_pc_callback`) |
| 2. Frame rejection | drops a frame when the instantaneous velocity between consecutive frames exceeds a limit | same file (`_tf_passes_lowpass_filter`) |
| 3. Capture window | uses only points received while a capture window is open (coverage driving and boundary repass) | `surface_profiling/capture_services_mixin.py` |
| 4. Accumulate, downsample | stacks the points, reduces them to voxel centroids and saves `combined_<ts>.pcd` | `surface_profiling/surface_profiler.py` (`_save_combined_pcd`) |
| 5. z window | keeps points in the floor-height band and saves `combined_filtered_<ts>.pcd` | `surface_profiling/utils/floor_extractor.py` |
| 6. Cell mean, colour | draws the mean z of each grid cell as colour and saves the PNG | `surface_profiling/utils/heatmap_generator.py` (`generate_floor_heatmap`) |
| (parallel) frame log | stores per-frame pose, points and intensity in `frames_<ts>.npz` | `surface_profiling/utils/frame_recorder.py` |

`run()` in `surface_profiling/surface_profiler.py` sequences the stages and file saving (collection → floor extraction → visualisation). `surface_profiling/reprocess_pcd.py` reruns stages 5 and 6 from a single stored `combined_<ts>.pcd`.

## 2. Stages and rationale

Evidence labels: **[Observed]** recorded data or a reproduced event, **[Design judgement]** reasoning that was not confirmed by measurement, **[Empirical value]** a value chosen without deriving it from a model (the effect of changing it was not tested).

### 2.1 Transform to the map frame

- The subscribed topic is `/velodyne_points` (VLP-16, 600 rpm = 10 Hz). NaN points are discarded.
- The `map → velodyne_link` transform is taken at the **message stamp**. Because AMCL's `map → odom` updates more slowly than the clouds, a message is queued until a TF covering its stamp has arrived, and processed then.
- The transform matrix is `(TF) × (LiDAR mount correction)`. The mount correction is `lidar_mount_correction_rpy_deg_{sim,real}` ([roll, pitch, yaw], deg), applied to the sensor frame as an extra rotation.
- Each point becomes `map` coordinates `(x, y, z)` through a homogeneous transform. `z` is therefore the height above the `map` plane, so heatmap colour is the **absolute height in the map frame**, not a relative height.

Rationale:

- **TF at the stamp** — [Observed] AMCL's `map → odom` updates at about 5 Hz, the clouds at 10 Hz. [Design judgement] Using the latest TF would apply a pose that is offset in time from the cloud by up to 0.2 s; in that time the robot moves 3 cm at 0.16 m/s or turns 4° at 20 deg/s. The flatness signal is below a centimetre, so the stamp is matched. The effect of the offset on results was not measured separately.
- **Mount correction** — [Observed] On a flat floor, z tilts to one side in proportion to range from the sensor; this is explained by the sensor being mounted differently from the URDF pose. The correction values are estimated from a flat-floor recording by `surface_profiling/analyze_z_bias.py` (procedure in the [LiDAR calibration guide](lidar_calibration.md)). Limitation: the estimate assumes the recorded floor is flat, so a real gentle slope can leak into the correction.

### 2.2 Frame rejection

From the TF of two consecutive clouds, the instantaneous linear and angular velocities are computed; if either exceeds `tf_lowpass_max_linear_vel` (0.3 m/s) or `tf_lowpass_max_angular_vel_deg` (20 deg/s), the whole frame is dropped. The reference frame is updated only on frames that pass and is reset when a new capture starts. Rejected frames still keep their pose and rejection flag in the frame log.

The rationale is long, so it is collected in [section 3](#3-basis-of-the-frame-rejection-thresholds).

### 2.3 Capture-window selection

With `only_capture_at_waypoints: true` (default), only points received while `mission_executor` keeps the capture window open enter the result. Despite the name, this is not "measure while stopped".

- Coverage sub-segments are captured **continuously while driving** at or below `coverage_speed_limit_mps` (0.16 m/s).
- The window opens when a node's coverage starts, stays open through corners inside the node, and closes after the boundary repass (driving back over the boundary).
- A single-point sub-segment has no motion, so it dwells for `active_capture_seconds` (2.0 s) while capturing.
- Transit between nodes (`transit_speed_limit_mps`, 0.26 m/s) is not captured.
- With `false`, all segments including transit are collected continuously.

Rationale:

- **Continuous capture while driving** — [Design judgement] The path is driven continuously along swaths, so stop-and-measure would need many stops along each swath and does not fit the coverage path. In one recorded run (1,602 captured frames), about 80% of captured frames had a linear velocity of 0.1–0.2 m/s and only one exceeded 0.3 m/s.
- **Transit excluded** — [Design judgement] The transit speed (0.26 m/s) is above the coverage speed, outside the speed range whose measurement quality was checked, and the movement between nodes is not a measurement target.
- **0.16 m/s** — [Empirical value] It keeps the speed at which measurement precision was previously checked. In the recording above, the median frame mean offset of passing frames by linear velocity is 0.90 mm at 0–0.02 m/s, 1.34 mm at 0.1–0.2 m/s and 1.27 mm at 0.2–0.3 m/s (same definition as the table in section 3.1), so no clear relation between linear velocity and offset is visible between 0.1 and 0.3 m/s. This data therefore gives no direct basis for the 0.16 limit.
- **2.0 s dwell** — [Empirical value] About 20 frames at 10 Hz; the effect of changing it was not tested.

### 2.4 Accumulate and downsample

All captured points are stacked and downsampled on a `voxel_size` (0.01 m) grid. Each occupied voxel is replaced by **one centroid** of the points in it, so the later cell mean averages the centroids of occupied voxels rather than the original returns. The result is `combined_<ts>.pcd` in the map frame; if the pre-downsample points are needed, `save_raw_pcd: true` also saves `combined_raw_<ts>.pcd`.

Rationale:

- [Design judgement] Stacking every point of a run reaches millions of points and exhausts memory (the purpose given in the configuration comment), so the cloud is reduced before saving.
- [Empirical value] 1 cm matches the heatmap grid (`grid_size`, 0.01 m); the effect of changing it was not tested. Centroid averaging gives a densely hit voxel and a sparsely hit voxel the same weight.

### 2.5 z window

Only points with `z_min ≤ z ≤ z_max` (`-0.025` to `+0.025` m) are kept. This is a height-band cut without plane fitting. Floor points are taken to spread above and below the design height, so the window is symmetric. **Points outside the window are removed, not saturated in colour**, so a defect that rises above the window drops out of its cell's mean.

Rationale:

- **Symmetric window** — [Observed] Floor points spread symmetrically around z = 0 (peak −0.024 to +0.015 m). Processing one recording with a [0, 0.035 m] window cut away the lower half of the points and gave a grid completeness of 71.3% inside nodes; changing it to [−0.020, +0.020 m] gave 94.65%. Cutting the lower side biases both completeness and the mean upward.
- **±2.5 cm** — [Observed] This width includes the lower end of the floor peak (−0.024 m). [Design judgement] The higher the upper limit, the more points from the lower part of walls and objects mix into near-wall cells, so it is not widened more than needed.
- Limitation: a defect larger than the window width is clipped, so its height reads low or its cell becomes empty; interpret defect size only within the window width.

### 2.6 Cell mean and colour mapping

`generate_floor_heatmap` computes the following from the filtered points.

1. Divide the points into a `grid_size` (0.01 m) grid starting at the minimum `x` and `y`.
2. Cell value = mean of `z` of the points in the cell (`Σz / count`). Cells with no points are NaN (empty).
3. Clip the cell value to `[z_min, z_max]`, convert to cm and paint it with the `jet` colormap. The colour range is **fixed** to `z_min`–`z_max`, not to the data min/max, so the colour-bar ticks are the actual z in cm. An extreme value such as a wall base does not rescale the colours.
4. If a map image (`map_from_dae.yaml`) exists it is drawn as the background and the heatmap is overlaid with opacity 0.75. Empty cells are fully transparent.
5. Save as a 300 dpi PNG (`visualization/surface_profiling/floor_heatmap_<ts>.png`).

Rationale:

- **Cell mean** — [Design judgement] Each cell is passed over several times, and averaging reduces per-point noise. The mean does not distinguish observation range or direction, so the cautions in section 6 apply.
- **Fixed colour range** — [Design judgement] If the colour scale changed per run, heatmaps from different recordings could not be compared by eye, and one or two extreme points would dominate the scale. With the range equal to the z window, the ticks span exactly what the filter allows.
- **1 cm grid, `jet`, opacity 0.75** — [Empirical value] These are presentation choices without quantitative backing. A smaller grid is sharper but leaves more cells with few points and costs more computation.

## 3. Basis of the frame-rejection thresholds

This collects the rationale for the rejection rule in section 2.2. The conclusion first: the thresholds are **empirical values, not derived from a model**, and the numbers below check after the fact, on one real recording, that the values are not unreasonable. How flatness results (cross-view consistency and so on) change with the thresholds was not tested, and the effect of rejection is small.

### 3.1 Why frames taken during fast rotation or motion blur the comparison

- [Design judgement] A cloud collects the points of the 0.1 s the sensor takes for one revolution, but the transform to the map frame uses a single pose at the stamp. During rotation the pose at the start and end of the scan differ, and that difference becomes error directly. At 20 deg/s the sensor turns 2° in one revolution, and at a floor point 2 m from the sensor a 1° pose error is about 3.5 cm (2 m × tan 1°). This is a calculation, not a measurement.
- [Design judgement] Several frames are averaged and compared per cell, so a frame with a large pose error blurs the cell value and the consistency between frames.
- [Observed] In one real recording (1,602 captured frames), a 5 cm cell reference map was built from the passing frames and each frame's deviation from it was measured; the offset grows with angular rate.

| Angular rate between frames [deg/s] | Frames | Frame mean offset \|e\| median [mm] | p90 [mm] |
|---|---|---|---|
| 0–2 | 488 | 0.94 | 2.83 |
| 2–5 | 404 | 1.42 | 3.32 |
| 5–10 | 381 | 1.59 | 3.46 |
| 10–20 | 250 | 1.59 | 3.70 |
| 20–40 | 51 | 2.38 | 4.88 |
| 40 and above | 24 | 2.65 | 4.31 |

The 76 rejected frames have a median offset of 2.51 mm, about twice the 1.30 mm of the 1,526 passing frames, and a larger median within-frame z standard deviation (6.40 mm against 4.91 mm). The median tilt magnitude of the frame plane is nearly the same (3.04 against 2.73 mrad).

Limits of this evidence:

- The offset difference is around 1 mm, far below the target defect (15 mm). It is not evidence that rejection greatly improves results; it trims a few high-error frames.
- Angular rate cannot be stated as the cause. During rotation, other factors such as AMCL pose-update delay and body wobble after a turn change together with it, and they were not separated.
- The reference map was built from passing frames only, so rejected frames are not in the reference (the reasoning is not circular). It is still one recording, and reproduction on repeated or other environments was not checked.
- No case was observed where a rejected frame produced a visible pattern in a result heatmap. The judgement that such frames "contaminate" rests on the reasoning and statistics above, not on a confirmed individual event.

### 3.2 Why 20 deg/s and 0.3 m/s

The two values differ in nature.

**20 deg/s (angular rate)** — [Empirical value] This value does the actual rejecting. It roughly matches the point in the table above where the offset steps up at 20 deg/s and above, and it discards only 4.7% of captured frames in this recording. The rejection rate for other thresholds on the same recording is (linear velocity fixed at 0.3 m/s):

| Angular threshold [deg/s] | 10 | 20 | 30 | 45 |
|---|---|---|---|---|
| Rejection rate | 32.1% | 4.7% | 3.1% | 1.6% |

Lowering it to 10 deg/s discards a third of the frames, a large loss of observations. Above 20 deg/s the rate falls gently as the threshold rises, so the choice is less sensitive there, but which value inside that range is best is unknown.

**0.3 m/s (linear velocity)** — [Design judgement] It is above the driving speed limits (coverage 0.16, transit 0.26 m/s), so normal driving frames do not trigger it; in the recording above only one captured frame exceeded 0.3 m/s. The value is therefore not a filter for travel speed but a jump detector for sudden position changes (AMCL re-estimation, a teleport in simulation). Lowering it to 0.2 m/s rejects 6.6% and to 0.15 m/s rejects 37.1%, and 0.15 m/s is close to the driving speed (0.16 m/s), so normal driving frames would be discarded.

### 3.3 Why the reference updates only on passing frames and resets at capture start

- [Design judgement] If a spiking frame became the reference, the next normal frame would be computed as a jump back from the spike and rejected in a chain, so the reference is updated only on frames that pass.
- [Observed] Conversely, a reference that is not updated can become stale. In simulation, when the robot is teleported between missions, the first frame of the first waypoint is computed as a large jump from the last position of the previous mission to the new start and is rejected. Because the reference is not updated, the jump distance stays fixed and only dt grows; while the robot was stationary, the computed velocity fell only from 0.85 to 0.71 m/s over about 2.1 s, far above the threshold, all frames were rejected, and the capture window of the first waypoint closed with 0 points (reproduced in 2 of 2 runs). The reference is now cleared when a capture starts, so the next frame becomes the new reference. The first frame after clearing has nothing to compare against and always passes.

### 3.4 Verification not yet done

Re-running the rejection on the same recording with different thresholds (recomputing the rule from `poses` and `stamps`) and comparing the cross-view-consistency std in the [local-plane heatmap](local_plane_heatmap.md) would show the effect of the rule quantitatively. This comparison has not been made.

## 4. Evidence level per decision

| Decision | Value | Kind of evidence | Not verified |
|---|---|---|---|
| TF at the stamp | — | Observed (AMCL 5 Hz vs clouds 10 Hz) + reasoning | size of the effect of the offset on results |
| LiDAR mount correction | see guide | Observed (range-proportional bias on a flat floor) | separation from a real floor slope |
| Rejection: angular rate | 20 deg/s | Empirical value + after-the-fact check on one recording | sensitivity of results to the threshold, causation of angular rate |
| Rejection: linear velocity | 0.3 m/s | Jump detector (above driving speed limits) | size distribution of real AMCL jumps |
| Reference update and reset | — | Reproduced event (2/2) | — |
| Capture window scope | coverage + boundary repass | Design | — |
| Coverage speed | 0.16 m/s | Previously checked value kept | precision-versus-speed curve |
| Voxel, grid | 0.01 m | Empirical value (memory, matched to grid) | effect of changing the size |
| z window | ±0.025 m | Observed (symmetric spread, completeness 71.3% vs 94.65%) | effect of changing the width |
| Colour range, colormap | fixed to z window, `jet`, 0.75 | Presentation choice | — |

## 5. Frame log

Independently of the heatmap, with `save_frame_log: true` (default) the `map`-frame points before downsampling are saved per frame to `pointclouds/frames/frames_<ts>.npz`. Its layout:

| Key | Shape | Content |
|---|---|---|
| `stamps` | (F,) | message stamp [s] |
| `poses` | (F, 4) | `velodyne_link` pose in `map` (x, y, z, yaw) |
| `flags` | (F,) | bits: capture window, low-pass rejection |
| `n_raw` | (F,) | number of valid points before filtering |
| `offsets` | (F+1,) | `points[offsets[i]:offsets[i+1]]` are the points of frame i |
| `points` | (P, 3) | `map`-frame points (z kept only within `frame_log_z_min/max`, default ±0.5 m) |
| `intensity` | (P,) | reflectance intensity in the order of `points` (NaN if the topic has no `intensity` field) |
| `meta_*` | scalars | measurement conditions (IMU TF mode, LiDAR mount correction, …) |

This log is the input of the accumulation video, per-cell point-count analysis and the [local-plane heatmap](local_plane_heatmap.md).

## 6. Points to keep in mind when interpreting

- The cell mean does not distinguish at which range (ring) or in which travel direction a point was observed. Attitude error grows with range, so the same cell can take different values depending on the observation conditions, and circular-arc patterns centred on the sensor can appear.
- Heights are absolute in the `map` plane, so residual error in sensor height or mounting attitude shows up as a global offset or tilt. To view flatness as deviation from the surrounding plane, use the [local-plane heatmap](local_plane_heatmap.md).
- The colour range is a fixed representation of the measurement band, not a tolerance from a flatness standard.
