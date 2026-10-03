# Local-Plane Heatmap (`local_plane_heatmap.py`)

A tool that turns a frame log (`frames_*.npz`) into a heatmap of the **deviation from a local plane**, together with comparison metrics. Flatness is defined as what remains after subtracting a plane fitted to the surrounding floor, so long-wavelength slope is not measured; the mode selects the extent over which each point's plane is fitted (one frame, or a spatial window pooled over many frames). It runs on stored logs only, without re-driving.

## 1. Input and processing

The input is the frame log saved by `surface_profiler`. Given only a file name, the tool looks in `~/dae_floor_maps/analytics/pointclouds/{frames,waypoints}/`. Wall-proximity exclusion and the map overlay use `~/dae_floor_maps/maps/topology/final_topological_map.npz` and `~/dae_floor_maps/maps/grid/map_from_dae.yaml`.

Processing steps:

1. **Point selection**: only points whose sensor range `r` lies in `[--r-min, r_max]` and with `|z| ≤ --z-band` are used. `r_max` is the midpoint between the floor ring `--max-ring` (downward beam, 1 = innermost) and the next ring's radius (about 2.44 m for ring 4, the default, at a sensor height of 0.338 m).
2. **Wall-proximity exclusion**: by default, points outside the topology node masks (the band within 0.1 m of walls and the doorway-threshold sections) are removed. `--wall-margin M` additionally removes points within `M` m of a map wall.
3. **Residual per mode**: the plane is subtracted from each point. Planes are fitted with 2.5σ sigma clipping so that points far from the plane (local defects) do not bend it and erase themselves. Residuals are computed for points not used in the fit as well.

| Mode | Extent of the fitted plane |
|---|---|
| `raw` | none; only the global median is subtracted (reference for comparison) |
| `frame` | all points of one frame |
| `window` | frames pooled in map coordinates; for each point, all points within a circle of `--window` diameter (default 3 m) |

## 2. Design choices

- **Ring 4 and inside only**: attitude error grows with range (z error ≈ tilt × range), so outer rings disagree with inner-ring observations of the same cell. Using inner rings only reduces this term. The area within about 1.26 m of the sensor (inside where the lowest beam lands) is not seen by the LiDAR.
- **Points outside the nodes are excluded**: points within 0.1 m of a wall are consistently higher than inner observations of the same cell, and their z distribution stretches up to the top of the z window. The cause is not established (wall-face points may leak into the z window), and the exclusion is not evidence of improved accuracy. Final results use the default run with exclusion; `--outside-nodes keep` is for comparison diagnostics.
- **`frame` versus `window`**: `frame` absorbs the per-frame chassis attitude error (roll/pitch) into the plane, but also removes gentle real undulation inside the frame radius. `window` keeps more real undulation but does not reduce per-frame attitude error (different frames are merely averaged). Which is appropriate depends on the extent of the reference plane in the flatness definition.

## 3. Running it

Run from the package root. The default run computes all three modes and the synthetic-dome retention, so it takes a few minutes.

```bash
# Default: ring 4 and inside, nodes-outside points excluded, raw/frame/window compared
python3 surface_profiling/local_plane_heatmap.py frames_<timestamp>.npz

# Diagnostic result including points outside the nodes (file name gets _keep)
python3 surface_profiling/local_plane_heatmap.py frames_<timestamp>.npz --outside-nodes keep

# Also exclude points within 0.2 m of walls, and draw where the excluded points are
python3 surface_profiling/local_plane_heatmap.py frames_<timestamp>.npz --wall-margin 0.2 --show-excluded

# frame mode only, synthetic dome placed inside a room at (x, y), colour range ±1.5 cm
python3 surface_profiling/local_plane_heatmap.py frames_<timestamp>.npz --modes frame --inject-at -6.7 1.0 --vrange 1.5

# Heatmap only, skipping the retention computation
python3 surface_profiling/local_plane_heatmap.py frames_<timestamp>.npz --no-inject
```

| Option | Default | Description |
|---|---|---|
| `npz` | required | frame log (path or file name) |
| `--modes` | `raw,frame,window` | modes to compare (comma-separated) |
| `--max-ring` | 4 | outermost floor ring to use |
| `--r-min` | 1.1 | minimum sensor range [m] |
| `--z-band` | 0.06 | only points with `|z|` at or below this [m] are used |
| `--window` | 3.0 | plane window diameter for `window` mode [m] |
| `--outside-nodes` | `exclude` | `keep` includes points outside the nodes |
| `--wall-margin` | 0 | exclude points within this distance [m] of a map wall (0 = off) |
| `--show-excluded` | off | additionally save a map of the excluded points only |
| `--vrange` | 2.0 | heatmap colour range ±[cm] |
| `--inject-at X Y` | densest 1 m block | synthetic dome centre (map coordinates [m]); give a point inside a room so it does not land near a node boundary |
| `--no-inject` | off | skip the synthetic-dome retention |
| `--topology`, `--map` | `~/dae_floor_maps/maps/…` | node mask and map paths |
| `--out-dir` | `~/dae_floor_maps/visualization/surface_profiling` | output folder |

## 4. Output

File names differ per option combination, so runs do not overwrite each other (`<ts>` is the log's timestamp).

| File | Content |
|---|---|
| `local_plane_<ts>[_keep][_wm<M>].png` | heatmap per mode (cell-mean residual, colour range ±`--vrange` cm) |
| `local_plane_<ts>[_keep][_wm<M>]_metrics.png` | synthetic-dome retention and per-ring mean residual plots |
| `local_plane_<ts>[_keep][_wm<M>]_excluded.png` | map of the excluded points only (with `--show-excluded`) |

Excluded regions (the outside-nodes band, the `--wall-margin` band) contain no points and appear white in the heatmap. The terminal prints the metrics below, one line per mode.

## 5. Reading the metrics

Without ground truth, a heatmap alone cannot tell "error that was removed" from "a real defect that was removed with it". All metrics below are computed without ground truth and are **necessary conditions** only; a bias that is consistent in one direction is not detected.

| Metric | Meaning | Better |
|---|---|---|
| Crossover std | std of the difference between the mean of the same 5 cm cell as seen by frames of opposite travel directions. The floor is the same, so the difference is measurement error | smaller |
| Dome retention | a raised-cosine dome `z = h/2·(1+cos(πr/R))` (`r < R`, h = 2 cm, R = 0.25–1.25 m) is added to the points, the same processing is applied, and the returned signal is fitted to the dome shape by least squares (gain) | closer to 1 keeps real defects; larger domes lower it as `frame`/`window` absorb them into the plane |
| Per-ring mean residual | mean residual [cm] per ring | smaller bias between rings |
| Cell-mean spatial std | std of the cell-mean residuals (size of the remaining pattern) | alone it does not separate distortion from signal; read it with the metrics above |

## 6. Limitations

- If the path was generated without setting the LiDAR range (`lidar_range`) to the ring-4 limit, strips along walls may be observed only by ring 5 and beyond, and appear empty in this tool.
- Synthetic-dome retention shows how much the processing erases large defects; it does not guarantee detection of real construction defects.
- If the robot drives over a protrusion and the chassis tilts or the sensor height changes, the `raw`/`window` residuals are contaminated. `frame` removes the constant and planar parts within a frame and is not affected by this.
