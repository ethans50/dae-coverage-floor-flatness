# Evaluation Protocol

[English](evaluation.md) · [한국어](../kr/evaluation.md) · [← README](../../README.md)

Metrics, experiment configurations, and reproducibility measures used to isolate the contribution of each of the five optimizations through leave-one-out ablation.

---

## Metrics

All computed by `analyze_coverage_comparison.py` from stored artefacts:

| Metric | Definition |
|---|---|
| **Completeness** | Fraction of target cells (1 cm) containing at least one floor return. Target cells are the union of coverage-node masks. The map-wide free-space value is reported alongside. |
| **Gap** | Count and area of unmeasured cells, and where they cluster. |
| **Per-cell z standard deviation** | Measurement noise; requires `save_raw_pcd` since the stored cloud is voxel-downsampled to one point per cell. |
| **Distance / time / total rotation** | Trajectory cost, on the same axes used by comparable coverage-planning papers. |
| **Active time** | Total time minus stalled time, so the effect of path planning is compared independently of whether a stall occurred. |
| **Efficiency** | Completeness gained per metre and per second. |

## Configurations

A leave-one-out ablation over the five optimizations, plus two anchors:

| Configuration | swath angle | pendant | entry hint | simplify | repass |
|---|:--:|:--:|:--:|:--:|:--:|
| `ours` | ✅ | ✅ | ✅ | ✅ | ✅ |
| `ours_wo_swathangle` | ❌ | ✅ | ✅ | ✅ | ✅ |
| `ours_wo_pendant` | ✅ | ❌ | ✅ | ✅ | ✅ |
| `ours_wo_entryhint` | ✅ | ✅ | ❌ | ✅ | ✅ |
| `ours_wo_simplify` | ✅ | ✅ | ✅ | ❌ | ✅ |
| `ours_wo_repass` | ✅ | ✅ | ✅ | ✅ | ❌ |
| `repass_only` | ❌ | ❌ | ❌ | ❌ | ✅ |
| `all_off` | ❌ | ❌ | ❌ | ❌ | ❌ |
| `centroid_only` | — | — | — | — | — |

Each configuration is repeated *n* ≥ 4, and every run is snapshotted into `eval_runs/<label>/` so configurations cannot overwrite one another.

## Reproducibility

Measures taken so the same input produces the same path on a different machine:

- **Pinned dependencies.** `requirements-mission_generation.txt` and `requirements-surface_profiling.txt` fix versions. Different library versions can change the node visit order between machines.
- **Pinned Fields2Cover commit.** Built from source at `85d6cf7`; the PyPI release produces different swath geometry.
- **Plan/runtime parameter cross-check.** `final_path_meta.json` records the parameters used at planning time and the executor refuses to start if they disagree with the live configuration.
- **Run labelling.** `eval_run_label:=<name>` snapshots the map, topology, path and all artefacts of a run into `eval_runs/<name>/`. `run_ts:=<YYYY-MM-DD_HH-MM-SS>`, given identically to both nodes, makes the two machines stamp their outputs with one shared timestamp.
- **Archival point clouds.** `combined_*.pcd` is kept for every run so a different z window or analysis grid can be recomputed offline without re-driving.
- **Deterministic re-analysis.** `reprocess_pcd.py` reads exactly the same `params.yaml` as the live pipeline, so it can be used to confirm which configuration values are actually in effect.

---

[← README](../../README.md)
