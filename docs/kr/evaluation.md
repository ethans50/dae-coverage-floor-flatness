# 평가 프로토콜

[English](../en/evaluation.md) · [한국어](evaluation.md) · [← README](../../README.kr.md)

5개 최적화 요소의 기여도를 leave-one-out ablation으로 분리 측정하기 위한 지표, 실험 조합, 재현성 장치를 정리함.

---

## 지표

전부 `analyze_coverage_comparison.py`가 저장된 산출물로부터 계산함:

| 지표 | 정의 |
|---|---|
| **완전성(Completeness)** | 바닥 리턴이 하나라도 있는 대상 셀(1 cm)의 비율. 대상 셀은 **Coverage 노드 마스크의 합집합**임. 맵 전체 free-space 기준값도 함께 출력함. |
| **Gap** | 미측정 셀의 개수/면적과, 미측정 영역이 몰려 있는 위치. |
| **셀당 z 표준편차** | 측정 노이즈. 저장된 Point Cloud는 셀당 1점으로 Downsampling되므로 `save_raw_pcd`가 필요함. |
| **거리 / 시간 / 총 회전각** | Trajectory 비용. 비교 대상 Coverage Path Planning 논문들과 동일한 축. |
| **순주행시간(Active Time)** | 총 시간에서 Stall 시간을 뺀 값. 경로 계획의 효과를 Stall 발생 여부와 분리해 비교하기 위함. |
| **효율(Efficiency)** | 단위 거리당 / 단위 시간당 Completeness 증가율. |

## 실험 조합

5개 최적화에 대한 leave-one-out ablation에 Anchor 조합 2개를 더함:

| 조합 | Swath 각도 | pendant | entry hint | simplify | repass |
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

각 조합을 *n* ≥ 4회 반복하고, 매 주행을 `eval_runs/<라벨>/`로 스냅샷해 조합끼리 서로 덮어쓰지 않게 함.

## 재현성

같은 입력이 다른 기기에서도 같은 경로를 내도록 하기 위한 장치:

- **의존성 버전 고정.** `requirements-mission_generation.txt` / `requirements-surface_profiling.txt`로 버전을 고정함. 라이브러리 버전이 다르면 기기 간 노드 방문 순서가 달라질 수 있음.
- **Fields2Cover 커밋 고정.** `85d6cf7` 소스 빌드. PyPI 릴리스는 Swath 형상이 다름.
- **계획-실행 파라미터 대조.** `final_path_meta.json`에 계획 시점 값을 기록하고, `mission_executor`가 현재 설정과 다르면 시작을 거부함.
- **주행 라벨링.** `eval_run_label:=<이름>`을 주면 그 주행의 맵·토폴로지·경로·산출물 전체가 `eval_runs/<이름>/`에 스냅샷됨. `run_ts:=<YYYY-MM-DD_HH-MM-SS>`를 두 노드에 동일하게 주면 서로 다른 기기가 하나의 공유 타임스탬프를 파일명에 씀.
- **Point Cloud 보관.** `combined_*.pcd`를 모든 주행분 보존해, z-window나 분석 격자를 바꿔도 재주행 없이 오프라인으로 재계산함.
- **결정적 재분석.** `reprocess_pcd.py`는 실제 파이프라인과 **완전히 동일한 방식**으로 `params.yaml`을 읽으므로, 어떤 설정값이 실제로 적용 중인지 확인하는 용도로도 쓸 수 있음.

---

[← README](../../README.kr.md)
