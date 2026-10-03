# 국소 평면 히트맵 (`local_plane_heatmap.py`)

프레임 기록(`frames_*.npz`)에서 **국소 평면 대비 편차**를 계산해 히트맵과 비교 지표를 만드는 도구임. 평탄도를 "주변 바닥에 맞춘 평면을 뺀 나머지"로 정의하므로 장파장 경사는 측정 대상이 아니며, 점마다 평면을 맞추는 범위(프레임 하나 / 여러 프레임을 합친 공간 창)를 모드로 고름. 재주행 없이 저장된 기록만으로 실행함.

## 1. 입력과 처리

입력은 `surface_profiler`가 저장한 프레임 기록임. 파일 이름만 주면 `~/dae_floor_maps/analytics/pointclouds/{frames,waypoints}/`에서 찾음. 벽 근처 제외와 지도 겹쳐 그리기에는 `~/dae_floor_maps/maps/topology/final_topological_map.npz`와 `~/dae_floor_maps/maps/grid/map_from_dae.yaml`을 씀.

처리 순서:

1. **사용 점 선택**: 센서 거리 `r`이 `[--r-min, r_max]`이고 `|z| ≤ --z-band`인 점만 씀. `r_max`는 `--max-ring`번째 하향 빔(바닥 ring, 1이 가장 안쪽)과 그다음 ring 반경의 중간값임(센서 높이 0.338 m, 기본 ring 4이면 약 2.44 m).
2. **벽 근처 제외**: 기본은 토폴로지 노드 마스크 밖의 점(벽 0.1 m 이내 띠와 문지방 구간)을 뺌. `--wall-margin M`을 주면 지도 벽에서 `M` m 이내 점도 뺌.
3. **모드별 잔차**: 점에서 평면을 뺀 값을 모드별로 계산함. 평면은 2.5σ 시그마 클리핑으로 맞춰서 평면에서 크게 벗어난 점(국소 결함)이 평면을 휘게 해 스스로를 지우는 것을 줄임. 잔차는 평면 적합에 안 쓰인 점에도 계산함.

| 모드 | 평면을 맞추는 범위 |
|---|---|
| `raw` | 평면 없음. 전체 중앙값만 뺌(비교 기준) |
| `frame` | 프레임 하나의 점 전체 |
| `window` | 여러 프레임을 map 좌표에서 모아, 점마다 주변 `--window`(기본 3 m) 지름 원 안의 점 전체 |

## 2. 핵심 선택

- **라이다의 ring 4까지만 사용**: 자세 오차는 거리에 비례해 커지므로(z 오차 ≈ 기울기 × 거리) 바깥 ring일수록 같은 셀의 안쪽 ring 관측과 어긋남. 안쪽 4개 ring만 써서 이 항을 줄임. 센서 아래 반경 약 1.26 m(가장 아래 빔이 닿는 곳 안쪽)는 라이다가 보지 못하는 구간임.
- **노드 밖 점 제외**: 벽 0.1 m 이내 점은 같은 셀의 안쪽 관측보다 일관되게 높고 z 분포가 z 창 상단까지 길게 퍼짐. 원인은 확정되지 않았으며(벽 면의 점이 z 창에 섞였을 가능성), 이 제외가 정확도 개선을 입증하는 것은 아님. 최종 결과는 제외한 기본 실행을 쓰고, `--outside-nodes keep`은 비교 진단용임.
- **`frame`과 `window`의 차이**: `frame`은 프레임마다 다른 차체 자세 오차(roll/pitch)를 평면에 흡수하지만, 프레임 반경 안의 완만한 실제 기복도 같이 빠짐. `window`는 실제 기복이 덜 빠지지만 프레임별 자세 오차를 줄이지 못함(서로 다른 프레임이 섞여 평균될 뿐임). 어느 쪽이 맞는지는 평탄도를 어느 범위의 평면 대비로 정의하느냐에 달림.

## 3. 실행

패키지 루트에서 실행함. 기본 실행은 세 모드를 모두 계산하고 합성 돔 보존율까지 내므로 몇 분 걸림.

```bash
# 기본: ring 4 이내, 노드 밖 점 제외, raw/frame/window 비교
python3 surface_profiling/local_plane_heatmap.py frames_<타임스탬프>.npz

# 노드 밖 점을 포함한 진단용 결과(파일 이름에 _keep이 붙음)
python3 surface_profiling/local_plane_heatmap.py frames_<타임스탬프>.npz --outside-nodes keep

# 벽에서 0.2 m 이내도 제외하고, 제외된 점이 어디에 있는지도 그림
python3 surface_profiling/local_plane_heatmap.py frames_<타임스탬프>.npz --wall-margin 0.2 --show-excluded

# frame 모드만, 합성 돔을 방 안쪽 (x, y)에 넣어 보존율 확인, 색 범위 ±1.5 cm
python3 surface_profiling/local_plane_heatmap.py frames_<타임스탬프>.npz --modes frame --inject-at -6.7 1.0 --vrange 1.5

# 보존율 계산을 생략해 빠르게 히트맵만
python3 surface_profiling/local_plane_heatmap.py frames_<타임스탬프>.npz --no-inject
```

| 옵션 | 기본값 | 설명 |
|---|---|---|
| `npz` | 필수 | 프레임 기록 파일(경로 또는 파일 이름) |
| `--modes` | `raw,frame,window` | 비교할 모드(쉼표로 구분) |
| `--max-ring` | 4 | 쓸 가장 바깥 바닥 ring |
| `--r-min` | 1.1 | 최소 센서 거리 [m] |
| `--z-band` | 0.06 | `|z|`가 이 값[m] 이하인 점만 사용 |
| `--window` | 3.0 | `window` 모드 평면 창 지름 [m] |
| `--outside-nodes` | `exclude` | `keep`이면 노드 밖 점을 포함 |
| `--wall-margin` | 0 | 지도 벽에서 이 거리[m] 이내 점 제외(0이면 사용 안 함) |
| `--show-excluded` | 꺼짐 | 제외된 점만 그린 지도를 추가로 저장 |
| `--vrange` | 2.0 | 히트맵 색 범위 ±[cm] |
| `--inject-at X Y` | 점이 가장 많은 1 m 블록 | 합성 돔 중심(map 좌표 [m]). 노드 경계 근처가 되지 않도록 방 안쪽으로 지정할 것 |
| `--no-inject` | 꺼짐 | 합성 돔 보존율 계산 생략 |
| `--topology`, `--map` | `~/dae_floor_maps/maps/…` | 노드 마스크와 지도 경로 |
| `--out-dir` | `~/dae_floor_maps/visualization/surface_profiling` | 출력 폴더 |

## 4. 출력

옵션 조합별로 파일 이름이 달라서 서로 덮어쓰지 않음(`<ts>`는 기록의 타임스탬프).

| 파일 | 내용 |
|---|---|
| `local_plane_<ts>[_keep][_wm<M>].png` | 모드별 히트맵(셀 평균 잔차, 색 범위 ±`--vrange` cm) |
| `local_plane_<ts>[_keep][_wm<M>]_metrics.png` | 합성 돔 보존율과 링별 평균 잔차 그래프 |
| `local_plane_<ts>[_keep][_wm<M>]_excluded.png` | 제외된 점만 그린 지도(`--show-excluded`일 때) |

제외된 구간(노드 밖 띠, `--wall-margin` 띠)은 점이 없어 히트맵에서 하얗게 보임. 터미널에는 모드별로 아래 지표를 한 줄씩 출력함.

## 5. 지표 읽는 법

정답(실측 기준값)이 없으므로 히트맵만으로는 "오차가 지워진 것"과 "실제 결함이 같이 지워진 것"을 구분할 수 없음. 아래 지표는 모두 정답 없이 내는 **필요조건**이며, 같은 방향으로 일관된 편향은 잡지 못함.

| 지표 | 의미 | 좋은 방향 |
|---|---|---|
| 교차 일관성 std | 같은 5 cm 셀을 서로 반대 진행 방향 프레임 그룹이 각각 본 평균의 차이 표준편차. 바닥은 같으므로 차이는 측정 오차임 | 작을수록 좋음 |
| 돔 보존율 | 높이 2 cm의 raised-cosine 돔 `z = h/2·(1+cos(πr/R))`(`r < R`, R = 0.25~1.25 m)을 점에 더해 같은 처리를 한 뒤, 돌려받은 신호를 돔 모양에 최소제곱으로 맞춘 이득 | 1에 가까울수록 실제 결함이 보존됨. 돔이 클수록 `frame`/`window`가 평면으로 흡수해 낮아짐 |
| 링별 평균 잔차 | ring마다 평균 잔차[cm] | ring 사이 편향이 작을수록 좋음 |
| 셀 평균 공간 std | 셀 평균 잔차의 표준편차(남은 무늬의 크기) | 단독으로는 왜곡과 구분되지 않으므로 위 지표와 함께 볼 것 |

## 6. 한계

- 경로를 만들 때 라이다 범위(`lidar_range`)를 ring 4 기준으로 맞추지 않았다면, 벽 쪽 띠가 ring 5 이상으로만 관측돼 이 도구에서는 비어 보일 수 있음.
- 합성 돔 보존율은 후처리가 큰 결함을 얼마나 지우는지 보는 것이며, 실제 시공 결함의 검출을 보장하지 않음.
- 로봇이 돌출 위를 지나며 차체가 기울거나 센서 높이가 변하면 `raw`/`window`의 잔차가 오염됨. `frame`은 프레임 안의 상수·평면 성분을 지우므로 이 영향을 받지 않음.
