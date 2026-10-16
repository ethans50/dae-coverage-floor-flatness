# mission_generation/mission_planning/utils/coverage_check.py
"""
planning된 경로만으로 "바닥 점이 충분히 안 찍힐 셀"을 예측함(측정 전).

VLP-16은 센서 둘레의 몇 개 ring 원(반경 h/tan(각도))만 바닥에 찍으므로, 한 셀에 점이 모이려면
로봇이 그 셀과 ring 반경만큼 떨어진 자리를 지나가야 함. 한 직선 구간의 양 끝점 사이가 블라인드
반경(링1)의 2배보다 짧으면 구간 중앙 셀은 양 끝 어디서도 링 거리에 못 들어와 비게 됨(짧은 노드).
그 노드 바로 다음 transit을 측정(record_pcd)에 포함하면 메워지는 경우가 많아, 이 모듈은 어떤
transit을 측정에 넣을지 고르는 근거를 계산함.

예측 규칙: 경로를 STEP_M 간격으로 샘플링(한 샘플 = 센서 프레임 1개로 봄, 10 Hz에서 약 0.2 m/s)하고,
셀 중심과 샘플의 거리가 어떤 ring 반경의 +-RING_TOL_M 이내이며 둘을 잇는 선분이 벽을 지나지 않으면
그 샘플을 셀의 히트로 셈. 히트가 MIN_HITS 미만이면 부족 셀로 봄.
- CELL_M: 평탄도 분석 격자와 같은 크기. RING_TOL_M은 그 셀 반폭(셀 안에 링이 걸리는 거리 폭).
- 벽 가림 조건이 필요한 이유: 칸막이 벽 반대편에서 오는 링 히트는 실제로는 점을 못 만듦.
- 분석 영역은 평탄도 분석과 같은 조건(노드 안, 벽에서 WALL_MARGIN_M 이상).
로봇은 각 목표점에 허용 오차 안으로만 접근해 멈추므로 측정 구간의 끝은 계획 끝점보다 END_TRIM_M만큼 안쪽에서 끝나는 것으로 봄
(lens가 생기는 짧은 구간은 이 몇 cm에 판정이 갈림).
센서 오차, 로봇의 회전 중 기각 프레임, 속도 변화는 반영하지 않으므로 이 예측은 "부족 가능성이
있는 셀" 판정용이고 점 수 자체를 예측하지는 않음.
"""

import math

import cv2
import numpy as np

from mission_planning.utils.ring_coverage import vlp16_ring_radii_m

CELL_M = 0.125
RING_TOL_M = 0.0625
MIN_HITS = 3
STEP_M = 0.02
WALL_MARGIN_M = 0.2
END_TRIM_M = 0.1
_LOS_STEP_PX = 0.7
# 실행기의 sub-segment 분할 규칙(mission_executor)과 Nav2 BT의 RemovePassedGoals 반경(coverage는 nav2 기본 BT 0.7 m,
# transit은 이 패키지의 전용 BT 0.3 m). 둘 중 하나가 바뀌면 경로 예측이 어긋나므로 같이 맞출 것.
SPLIT_DEG = 30.0   # mission_execution.direction_change_threshold_deg 와 같은 값이어야 함
PASSED_GOAL_RADIUS_M = {'coverage': 0.7, 'transit': 0.3}


def executed_paths(final_path):
    """final_path.json 형식 웨이포인트(헤더 포함)에서 실행기가 실제로 주행할 궤적(미터)을 근사해 구간별로 돌려줌:
    [{'record': bool, 'xy': (N,2) 배열, 'stops_capture': bool}, ...].
    실행기는 경로를 heading이 SPLIT_DEG 넘게 바뀌거나 측정 여부가 바뀌는 지점에서 sub-segment로 나누고(코너 하나만
    남는 1점 구간은 다음 구간에 편입) 각 sub-segment를 목표점 목록으로 Nav2에 보냄. 단, 실행기는 코너에서 경로가
    크게 되돌아 나가는 경우(mission_execution.subsegment_merge_max_turn_deg 초과, 앞뒤 구간이 subsegment_merge_min_leg_m
    이상)에는 그 코너를 편입하지 않고 단독 목표로 먼저 도달함. 이 함수는 그 예외를 따르지 않고 항상 편입하는 것으로
    근사하므로, 그런 코너에서는 실제 주행이 예측보다 코너 안쪽까지 들어갈 수 있음(보수적 방향의 차이). 이때 BT가 로봇에서 반경
    PASSED_GOAL_RADIUS_M 안에 든 중간 목표를 지난 것으로 지우므로, 마지막 목표가 아닌 점은 그 반경 앞에서 다음
    목표를 향해 코너를 자르는 것으로 봄. 마지막 목표는 도달하는 것으로 봄(허용 오차는 END_TRIM_M로 따로 다룸)."""
    n = len(final_path)
    if n == 0:
        return []
    yaw = [2.0 * math.atan2(p['pose']['orientation']['z'], p['pose']['orientation']['w']) for p in final_path]
    rec = [bool(p['header'].get('record_pcd', True)) for p in final_path]
    xy = np.array([[p['pose']['position']['x'], p['pose']['position']['y']] for p in final_path])
    thr = math.radians(SPLIT_DEG)

    segs, st = [], 0
    for i in range(1, n):
        d = math.atan2(math.sin(yaw[i] - yaw[i - 1]), math.cos(yaw[i] - yaw[i - 1]))
        if abs(d) > thr or rec[i] != rec[i - 1]:
            segs.append((st, i))
            st = i
    segs.append((st, n))

    merged, i = [], 0
    while i < len(segs):
        a, b = segs[i]
        single = (b - a == 1 and not final_path[a]['header']['task_type'].endswith('_single')
                  and i + 1 < len(segs) and rec[a] == rec[segs[i + 1][0]])
        if single:
            merged.append((a, segs[i + 1][1]))
            i += 2
        else:
            merged.append((a, b))
            i += 1

    out, cur = [], xy[0]
    for k, (a, b) in enumerate(merged):
        goals = list(xy[a:b])
        if k > 0 and not rec[a] and len(goals) > 1 and np.hypot(*(goals[0] - cur)) < 1e-3:
            goals = goals[1:]
        radius = PASSED_GOAL_RADIUS_M['coverage' if final_path[a]['header']['task_type'].startswith('coverage') else 'transit']
        pts = [cur.copy()]
        for j, g in enumerate(goals):
            d = float(np.hypot(*(g - pts[-1])))
            if j < len(goals) - 1:
                if d > radius:
                    pts.append(pts[-1] + (g - pts[-1]) * (d - radius) / d)
            else:
                pts.append(g.copy())
        cur = pts[-1]
        out.append({'record': rec[a], 'xy': np.array(pts),
                    'stops_capture': rec[a] and (k == len(merged) - 1 or not rec[merged[k + 1][0]])})
    return out


class CoverageModel:
    """노드 마스크(px, 이미지 좌표)로 분석 셀과 벽 가림 마스크를 만들고 히트 수를 셈."""

    def __init__(self, nodes, map_resolution, lidar_mount_height, floor_ring_max=4):
        self.res = float(map_resolution)
        floor = np.zeros(nodes[0]['driveable_mask'].shape, bool)
        inside = np.zeros_like(floor)
        for n in nodes:
            inside |= n['driveable_mask'] > 0
            floor |= n['driveable_mask'] > 0
            if n.get('nondriveable_mask') is not None:
                floor |= n['nondriveable_mask'] > 0
        self.block = ~floor
        wall_m = cv2.distanceTransform(floor.astype(np.uint8), cv2.DIST_L2, 5) * self.res

        step = CELL_M / self.res
        ys, xs = np.meshgrid(np.arange(step / 2, floor.shape[0], step), np.arange(step / 2, floor.shape[1], step), indexing='ij')
        iy, ix = ys.astype(int), xs.astype(int)
        keep = inside[iy, ix] & (wall_m[iy, ix] >= WALL_MARGIN_M)
        self.cells = np.c_[xs[keep], ys[keep]]
        self.ring_px = vlp16_ring_radii_m(lidar_mount_height, floor_ring_max) / self.res
        self.tol_px = RING_TOL_M / self.res

    def sample(self, path, trim_start=False, trim_end=False):
        """px 꺾은선을 STEP_M 간격 점들로 바꿈(구간 끝점 포함). trim_*면 그쪽 끝에서 END_TRIM_M만큼 뺌."""
        pts = np.asarray(path, dtype=float).reshape(-1, 2)
        out = [pts[:1]]
        step = STEP_M / self.res
        for a, b in zip(pts[:-1], pts[1:]):
            n = max(int(np.ceil(np.hypot(*(b - a)) / step)), 1)
            out.append(a + (b - a) * (np.arange(1, n + 1) / n)[:, None])
        out = np.concatenate(out)
        k = int(round(END_TRIM_M / STEP_M))
        if len(out) <= 2 * k + 1:  # 한 점/아주 짧은 구간은 자르지 않음
            return out
        return out[(k if trim_start else 0):len(out) - (k if trim_end else 0)]

    def hits(self, samples):
        """셀마다의 히트 수(셀 수,) - samples는 sample()이 돌려준 (N,2) px 점."""
        h = np.zeros(len(self.cells), dtype=np.int32)
        if len(samples) == 0:
            return h
        for i in range(0, len(self.cells), 100):
            c = self.cells[i:i + 100]
            d = np.hypot(c[:, None, 0] - samples[None, :, 0], c[:, None, 1] - samples[None, :, 1])
            m = np.zeros(d.shape, bool)
            for r in self.ring_px:
                m |= np.abs(d - r) <= self.tol_px
            ci, si = np.nonzero(m)
            if ci.size:
                clear = self._clear(c[ci], samples[si])
                m[ci[~clear], si[~clear]] = False
            h[i:i + 100] = m.sum(axis=1)
        return h

    def _clear(self, a, b):
        """a, b (K,2) 쌍을 잇는 선분이 벽 픽셀을 지나지 않으면 True."""
        ok = np.ones(len(a), bool)
        n = int(np.ceil(np.hypot(*(b - a).T).max() / _LOS_STEP_PX)) + 1
        for t in np.linspace(0.0, 1.0, n):
            p = a + (b - a) * t
            ix = np.clip(p[:, 0].astype(int), 0, self.block.shape[1] - 1)
            iy = np.clip(p[:, 1].astype(int), 0, self.block.shape[0] - 1)
            ok &= ~self.block[iy, ix]
        return ok
