# mission_generation/mission_planning/algorithms/pendant_reorder.py
"""
허브형 토폴로지의 pendant 방문 순서를 허브의 실제 coverage 종료 지점 기준으로
재정렬하는 국소 후처리.

`tsp.py`의 Christofides 근사는 F2C 스와스가 생기기 전 노드 중심점 거리만 보므로,
"중앙 복도 하나에 방 여러 개가 매달린" 구조에서 각 방의 문이 허브의 실제 exit에서
얼마나 가까운지를 반영하지 못함.

`mission_planner.py`가 Step2.5에서 호출함 - bucket/safe_node_mask가 채워진 직후라야
허브의 스와스를 생성해 exit 좌표를 알 수 있기 때문에 그 시점에만 호출 가능함.
"""

import numpy as np

from mission_planning.utils import geometry


def reorder(tsp_sequence, detailed_sequence, node_waypoints, nodes,
            compute_node_raw_points, compute_adjusted_exit):
    """
    Christofides 근사(tsp.py)는 F2C 스와스가 생성되기 전, 노드
    중심점(centroid) 거리만으로 방문 순서를 정함. 허브형 토폴로지
    (중앙 복도 하나에 여러 방이 매달린 구조, 예: Apt.dae의 node2 <->
    {1,4,5,6})에서는 각 pendant 노드의 실제 연결 지점(waypoint)이
    허브의 실제 coverage 종료 지점(entry_hint에 의해서만 정해짐 -
    exit_hint는 고려하지 않음, order_swaths_by_entry 참고)에서 얼마나
    가까운지를 이 근사가 전혀 반영하지 못함.

    허브 h의 coverage가 확정된 직후(=h의 실제 물리적 exit 좌표를 알 수
    있는 시점), h에 '직접' 연결된(다른 노드를 거치지 않는) pendant
    노드들이 tsp_sequence 상에서 연속으로 나타나는 구간(=서로 직접
    연결되어 있지 않아 매번 h를 되짚어 지나가야만 하는 구간)을 찾아,
    그 구간만 h의 실제 exit 좌표에서부터 시작하는 nearest-neighbor
    순서로 재배열함. Christofides가 정한 전역적인 큰 흐름(어느
    허브/군집을 먼저·나중에 방문할지)은 건드리지 않고, 이미 정해진
    허브 도착 이후의 로컬 pendant 방문 순서만 다듬는 국소적 후처리임.

    pendant 사이의 이동 비용은 각자의 실제 coverage 스와스 형태까지
    고려하지 않고, hub<->pendant 연결 지점(waypoint) 사이의 유클리드
    거리로 근사함 - 매번 hub 복도를 되짚어 지나가야 하는 이 특정
    상황에서는 '문이 서로 얼마나 가까운가'가 실제 이동거리를 잘
    근사하기 때문임(각 pendant 자체의 coverage 왕복 비용은 방문
    순서와 무관하게 고정되므로 비교 대상에서 제외해도 됨).

    detailed_sequence 구조가 예상(hub와 pendant가 정확히 번갈아 나오는
    패턴)과 다르면(예: pendant끼리 직접 연결되어 있어 Dijkstra가 hub를
    경유하지 않은 경우) 안전하게 해당 구간의 재배열을 건너뜀 - 잘못된
    가정으로 경로 데이터를 조용히 훼손하는 것보다 나음.
    """
    tsp_sequence = list(tsp_sequence)
    detailed_sequence = list(detailed_sequence)

    def _dist(a, b):
        return float(np.hypot(a[0] - b[0], a[1] - b[1]))

    # tsp_sequence[j]가 detailed_sequence의 어느 인덱스에서 '커버리지
    # 방문'으로 등장하는지 Step3와 동일한 매칭 규칙으로 미리 기록해둠.
    target_positions = {}
    tsp_idx = 0
    for idx, n in enumerate(detailed_sequence):
        if tsp_idx < len(tsp_sequence) and n == tsp_sequence[tsp_idx]:
            target_positions[tsp_idx] = idx
            tsp_idx += 1

    j = 0
    while j < len(tsp_sequence):
        hub = tsp_sequence[j]
        run_start = j + 1
        k = run_start
        while k < len(tsp_sequence) and (hub, tsp_sequence[k]) in node_waypoints:
            k += 1
        run = tsp_sequence[run_start:k]

        if len(run) >= 2:
            hub_pos = target_positions.get(j)
            if hub_pos is not None:
                expected_old = []
                for idx2, n in enumerate(run):
                    expected_old.append(n)
                    if idx2 != len(run) - 1:
                        expected_old.append(hub)
                seg_start = hub_pos + 1
                seg_end = seg_start + len(expected_old)
                old_slice = detailed_sequence[seg_start:seg_end]

                if old_slice == expected_old:
                    prev_node = detailed_sequence[hub_pos - 1] if hub_pos > 0 else None
                    hub_entry_hint = node_waypoints.get((prev_node, hub)) if prev_node is not None else None
                    hub_raw_points = compute_node_raw_points(hub, hub_entry_hint)
                    # 허브 자신도 exit repass를 거치므로, pendant 순서를
                    # 정하는 anchor는 F2C 종료 지점이 아니라 repass가
                    # 끝난 뒤 로봇이 실제로 서 있을 위치여야 함
                    # (utils/repass_preview.py 참고) - Step3의
                    # current_pos 갱신과 동일한 기준.
                    anchor = compute_adjusted_exit(hub_raw_points) or \
                        geometry.get_centroid(nodes[hub]['driveable_mask'])

                    if anchor is not None:
                        remaining = list(run)
                        new_order = []
                        pos = anchor
                        while remaining:
                            best = min(remaining, key=lambda n: _dist(pos, node_waypoints[(hub, n)]))
                            new_order.append(best)
                            pos = node_waypoints[(hub, best)]
                            remaining.remove(best)

                        if new_order != run:
                            print(f"[*] Pendant reorder: hub Node {hub + 1}'s neighbors "
                                  f"{[n + 1 for n in run]} -> {[n + 1 for n in new_order]} "
                                  f"(nearest-neighbor from hub's actual coverage exit point).")
                            tsp_sequence[run_start:k] = new_order
                            new_slice = []
                            for idx2, n in enumerate(new_order):
                                new_slice.append(n)
                                if idx2 != len(new_order) - 1:
                                    new_slice.append(hub)
                            detailed_sequence[seg_start:seg_end] = new_slice
                else:
                    print(f"[WARN] Pendant reorder for hub Node {hub + 1}: detailed_sequence "
                          f"구조가 예상과 달라(pendant끼리 직접 연결된 경우 등) 재배열을 건너뜁니다.")

        j = k if len(run) >= 2 else run_start

    return tsp_sequence, detailed_sequence

