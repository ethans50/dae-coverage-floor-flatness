# mission_generation/mission_planning/mission_planner.py

import numpy as np
import cv2
import os
import math
import time

from mission_planning.algorithms import tsp, coverage, transit, pendant_reorder
from mission_planning.utils import visualizer, geometry, repass_preview
from mission_planning import path_exporter

class MissionPlanner:
    # 파라미터 업데이트
    def __init__(self, topomap_path, visualization_dir="./debug", robot_width=0.28, path_safety_margin=0.25, lidar_range=8.4, overlap=0.2, turn_weight=2.0, wall_weight=5.0, lidar_mount_height=0.338, lidar_vertical_fov_deg=15.0,
             blind_radius_m=None, boundary_repass_distance_m=1.5, enable_boundary_repass=True,
             boundary_repass_max_segment_m=2.8,
             enable_pendant_reorder=True, enable_entry_hint_ordering=True, enable_path_simplification=True,
             coverage_mode="full", enable_optimal_swath_angle=True, **kwargs):
        if not os.path.exists(topomap_path):
            raise FileNotFoundError(f"[!] Topomap file not found at: {topomap_path}")
        if coverage_mode not in ("full", "centroid_only"):
            raise ValueError(f"[!] Invalid coverage_mode: {coverage_mode!r} (expected 'full' or 'centroid_only')")

        data = np.load(topomap_path, allow_pickle=True)

        masks = data['nodes']
        nondriveable_masks = data.get('nondriveable_nodes', [])

        self.map_resolution = float(data.get('resolution', 0.05))
        self.origin = data.get('origin', [0, 0])

        self.robot_width = robot_width
        self.path_safety_margin = path_safety_margin
        self.visualization_dir = os.path.abspath(visualization_dir)

        # mission_execution.boundary_repass_distance_m/enable_boundary_repass와
        # 동일한 값 - 실행 시 BoundaryRepassController가 실제로 로봇을 데려다
        # 놓을 위치(retrace 지점)를 planning 단계에서도 반영하기 위함
        # (_compute_repass_adjusted_exit 참고). 이 값은 시각화 미리보기뿐
        # 아니라 Step3의 current_pos(다음 노드로 가는 transit의 실제
        # 시작점)에도 반영되어 self.path_segments/final_path.json 자체를
        # 바꿈 - planning-실행 값 불일치 시 위험.
        self.boundary_repass_distance_m = boundary_repass_distance_m
        self.enable_boundary_repass = enable_boundary_repass
        self.boundary_repass_max_segment_m = boundary_repass_max_segment_m

        # ablation 실험용 토글 3종 - 각 메커니즘의 기여도를 개별적으로 끄고
        # 측정하기 위함. 기본값은 모두 True(현재
        # 파이프라인 동작과 동일) - False로 두면 해당 메커니즘 없이 생성했을
        # final_path.json을 얻을 수 있음.
        self.enable_pendant_reorder = enable_pendant_reorder
        self.enable_entry_hint_ordering = enable_entry_hint_ordering
        self.enable_path_simplification = enable_path_simplification

        # 3-way 알고리즘 비교 실험용 토글임. coverage_mode="centroid_only"면
        # 모든 노드에서 F2C 스와스 생성을 건너뛰어 swath_pairs가 빈 리스트가 되고,
        # 이미 있는 "스와스 생성 실패 시 centroid로 폴백"하는 코드 경로
        # (_compute_node_raw_points/Step3 인라인 로직)가 그대로 재사용되어 노드
        # 중앙점 1점만 방문하는 경로가 만들어짐 - 새 알고리즘 코드 없이 기존
        # 폴백을 재활용하는 구조임. enable_optimal_swath_angle=False면 wide 노드도
        # generateBestSwaths 각도 자동탐색 없이 0도 고정 스와스를 씀(coverage.py 참고).
        self.coverage_mode = coverage_mode
        self.enable_optimal_swath_angle = enable_optimal_swath_angle

        self.turn_weight = float(turn_weight)
        self.wall_weight = float(wall_weight)

        default_r = lidar_mount_height / math.tan(math.radians(lidar_vertical_fov_deg))
        self.blind_radius_m = blind_radius_m if blind_radius_m is not None else max(default_r, 1.0)
        self.blind_radius_px = self.blind_radius_m / self.map_resolution

        print(f"[*] map_resolution={self.map_resolution}, blind_radius_m={self.blind_radius_m:.3f}, blind_radius_px={self.blind_radius_px:.1f}")

        self.nodes = []
        for i in range(len(masks)):
            node_dict = {
                'id': i + 1,
                'driveable_mask': masks[i],
                'nondriveable_mask': nondriveable_masks[i] if i < len(nondriveable_masks) else None
            }
            self.nodes.append(node_dict)

        if not self.nodes:
            print("[ERROR] No nodes found in topomap file.")
            return

        # 전체 구동 가능 영역 병합 (A* 등에서 활용)
        self.global_mask = np.zeros_like(self.nodes[0]['driveable_mask'])
        for node in self.nodes:
            if node['driveable_mask'] is not None:
                self.global_mask = cv2.bitwise_or(self.global_mask, node['driveable_mask'])
        
        # 알고리즘 모듈들에 넘겨줄 로봇 파라미터 캡슐화
        self.robot_params = {
            'width_px': robot_width / self.map_resolution,
            'effective_swath_m': lidar_range * (1.0 - overlap),
            'swath_width_px': (lidar_range * (1.0 - overlap)) / self.map_resolution
        }
        
        # 결과 저장 리스트
        self.path_segments = []

        print(f"[*] MissionPlanner Initialized")

        if kwargs:
            print(f"[WARN] MissionPlanner received unexpected kwargs (ignored): {list(kwargs.keys())}")

    def generate_cost_map(self, mask):
        dist_px = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
        dist_m = dist_px * self.map_resolution
        
        cost_map = np.zeros_like(mask, dtype=np.uint8)
        
        # 1. 완벽한 안전 영역 (벽으로부터 로봇반경+마진 이상 떨어짐): 255
        safe_threshold = self.robot_width + self.path_safety_margin
        cost_map[dist_m >= safe_threshold] = 255
        
        # 2. 소프트 페널티 영역 (벽과 가깝지만 통과는 가능함, ex: 문지방): 50 ~ 250 그라데이션
        penalty_mask = (dist_m >= self.robot_width) & (dist_m < safe_threshold)
        if np.any(penalty_mask):
            normalized_dist = (dist_m[penalty_mask] - self.robot_width) / self.path_safety_margin
            cost_map[penalty_mask] = (50 + normalized_dist * 200).astype(np.uint8)
            
        # 3. 절대 불가 영역 (물리적 로봇 반경 이내): 0 (기본값이 0이므로 별도 대입 생략)
        return cost_map

    def generate_coverage_mask(self, mask):
        dist_px = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
        dist_m = dist_px * self.map_resolution
        
        coverage_mask = np.zeros_like(mask, dtype=np.uint8)
        safe_threshold = self.robot_width + self.path_safety_margin
        
        coverage_mask[dist_m >= safe_threshold] = 255
        
        # 폴백 방어 로직: 맵이 너무 좁아서 마진 적용 시 영역이 아예 사라지면 로봇 반경까지만 깎음
        if cv2.countNonZero(coverage_mask) == 0:
            coverage_mask[dist_m >= self.robot_width] = 255
            
        return coverage_mask

    def _order_swaths(self, swath_pairs, entry_hint, exit_hint=None):
        """entry_hint(없으면 exit_hint 역산) 기준 스와스 정렬 - Step3
        인라인 로직과 _compute_node_raw_points가 공유하는 단일 구현.
        enable_entry_hint_ordering=False면 진입/진출 힌트를 모두 무시하고
        order_swaths_by_entry(swath_pairs, None)의 기본 순서(첫 스와스
        시작점 기준)를 강제함 - ablation 실험용 토글."""
        if not self.enable_entry_hint_ordering:
            return geometry.order_swaths_by_entry(swath_pairs, None)
        if entry_hint is None:
            if exit_hint is not None:
                ordered_pairs = geometry.order_swaths_by_entry(swath_pairs, exit_hint)
                return [(p2, p1) for p1, p2 in reversed(ordered_pairs)]
            return geometry.order_swaths_by_entry(swath_pairs, None)
        return geometry.order_swaths_by_entry(swath_pairs, entry_hint)

    def _compute_node_raw_points(self, node_idx, entry_hint, exit_hint=None):
        """지정 노드의 F2C 커버리지 raw_points를 생성함(entry_hint 기준
        방향 정렬 - entry_hint가 None일 때만 exit_hint로 대신 정렬). Step3의
        인라인 계산과 정확히 동일한 로직임(디버그 이미지 저장 부분만
        제외) - _reorder_pendant_groups가 허브 노드의 실제 coverage 종료
        지점을 Step3보다 먼저 알아야 해서 이 부분만 별도 메서드로 추출함.
        node['bucket']/['safe_node_mask']가 이미 채워져 있어야 함(Step2
        완료 후에만 호출 가능)."""
        bucket = self.nodes[node_idx]['bucket']
        safe_node_mask = self.nodes[node_idx]['safe_node_mask']

        if self.coverage_mode == 'centroid_only':
            # 스와스 생성을 아예 건너뜀 - 아래 "swath_pairs가 비면 centroid로
            # 폴백"하는 기존 로직이 그대로 노드 중앙점 방문 경로를 만들어줌
            swath_pairs = []
        elif bucket in ('narrow', 'ultra_narrow'):
            forced_angle = geometry.get_long_axis_angle_rad(safe_node_mask)
            swath_pairs = coverage.generate_raw_swaths(safe_node_mask, self.robot_params, decompose=True, split_angle_rad=forced_angle)
        else:
            swath_pairs = coverage.generate_raw_swaths(safe_node_mask, self.robot_params, enable_optimal_swath_angle=self.enable_optimal_swath_angle)

        raw_points = []
        if swath_pairs:
            ordered_pairs = self._order_swaths(swath_pairs, entry_hint, exit_hint)
            for p1, p2 in ordered_pairs:
                raw_points.extend([p1, p2])
        else:
            centroid = geometry.get_centroid(self.nodes[node_idx]['driveable_mask'])
            if centroid:
                raw_points.append(centroid)
        return raw_points

    def _compute_repass_adjusted_exit(self, raw_points):
        """coverage 노드 하나의 실제 물리적 exit 지점(되짚기 후 로봇이 서 있게
        될 retrace 지점)을 돌려줌 - 계산 규칙은 utils/repass_preview.py 참고.
        Step3의 current_pos와 _reorder_pendant_groups의 허브 anchor가 공유함."""
        return repass_preview.compute_adjusted_exit(
            raw_points, self.enable_boundary_repass,
            self.boundary_repass_distance_m, self.map_resolution,
            self.boundary_repass_max_segment_m)

    def _reorder_pendant_groups(self, tsp_sequence, detailed_sequence, node_waypoints):
        """허브에 매달린 pendant 노드들의 방문 순서만 국소적으로 다듬음 -
        계산 규칙은 algorithms/pendant_reorder.py 참고. 허브의 exit 좌표가
        필요해 raw point 계산과 repass 보정을 콜백으로 넘김."""
        return pendant_reorder.reorder(
            tsp_sequence, detailed_sequence, node_waypoints, self.nodes,
            self._compute_node_raw_points, self._compute_repass_adjusted_exit)
    def execute_full_mission(self):
        """하위 알고리즘 모듈들을 오케스트레이션해서 전체 로봇 mission plan을 생성함."""
        print(f"\n{'-'*14} [Full Mission Planning Start] {'-'*14}")
        start_time = time.time()
        
        # [Step 1] TSP 순서 및 상세 경유 시퀀스 계산
        print("[Step 1/3] Calculating TSP and transit sequence...")
        tsp_sequence, detailed_sequence, planning_mask, node_waypoints, _connection_widths_px, _connection_masks = tsp.solve_tsp_sequence(
            nodes=self.nodes, global_mask=self.global_mask
        )
        
        # Transit A* 전용 글로벌 비용 지도(Cost Map) 생성
        print("[*] Generating Safe Cost Map for Transit Paths...")
        global_cost_map = self.generate_cost_map(self.global_mask)
        
        # [Step 2] 각 타겟 노드별 F2C 측정(Coverage) 경로 계산
        print("[Step 2/3] Generating F2C coverage paths...")
        width_bucket_counts = {'wide': 0, 'narrow': 0, 'ultra_narrow': 0}
        width_bucket_log = []  # (node_id, safe_width_px, bucket) - 필요 시 CSV로 dump 가능

        for node_idx in range(len(self.nodes)):
            # 원본 도면이 아닌 마진이 확보된 Coverage 전용 마스크 전달
            safe_node_mask = self.generate_coverage_mask(self.nodes[node_idx]['driveable_mask'])
            safe_width_px = geometry.estimate_min_width_px(safe_node_mask)

            node_id = self.nodes[node_idx]['id']
            contours, _ = cv2.findContours(safe_node_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if node_id == 2:
                print(f"[DEBUG] Node {node_id}의 safe_node_mask의 윤곽선 개수: {len(contours)}")
                if len(contours) > 1:
                    areas = [cv2.contourArea(c) for c in contours]
                    print(f"[DEBUG] 각 조각의 면적: {areas}")

            if contours:
                c = max(contours, key=cv2.contourArea)
                rect = cv2.minAreaRect(c)
                box_pts = cv2.boxPoints(rect).astype(int)

                debug_img = cv2.cvtColor(self.nodes[node_idx]['driveable_mask'], cv2.COLOR_GRAY2BGR)
                cv2.drawContours(debug_img, [box_pts], 0, (0, 0, 255), 2)
                cv2.drawContours(debug_img, [c], -1, (0, 255, 0), 1)
                cv2.putText(debug_img, f"node {node_id}: {safe_width_px:.1f}px", (10, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                width_debug_dir = os.path.join(self.visualization_dir, "width_debug")
                os.makedirs(width_debug_dir, exist_ok=True)
                cv2.imwrite(os.path.join(width_debug_dir, f"node_{node_id:03d}.png"), debug_img)

            assist_needed = safe_width_px < self.blind_radius_px # bucket 분류용으로만 사용
            bucket = ('ultra_narrow' if assist_needed
                    else 'narrow' if safe_width_px < 2 * self.blind_radius_px
                    else 'wide')
            width_bucket_counts[bucket] += 1
            width_bucket_log.append((node_id, round(safe_width_px, 1), bucket))

            self.nodes[node_idx]['bucket'] = bucket
            self.nodes[node_idx]['safe_node_mask'] = safe_node_mask

        # 너비 측정 로그
        print(f"\n[*] Node width classification (blind_radius_px={self.blind_radius_px:.1f}):")
        print(f"    wide={width_bucket_counts['wide']}, narrow={width_bucket_counts['narrow']}, "
            f"ultra_narrow={width_bucket_counts['ultra_narrow']}  (total={len(width_bucket_log)})")
        for node_id, w, bucket in sorted(width_bucket_log, key=lambda t: t[1]):
            print(f"      node {node_id:>3}: safe_width_px={w:>6}  -> {bucket}")

        # [Step 2.5] 허브형 토폴로지(중앙 복도 하나에 여러 방이 매달린 구조)의
        # pendant 방문 순서를 허브의 실제 coverage 종료 지점 기준으로 재정렬.
        # bucket/safe_node_mask가 막 채워진 직후라야 각 노드의 F2C 스와스를
        # 생성할 수 있어 여기(Step2 이후, Step3 이전)에서 수행함. 자세한
        # 이유는 _reorder_pendant_groups 참고.
        # ablation 토글: enable_pendant_reorder=False면 Christofides 근사가
        # 정한 순서를 그대로 두고 이 국소 재정렬을 건너뜀.
        if self.enable_pendant_reorder:
            tsp_sequence, detailed_sequence = self._reorder_pendant_groups(
                tsp_sequence, detailed_sequence, node_waypoints
            )

        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            from matplotlib.patches import Patch

            os.makedirs(self.visualization_dir, exist_ok=True)

            sorted_log = sorted(width_bucket_log, key=lambda t: t[1])
            node_labels = [f"node {nid}" for nid, _, _ in sorted_log]
            widths = [w for _, w, _ in sorted_log]
            buckets = [b for _, _, b in sorted_log]

            bucket_colors = {'wide': '#2ca02c', 'narrow': '#ff9900', 'ultra_narrow': '#d62728'}
            bar_colors = [bucket_colors[b] for b in buckets]

            fig, ax = plt.subplots(figsize=(max(8, len(sorted_log) * 0.9), 6))
            bars = ax.bar(range(len(sorted_log)), widths, color=bar_colors, edgecolor='black')

            for bar, w in zip(bars, widths):
                ax.text(bar.get_x() + bar.get_width() / 2, w + max(widths) * 0.015,
                        f"{w:.1f}", ha='center', va='bottom', fontsize=9)

            ax.set_xticks(range(len(sorted_log)))
            ax.set_xticklabels(node_labels, rotation=45, ha='right')
            ax.axhline(self.blind_radius_px, color='orange', linestyle='--',
                    label=f'blind_radius_px ({self.blind_radius_px:.0f})')
            ax.axhline(2 * self.blind_radius_px, color='red', linestyle='--',
                    label=f'2x blind_radius_px ({2 * self.blind_radius_px:.0f})')
            ax.set_ylabel('safe_width_px')
            ax.set_title('Node width classification (per-node)')

            bucket_handles = [Patch(facecolor=bucket_colors[b], edgecolor='black', label=b)
                            for b in ['wide', 'narrow', 'ultra_narrow']]
            line_handles, line_labels = ax.get_legend_handles_labels()
            ax.legend(handles=bucket_handles + line_handles, loc='upper left')

            summary_line = (f"wide={width_bucket_counts['wide']}, narrow={width_bucket_counts['narrow']}, "
                            f"ultra_narrow={width_bucket_counts['ultra_narrow']}  (total={len(width_bucket_log)})")
            fig.suptitle(summary_line, fontsize=10, y=0.98)

            detail_lines = [f"node {nid:>3}: safe_width_px={w:>6} -> {b}" for nid, w, b in sorted_log]
            fig.text(0.02, -0.02, "\n".join(detail_lines), fontsize=7, family='monospace', va='top')

            plt.tight_layout(rect=[0, 0.02, 1, 0.95])
            hist_path = os.path.join(self.visualization_dir, 'width_classification_histogram.png')
            plt.savefig(hist_path, dpi=150, bbox_inches='tight')
            plt.close()
            print(f"[*] Histogram saved to: {hist_path}")
        except ImportError:
            print("[WARN] matplotlib not available - counts above are still printed.")

        
        # [Step 3] 최종 궤적 생성 및 연결 (Transit via A*)
        print("[Step 3/3] Finalizing trajectory (Linking all paths)...")
        self.path_segments = [] 
        current_pos = None
        tsp_idx = 0 
        
        coverage_count = 0
        transit_count = 0

        def simplify_path(path, epsilon_px=3.0):
            """
            A* 8방향 격자 이동이 만드는 계단식(staircase) 지그재그를 제거하고 진짜
            꺾이는 지점만 남김. 격자는 임의 각도의 직선을 정확히 못 그리고 두
            방향을 번갈아 밟아 근사하는데, 이 계단 하나하나를 sampler.py의 앵커
            감지 로직이 '진짜 코너'로 착각하는 문제를 막기 위함임.
            """
            if len(path) < 3:
                return path
            arr = np.array(path, dtype=np.int32).reshape((-1, 1, 2))
            simplified = cv2.approxPolyDP(arr, epsilon_px, closed=False)
            return [tuple(map(int, pt[0])) for pt in simplified]

        for i in range(len(detailed_sequence)):
            curr_node = detailed_sequence[i]
            
            # 1. 측정(Coverage) 대상 노드 처리
            if tsp_idx < len(tsp_sequence) and curr_node == tsp_sequence[tsp_idx]:
                prev_node = detailed_sequence[i - 1] if i > 0 else None
                entry_hint = node_waypoints.get((prev_node, curr_node)) if prev_node is not None else None

                bucket = self.nodes[curr_node]['bucket']
                safe_node_mask = self.nodes[curr_node]['safe_node_mask']

                if self.coverage_mode == 'centroid_only':
                    # _compute_node_raw_points와 동일한 원리 - 스와스 생성을
                    # 건너뛰어 아래 centroid 폴백 경로를 그대로 재사용함
                    forced_angle = None
                    swath_pairs = []
                elif bucket in ('narrow', 'ultra_narrow'):
                    forced_angle = geometry.get_long_axis_angle_rad(safe_node_mask)
                    swath_pairs = coverage.generate_raw_swaths(safe_node_mask, self.robot_params, decompose=True, split_angle_rad=forced_angle)
                else:
                    forced_angle = None
                    swath_pairs = coverage.generate_raw_swaths(safe_node_mask, self.robot_params, enable_optimal_swath_angle=self.enable_optimal_swath_angle)

                if self.nodes[curr_node]['id'] in (1, 2):
                    node_id_dbg = self.nodes[curr_node]['id']
                    angle_str = f"{math.degrees(forced_angle):.1f}" if forced_angle is not None else "N/A(wide)"
                    print(f"[DEBUG] Node {node_id_dbg}: forced_angle_deg={angle_str}, "
                        f"num_swaths={len(swath_pairs)}, swath_pairs={swath_pairs}")

                    debug_img = cv2.cvtColor(safe_node_mask, cv2.COLOR_GRAY2BGR)
                    contours, _ = cv2.findContours(safe_node_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    cv2.drawContours(debug_img, contours, -1, (0, 255, 0), 1)  # 초록: 폴리곤 전체
                    for p1, p2 in swath_pairs:
                        cv2.line(debug_img, p1, p2, (0, 0, 255), 2)   # 빨강: 실제 생성된 스와스
                        cv2.circle(debug_img, p1, 4, (255, 0, 0), -1)  # 파랑: 스와스 시작점
                        cv2.circle(debug_img, p2, 4, (0, 255, 255), -1)  # 노랑: 스와스 끝점

                    dbg_dir = os.path.join(self.visualization_dir, "width_debug")
                    os.makedirs(dbg_dir, exist_ok=True)
                    cv2.imwrite(os.path.join(dbg_dir, f"node_{node_id_dbg:03d}_swaths.png"), debug_img)  # 파일명에 id 반영
                    print(f"[DEBUG] Saved node_{node_id_dbg:03d}_swaths.png")

                raw_points = []
                if swath_pairs:
                    # 미션의 첫 coverage 노드는 진입 기준점이 없음 - 대신 다음 노드로
                    # 나가는 출구 방향에 최대한 가깝게 '끝나도록' exit_hint를 역산해
                    # 넘김(_order_swaths가 통째로 뒤집어 처리).
                    exit_hint = None
                    if entry_hint is None:
                        next_node = detailed_sequence[i + 1] if i < len(detailed_sequence) - 1 else None
                        exit_hint = node_waypoints.get((curr_node, next_node)) if next_node is not None else None

                    ordered_pairs = self._order_swaths(swath_pairs, entry_hint, exit_hint)

                    for p1, p2 in ordered_pairs:
                        raw_points.extend([p1, p2])
                else:
                    centroid = geometry.get_centroid(self.nodes[curr_node]['driveable_mask'])
                    if centroid: raw_points.append(centroid)
                
                if raw_points:
                    # 진입/진출 힌트 확보: detailed_sequence 상에서 이 노드의 바로 앞/뒤
                    # 노드와의 연결 지점(waypoints, tsp.py의 extract_waypoints가 계산한
                    # '문지방 등 안전 통과 지점'). 두 노드가 항상 그래프 상 인접하도록
                    # detailed_sequence가 Dijkstra 최단경로로 구성되어 있으므로, 이
                    # 조회는 항상 유효한 값을 반환함(첫/마지막 노드의 바깥쪽 방향 제외).

                    # 방향 최적화: 진입점 근접성뿐 아니라 진출점(다음 노드로 가는 방향)까지
                    # 함께 고려해서 정방향/역방향을 선택함. 이렇게 해야 예를 들어 방 A ->
                    # 복도 B -> 복도 C로 이동할 때, B의 coverage가 A쪽에서 들어와서 C쪽으로
                    # 나가도록 자연스럽게 정렬되어 불필요한 되돌아가기(우회 transit)가 줄어듦.

                    # 노드 진입 경로 (Transit) 계산 (A* 알고리즘)
                    if current_pos:
                        via_point = entry_hint  # curr_node로 들어가는 연결부의 로컬 중심점
                        enter_path = []

                        if via_point is not None and via_point != current_pos:
                            print(f"[TRACE_ENTER] Leg 1 (via doorway center): {current_pos} -> {via_point}")
                            _, leg1 = transit.find_path_with_penalty(
                                start=current_pos, goal=via_point, planning_mask=global_cost_map,
                                turn_weight=self.turn_weight, wall_weight=self.wall_weight
                            )
                            if leg1:
                                enter_path.extend(leg1)
                            else:
                                print(f"[WARN] Leg1 (doorway center 경유) 실패 - 직접 경로로 폴백.")

                        leg2_start = enter_path[-1] if enter_path else current_pos
                        print(f"[TRACE_ENTER] Leg 2: {leg2_start} -> {raw_points[0]}")
                        _, leg2 = transit.find_path_with_penalty(
                            start=leg2_start, goal=raw_points[0], planning_mask=global_cost_map,
                            turn_weight=self.turn_weight, wall_weight=self.wall_weight
                        )

                        if leg2:
                            if enter_path and enter_path[-1] == leg2[0]:
                                full_enter_path = enter_path + leg2[1:]
                            else:
                                full_enter_path = enter_path + leg2

                            if self.enable_path_simplification:
                                full_enter_path = simplify_path(full_enter_path, epsilon_px=3.0)  # A* 지그재그 제거는 유지
                            self.path_segments.append({
                                'type': 'transit', 'path': full_enter_path, 'record_pcd': False,
                                'from_node_id': self.nodes[prev_node]['id'] if prev_node is not None else None,
                                'to_node_id': self.nodes[curr_node]['id'],
                            })
                            transit_count += 1
                        else:
                            print(f"[ERROR] Cannot find safe path to Node {curr_node+1}. Wall detected!")
                    
                    # 측정 경로 추가 - coverage 자체는 F2C 스와스 그대로
                    # 저장함(raw_points, 종료 지점은 여전히 raw_points[-1]).
                    self.path_segments.append({
                        'type': 'coverage', 'path': raw_points, 'record_pcd': True,
                        'node_id': self.nodes[curr_node]['id'],
                    })
                    coverage_count += 1
                    # 다음 노드로 가는 transit(Leg1)은 F2C 종료 지점이 아니라
                    # exit repass가 끝난 뒤 로봇이 실제로 있을 위치에서
                    # 시작해야 함 - 안 그러면 오프라인 planning/시각화가 "repass가
                    # 없는 것처럼" coverage 끝점에서 곧장 transit이 이어지는
                    # 것으로 그려지는데, 실제로는 그 사이에 되짚기 왕복이 있음
                    # repass_preview 화살표
                    # (_build_boundary_repass_preview)가 raw_points[-1]->이
                    # 지점 구간을 시각적으로 이어줌.
                    current_pos = self._compute_repass_adjusted_exit(raw_points)
                
                print(f"    -> Completed Coverage task in Node {curr_node+1}")
                tsp_idx += 1 
            else:
                print(f"    -> Transiting through Node {curr_node+1}")
            
        total_time = time.time() - start_time
        print(f"\n[DEBUG] Path Segments Created: Coverage({coverage_count}), Transit({transit_count})")
        print(f"{'-'*20} [Planning Completed in {total_time:.2f}s] {'-'*20}\n")

        return tsp_sequence

    def _build_boundary_repass_preview(self):
        """repass_preview.build_preview()에 현 설정값을 넘겨 시각화 전용
        세그먼트 목록을 받아옴. self.path_segments(=final_path.json 원본)는
        건드리지 않음."""
        return repass_preview.build_preview(
            self.path_segments, self.enable_boundary_repass,
            self.boundary_repass_distance_m, self.map_resolution,
            self.boundary_repass_max_segment_m)

    def plan(self, save_debug=True, show_plot=False, output_dir=None):
        # output_dir 미지정 시, 현재 작업 디렉토리(cwd)에 의존하는 상대경로
        # "analytics/metrics" 대신 외부 저장소(workspace_root) 기준 절대경로로 fallback.
        if output_dir is None:
            default_workspace_root = os.path.expanduser("~/dae_floor_maps")
            output_dir = os.path.join(default_workspace_root, "analytics/metrics")
            print(f"[WARN] 'output_dir' not provided to plan(). Falling back to: {output_dir}")

        # 1. 전역 미션 planning 실행 (Pixel 단위 경로 생성)
        self.execute_full_mission()

        # 2. 경로 생성 실패 시 예외 처리
        if not self.path_segments:
            print("[WARN] No path generated. Mission aborted.")
            return None
        
        # 3. 결과 시각화 (Visualizer)
        # boundary_repass 미리보기는 시각화 전용 목록에만 추가 - self.path_segments
        # 자체(translator로 넘어가 final_path.json이 되는 원본)는 그대로 둠.
        viz_path_segments = self.path_segments + self._build_boundary_repass_preview()

        if save_debug:
            print(f"[*] Saving debug visualization to centralized storage...")
            os.makedirs(self.visualization_dir, exist_ok=True)
            visualizer.save_debug_image(
                nodes=self.nodes,
                path_segments=viz_path_segments,
                global_mask=self.global_mask,
                output_dir=self.visualization_dir,
                filename="full_mission_path.png",
                map_resolution=self.map_resolution
            ) # full_mission_path.png 저장

        if show_plot:
            print("[*] Displaying mission state on screen.")
            visualizer.plot_mission_state(
                nodes=self.nodes,
                path_segments=viz_path_segments,
                global_mask=self.global_mask,
                map_resolution=self.map_resolution
            )

        # 4. 미터 좌표 변환 -> 샘플링 -> json/시각화 저장(path_exporter.py)
        return path_exporter.export(self, output_dir, save_debug=save_debug)
