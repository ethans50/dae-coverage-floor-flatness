# surface_profiling/capture_services_mixin.py
"""
SurfaceProfiler의 ROS 서비스 서버 담당 믹스인 - mission_executor.py(Jetson)가
보내는 트리거를 받는 쪽.

두 종류를 다룸 - (1) 미션 종료 트리거(정상 완료/중단)로 수집 루프를 멈추고
후처리 단계로 넘어가게 함, (2) 지점별 캡처 시작/종료 트리거로 정지-캡처 구간의
포인트만 따로 모아 waypoint 단위로 저장함.

캡처 시작 시 TF 저역통과 기준값(`last_tf_*`)을 리셋하는 것이 중요함 - 리셋하지
않으면 직전 기준 프레임과의 거리가 통째로 "순간 속도"로 잡혀 캡처 초반 프레임이
전부 기각됨.

믹스인으로 둔 이유는 `tf_sync_mixin.py`와 같음 - `capture_active`,
`current_waypoint_points`, `stop_requested`, `last_tf_*` 등 SurfaceProfiler의
상태를 그대로 읽고 씀.
"""

import os
import time

import numpy as np
import open3d as o3d
from std_srvs.srv import Trigger


class CaptureServicesMixin:
    """미션 종료/지점별 캡처 트리거 서비스 서버. SurfaceProfiler에 믹스인함."""

    # ------------------------------------------------------------------
    # 종료 트리거 서비스 서버
    # ------------------------------------------------------------------

    def _setup_stop_services(self):
        self.stop_success_srv = self.create_service(
            Trigger, '/surface_profiling/stop_collection_success', self._handle_stop_success
        )
        self.stop_abort_srv = self.create_service(
            Trigger, '/surface_profiling/stop_collection_abort', self._handle_stop_abort
        )
        self.get_logger().info(
            "Stop-trigger services ready: "
            "/surface_profiling/stop_collection_success, /surface_profiling/stop_collection_abort"
        )

    def _handle_stop_success(self, request, response):
        print("[*] Received STOP signal (mission succeeded). Stopping collection...")
        self.is_aborted = False
        self.stop_requested = True
        response.success = True
        response.message = "Collection stopped (mission succeeded)."
        return response

    def _handle_stop_abort(self, request, response):
        print("[!] Received ABORT signal (mission failed/canceled/timed out). "
              "Stopping collection immediately. Collected points so far will still be saved.")
        self.is_aborted = True
        self.stop_requested = True
        response.success = True
        response.message = "Collection aborted; partial data will be saved."
        return response

    # ------------------------------------------------------------------
    # 지점별(Waypoint) 캡처 시작/종료 트리거 서비스
    # ------------------------------------------------------------------

    def _setup_waypoint_capture_services(self):
        self.start_capture_srv = self.create_service(
            Trigger, '/surface_profiling/start_waypoint_capture', self._handle_start_waypoint_capture
        )
        self.stop_capture_srv = self.create_service(
            Trigger, '/surface_profiling/stop_waypoint_capture', self._handle_stop_waypoint_capture
        )
        self.get_logger().info(
            "Waypoint capture services ready: "
            "/surface_profiling/start_waypoint_capture, /surface_profiling/stop_waypoint_capture"
        )

    def _handle_start_waypoint_capture(self, request, response):
        # 방어 코드: start/stop은 항상 쌍으로 불리는 게 프로토콜상 전제지만,
        # 혹시 stop 없이 start가 재호출되면 미저장 포인트를 버리지 않고
        # 먼저 flush함.
        if self.capture_active and self.current_waypoint_points:
            print(f"[!] Warning: start_waypoint_capture re-invoked while capture "
                  f"#{self.waypoint_capture_counter} was still active - flushing pending buffer first.")
            self._handle_stop_waypoint_capture(request, Trigger.Response())

        # 새 캡처 구간을 위해 지점 전용 버퍼를 비우고 적재를 켬.
        self.current_waypoint_points = []
        self.capture_active = True
        self.waypoint_capture_counter += 1

        # TF 저역통과 필터 기준값 리셋: mission_executor는 로봇이 목표 지점에
        # 완전히 정지한 뒤에만 이 서비스를 호출하므로, 직전 기준 프레임이
        # 아무리 멀리/오래 전(이전 waypoint, 심지어 이전 미션의 마지막
        # 위치 - sim에서 재시작 없이 여러 미션을 연달아 돌리면 흔함)의
        # 것이었어도 지금부터는 리셋하는 게 안전함. 안 그러면 그 사이의
        # 실제 이동 거리가 통째로 "순간 속도"로 계산되어 임계치를 넘고,
        # "통과한 프레임에서만 기준 갱신" 규칙 때문에 몇 초간 모든 프레임이
        # 기각됨.
        self.last_tf_translation = None
        self.last_tf_yaw = None
        self.last_tf_stamp = None

        print(f"[*] [Waypoint #{self.waypoint_capture_counter}] Capture window OPENED.")
        response.success = True
        response.message = f"Capture started (waypoint_id={self.waypoint_capture_counter})"
        return response

    def _handle_stop_waypoint_capture(self, request, response):
        self.capture_active = False
        captured = self.current_waypoint_points
        self.current_waypoint_points = []

        point_count = sum(arr.shape[0] for arr in captured) if captured else 0
        print(f"[*] [Waypoint #{self.waypoint_capture_counter}] Capture window CLOSED. "
              f"Points captured: {point_count}")

        if point_count == 0:
            print(f"[!] Warning: Waypoint #{self.waypoint_capture_counter} captured 0 points "
                  f"(TF/토픽 타이밍 문제이거나 정지 시간이 너무 짧을 수 있음).")
            response.success = True
            response.message = f"Capture stopped (waypoint_id={self.waypoint_capture_counter}, points=0)"
            return response

        waypoint_points_np = np.vstack(captured)

        # 1. 지점별 개별 PCD로 저장 (추후 지점 단위 재처리/디버깅용).
        #    반복 측정 캠페인에서는 용량이 커서 save_waypoint_pcd로 끌 수 있음.
        if self.profiling_cfg.get('save_waypoint_pcd', False):
            ts = time.strftime("%Y-%m-%d_%H-%M-%S")
            waypoint_pcd = o3d.geometry.PointCloud()
            waypoint_pcd.points = o3d.utility.Vector3dVector(waypoint_points_np)
            filename = self.WAYPOINT_PCD_FILENAME_TEMPLATE.format(
                idx=self.waypoint_capture_counter, ts=ts
            )
            filepath = os.path.join(self.waypoint_pointcloud_dir, filename)
            o3d.io.write_point_cloud(filepath, waypoint_pcd)
            print(f"[+] Saved per-waypoint PCD: {filepath}")

        # 2. 최종 통합 히트맵을 위해 마스터 버퍼에도 병합.
        #    only_capture_at_waypoints=False 모드에서는 _pc_callback이 이미
        #    all_points에도 동시 적재했으므로, 여기서 다시 넣으면 중복이라 생략함.
        if self.only_capture_at_waypoints:
            self.all_points.append(waypoint_points_np)

        response.success = True
        response.message = f"Capture stopped (waypoint_id={self.waypoint_capture_counter}, points={point_count})"
        return response
