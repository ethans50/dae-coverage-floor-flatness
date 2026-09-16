# surface_profiling/surface_profiler.py

import os
import sys
import time

import yaml
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
import open3d as o3d
import numpy as np

try:
    from utils.floor_extractor import extract_floor_by_height
    from utils.heatmap_generator import generate_floor_heatmap
    from utils.pcd_io import export_pcd_to_csv
    from utils.stall_report_analyzer import analyze_and_visualize_stalls
    from utils.config_paths import (
        load_full_config as _load_full_config_shared,
        resolve_pointcloud_dir,
        resolve_visualization_dir,
        resolve_map_yaml_path,
    )
    from tf_sync_mixin import TfSyncMixin
    from capture_services_mixin import CaptureServicesMixin
except ImportError:
    from surface_profiling.utils.floor_extractor import extract_floor_by_height
    from surface_profiling.utils.heatmap_generator import generate_floor_heatmap
    from surface_profiling.utils.pcd_io import export_pcd_to_csv
    from surface_profiling.utils.stall_report_analyzer import analyze_and_visualize_stalls
    from surface_profiling.utils.config_paths import (
        load_full_config as _load_full_config_shared,
        resolve_pointcloud_dir,
        resolve_visualization_dir,
        resolve_map_yaml_path,
    )
    from surface_profiling.tf_sync_mixin import TfSyncMixin
    from surface_profiling.capture_services_mixin import CaptureServicesMixin


class SurfaceProfiler(TfSyncMixin, CaptureServicesMixin, Node):
    """
    3D라이다와 랜선으로 직결되어 pcd를 처리하는 노트북에서 구동되는 바닥 평탄도 측정 파이프라인 진입점 ROS 2 노드.

    1단계 (수집): Velodyne VLP-16 PointCloud2를 구독하며 TF('map' <- 'velodyne_link')를
                  통해 전역 좌표계로 정렬, 종료 신호를 받을 때까지 누적 후 PCD로 저장함.
                  TF는 같은 ROS2 도메인 내에서 Jetson(주행 시스템)이 publish하는 좌표 정보를
                  네트워크를 통해 직접 구독하는 구조이며,
                  Chrony 기반 시간 동기화 정밀도에 timestamp 매칭 신뢰도가 의존함.
    2단계 (바닥 추출): 높이(z) 기준으로 바닥면 후보 포인트만 필터링.
    3단계 (시각화): 필터링된 바닥면 데이터를 X-Y 격자 평균 높이로 변환해 히트맵 PNG 생성.
    4단계 (주행 지연 분석): 히트맵까지 다 만든 직후, mission_executor.py(Jetson)가
                  같은 미션에서 남긴 stall_report_<ts>.csv(utils/stall_logger.py)를
                  robot_path_<ts>.csv 궤적과 좌표로 대조해 분석 요약(analytics/logs)과
                  위치 시각화(visualization/mission_execution)를 자동 생성함
                  (utils/stall_report_analyzer.py). 실패해도 1~3단계 결과에는 영향 없음.

    종료 트리거: ENTER 입력 대신, mission_executor.py(Jetson)가 호출하는 두 개의
    std_srvs/Trigger 서비스로 종료를 제어함.
      - /surface_profiling/stop_collection_success : 미션 정상 완료. 수집을 멈추고
        2, 3단계까지 정상적으로 진행함.
      - /surface_profiling/stop_collection_abort : 미션 비정상 종료(타임아웃/실패/취소). 즉시 수집을 멈추되,
      그때까지 모은 포인트는 저장하고 2, 3단계도 동일하게 진행함. 이때 파일명에 '_aborted'라고 붙여 불완전한 데이터라는 뜻으로 저장함.

    지점별(Waypoint) 캡처: coverage 웨이포인트에서 로봇이 정지하는 동안만 정밀 데이터를
    모으기 위해, mission_executor.py가 호출하는 시작/종료 한 쌍의 std_srvs/Trigger
    서비스로 캡처 구간(on/off)을 제어함. 정착 대기(settling)와 실제 캡처 시간(active
    capture) 관리는 전부 mission_executor(주행 측)가 전담하며, 이 노드는 신호에만 반응함
    (주행과 측정의 기능적 분리 원칙 유지).
      - /surface_profiling/start_waypoint_capture : 이 시점부터 들어오는 포인트를
        지점 전용 버퍼(self.current_waypoint_points)에 적재하기 시작함.
      - /surface_profiling/stop_waypoint_capture : 적재를 멈추고, 모인 포인트를
        해당 지점 전용 PCD 파일로 저장한 뒤, 최종 통합 히트맵을 위해 마스터 버퍼
        (self.all_points)에도 병합함.

    params.yaml의 surface_profiling.only_capture_at_waypoints (기본값 True)로 동작을
    전환할 수 있음: True면 정지-캡처 구간의 포인트만 최종 결과에 반영하고(이동 중
    포인트는 모션 블러 우려로 버림), False면 기존처럼 전 구간을 연속 수집하되
    지점별 캡처 파일은 부가적으로만 별도 저장함.

    부가 기능: 필요 시 PCD를 CSV로 변환하는 보조 유틸(utils.pcd_io)을 제공.
    
    ROS 2 Node 내장 executor와의 이름 충돌 회피:
    - self.spin_executor
    - Node.executor는 read-only property로 존재하므로 직접 할당 불가
    - Python Descriptor Protocol에 의해 할당 시도가 무시되고 None이 됨
    """

    PCD_FILENAME_TEMPLATE = "combined_{ts}{suffix}.pcd"
    PCD_RAW_FILENAME_TEMPLATE = "combined_raw_{ts}{suffix}.pcd"
    PCD_FILTERED_FILENAME_TEMPLATE = "combined_filtered_{ts}{suffix}.pcd"
    CSV_FILENAME_TEMPLATE = "combined_{ts}{suffix}.csv"
    HEATMAP_FILENAME_TEMPLATE = "floor_heatmap_{ts}{suffix}.png"
    WAYPOINT_PCD_FILENAME_TEMPLATE = "waypoint_{idx:04d}_{ts}.pcd"

    def __init__(self):
        super().__init__('surface_profiler_node')

        print("\n=======================================================")
        print("[*] Surface Profiler (3D LiDAR Floor Flatness Measurement)")
        print("=======================================================\n")

        # self.spin_executor
        # self 전용 SingleThreadedExecutor. 이 노드는 ROS 통신을 전부 직접 처리하는
        # 단일 노드이므로(mission_executor.py처럼 별도 navigator Node가 없음),
        # executor 분리 없이 self 하나만 등록.
        self.spin_executor = SingleThreadedExecutor()
        self.spin_executor.add_node(self)

        # 상태 변수 초기화
        self.config = None
        self.global_cfg = None
        self.profiling_cfg = None
        self.mission_exec_cfg = None
        self.workspace_root = None

        self.pointcloud_dir = None
        self.visualization_dir = None
        self.map_yaml_path = None
        self.output_root = None

        # 수집 종료 제어 상태
        self.stop_requested = False
        self.is_aborted = False
        self.collection_start_ts = None  # 'YYYY-MM-DD_HH-MM-SS' 형식, 파일명 공용 타임스탬프

        # 포인트클라우드 누적 버퍼 (최종 히트맵용 마스터 버퍼)
        self.all_points = []

        # 지점별(Waypoint) 캡처 관련 상태.
        # capture_active=True인 동안에만 _pc_callback이 포인트를
        # self.current_waypoint_points에 적재함(정지 상태에서 모은 깨끗한
        # 데이터만 반영하기 위함). only_capture_at_waypoints 실제 값은
        # _load_config()에서 profiling_cfg를 읽은 뒤 갱신됨.
        self.capture_active = False
        self.waypoint_capture_counter = 0
        self.current_waypoint_points = []
        self.waypoint_pointcloud_dir = None  # _resolve_directories()에서 설정
        self.only_capture_at_waypoints = True

        # TF 저역통과 필터 상태. 연속된 두 PointCloud2 프레임 사이의 순간
        # 선속도/각속도가 임계치(params.yaml)를 넘으면 해당 프레임을 통째로
        # 버림. 실제 값(enable/threshold)은 _load_config()에서 갱신됨.
        self.enable_tf_lowpass_filter = True
        self.tf_lowpass_max_linear_vel = 0.3       # m/s
        self.tf_lowpass_max_angular_vel_deg = 20.0  # deg/s
        self.last_tf_translation = None
        self.last_tf_yaw = None
        self.last_tf_stamp = None

        # 1. ROS 2 파라미터로 시뮬레이션 모드 여부 확보 (launch argument로 주입됨)
        self._resolve_run_mode()

        # 2. params.yaml 설정 파일 파싱
        self._load_config()

        # 3. 외부 저장소 디렉토리 확보
        self._resolve_directories()

        # 4. TF, PointCloud2 구독, 종료/캡처 트리거 서비스 서버 설정
        self._setup_tf()
        self._setup_pointcloud_subscription()
        self._setup_stop_services()
        self._setup_waypoint_capture_services()

    # ------------------------------------------------------------------
    # 초기화 단계 헬퍼
    # ------------------------------------------------------------------

    def _resolve_run_mode(self):
        """
        'is_sim' ROS 2 파라미터를 선언하고 읽음. launch 파일이 주입하지 않으면
        기본값 False(Real-world)로 동작.
        """
        self.declare_parameter('is_sim', False)
        self.is_sim = self.get_parameter('is_sim').get_parameter_value().bool_value

        # EVAL.md 알고리즘 비교 실험용 - launch argument로 라벨을 주면 이번
        # 수집물(pcd/heatmap)을 <workspace_root>/eval_runs/<라벨>/ 아래로 모아
        # 저장함. 빈 문자열(기본값)이면 기존처럼 flat 경로에 저장함 -
        # _resolve_directories 참고. map_yaml_path(입력, 오버레이용 공유 맵)는
        # 이 라벨과 무관하게 항상 실제 workspace_root를 그대로 씀.
        self.declare_parameter('eval_run_label', '')
        self.eval_run_label = self.get_parameter('eval_run_label').get_parameter_value().string_value

        # mission_executor.py(Jetson)와 이 노드(노트북)의 산출물 파일명
        # 타임스탬프를 맞추기 위한 공유 값. 양쪽 launch에 같은 값을 넘기면
        # 자기 collection 시작 시각 대신 이 문자열을 씀. 빈 문자열이면
        # 기존처럼 자체 time.strftime()을 씀(DETAILS.md §3 참고).
        self.declare_parameter('run_ts', '')
        self.run_ts_param = self.get_parameter('run_ts').get_parameter_value().string_value

        if self.is_sim:
            self.get_logger().info("Running Simulation(Gazebo) Mode.")
        else:
            self.get_logger().info("Running Real-world Mode.")

        # use_sim_time 강제 동기화 (launch 파일이 누락했을 경우 안전장치)
        use_sim_time_param = self.get_parameter_or(
            'use_sim_time', Parameter('use_sim_time', Parameter.Type.BOOL, self.is_sim)
        )
        if use_sim_time_param.value != self.is_sim:
            self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, self.is_sim)])

    def _load_config(self):
        # [변경] 경로 해석 로직을 utils.config_paths.load_full_config()로 일원화.
        # reprocess_pcd.py는 여전히 load_config()(surface_profiling 섹션만)를 쓰므로
        # 서로 다르게 해석하는 문제는 재발하지 않음. 여기서 load_full_config를
        # 쓰는 이유는 Stage 4(stall 분석)가 mission_execution 섹션(stall_log_dir/
        # output_path_dir 등)도 읽어야 하기 때문 - profiling_cfg는 그대로 전체
        # config에서 동일하게 잘라내 쓰므로 기존 동작과 차이 없음.
        self.workspace_root, self._full_config = _load_full_config_shared()
        self.profiling_cfg = self._full_config.get('surface_profiling', {})
        self.mission_exec_cfg = self._full_config.get('mission_execution', {})
        self.only_capture_at_waypoints = self.profiling_cfg.get('only_capture_at_waypoints', True)
        self.get_logger().info(
            f"only_capture_at_waypoints = {self.only_capture_at_waypoints} "
            f"({'정지-캡처 구간만 반영' if self.only_capture_at_waypoints else '연속 수집 + 지점별 부가 저장'})"
        )

        self.enable_tf_lowpass_filter = self.profiling_cfg.get('enable_tf_lowpass_filter', True)
        self.tf_lowpass_max_linear_vel = self.profiling_cfg.get('tf_lowpass_max_linear_vel', 0.3)
        self.tf_lowpass_max_angular_vel_deg = self.profiling_cfg.get('tf_lowpass_max_angular_vel_deg', 20.0)
        self.get_logger().info(
            f"TF Low-pass Filter: enabled={self.enable_tf_lowpass_filter}, "
            f"max_linear_vel={self.tf_lowpass_max_linear_vel}m/s, "
            f"max_angular_vel={self.tf_lowpass_max_angular_vel_deg}deg/s"
        )

    def _resolve_directories(self):
        # [변경] 경로 조합 로직을 utils.config_paths로 일원화.
        # 예전에는 여기서 map_yaml_dir을 완전한 파일 경로로 조합해뒀는데,
        # _run_visualization_stage()가 이를 쓰지 않고 profiling_cfg에서
        # 원본 값("maps/grid", 폴더명만)을 다시 읽어버려서 서로 어긋나는
        # 버그가 있었음(HISTORY.md 참고). resolve_map_yaml_path()가 조합한
        # 값을 self.map_yaml_path에 저장해두고, 아래 단계들이 전부 이
        # 하나의 값만 참조하도록 통일함.
        # pointcloud_dir/visualization_dir는 순수 출력 전용이라 라벨이 있으면
        # eval_runs/<라벨> 아래로 리다이렉트해도 안전하지만, map_yaml_path는
        # 다른 실행(계획 단계)이 만든 공유 입력이라 항상 실제 workspace_root를
        # 그대로 참조해야 함 - 그래서 output_root만 따로 계산해서 씀.
        self.output_root = (
            os.path.join(self.workspace_root, 'eval_runs', self.eval_run_label)
            if self.eval_run_label else self.workspace_root
        )
        self.pointcloud_dir = resolve_pointcloud_dir(self.output_root, self.profiling_cfg)
        self.visualization_dir = resolve_visualization_dir(self.output_root, self.profiling_cfg)
        self.map_yaml_path = resolve_map_yaml_path(self.workspace_root, self.profiling_cfg)

        # 지점별(Waypoint) 캡처 PCD 저장용 서브폴더. 추후 지점 단위 재처리가
        # 가능하도록 combined PCD와 분리해서 보관함.
        self.waypoint_pointcloud_dir = os.path.join(self.pointcloud_dir, 'waypoints')
        os.makedirs(self.waypoint_pointcloud_dir, exist_ok=True)

    # ------------------------------------------------------------------
    # 1단계: 수집 (메인 spin 루프)
    # ------------------------------------------------------------------

    def _run_collection_stage(self):
        print("\n[Stage 1/4] Point Cloud Collection")
        print("[*] Waiting for stop signal from mission_executor.py (Jetson)...")

        if self.run_ts_param:
            self.collection_start_ts = self.run_ts_param
            print(f"[*] run_ts='{self.run_ts_param}' - 이 값을 combined/raw pcd 파일명에 그대로 씀.")
        else:
            self.collection_start_ts = time.strftime("%Y-%m-%d_%H-%M-%S")

        try:
            while rclpy.ok() and not self.stop_requested:
                self.spin_executor.spin_once(timeout_sec=0.1)

            # STOP 신호 직후에도 TF 도착을 기다리던 프레임들(_pending_pc_msgs)이
            # 아직 몇 개 남아있을 수 있으므로, 짧게 마저 처리되도록 유예 시간을 줌.
            # (최대 tf_timeout_sec * 2 정도, 그 이상은 어차피 stop_requested로
            #  새 콜백 등록이 막혀 있으니 무한 대기로 이어지지 않음.)
            grace_deadline = time.time() + max(self.tf_timeout_sec * 2, 1.0)
            while self._pending_pc_msgs and time.time() < grace_deadline:
                self.spin_executor.spin_once(timeout_sec=0.05)
        except KeyboardInterrupt:
            print("[!] KeyboardInterrupt received. Stopping collection...")
            self.is_aborted = True

        suffix = "_aborted" if self.is_aborted else ""
        pcd_filename = self.PCD_FILENAME_TEMPLATE.format(ts=self.collection_start_ts, suffix=suffix)
        pcd_path = self._save_combined_pcd(pcd_filename=pcd_filename)

        if pcd_path is None:
            print("[!] CRITICAL ERROR: No points were collected. Aborting pipeline.")
            self._shutdown()
            sys.exit(1)

        return pcd_path

    def _save_combined_pcd(self, pcd_filename):
        """누적된 포인트들을 다운샘플링 후 단일 PCD 파일로 저장하고 경로를 반환함."""
        if not self.all_points:
            print("[-] No points collected.")
            return None

        print("[*] Stacking all frames...")
        all_points_np = np.vstack(self.all_points)
        print(f"[*] Total points collected: {all_points_np.shape[0]}")

        combined_pcd = o3d.geometry.PointCloud()
        combined_pcd.points = o3d.utility.Vector3dVector(all_points_np)

        if self.save_raw_pcd:
            suffix = "_aborted" if self.is_aborted else ""
            raw_filename = self.PCD_RAW_FILENAME_TEMPLATE.format(ts=self.collection_start_ts, suffix=suffix)
            raw_file = os.path.join(self.pointcloud_dir, raw_filename)
            o3d.io.write_point_cloud(raw_file, combined_pcd)
            print(f"[+] Saved raw (pre-downsample) PCD: {raw_file}")

        print(f"[*] Downsampling PCD with voxel size: {self.voxel_size}m...")
        downsampled_pcd = combined_pcd.voxel_down_sample(voxel_size=self.voxel_size)

        pcd_file = os.path.join(self.pointcloud_dir, pcd_filename)
        o3d.io.write_point_cloud(pcd_file, downsampled_pcd)
        print(f"[+] Saved combined PCD: {pcd_file}")
        return pcd_file

    # ------------------------------------------------------------------
    # 2단계: 바닥 추출
    # ------------------------------------------------------------------

    def _run_floor_extraction_stage(self, pcd_path):
        print("\n[Stage 2/4] Floor Extraction (Height-based Filtering)")

        # z_min/z_max: 바닥으로 간주할 높이 밴드. params.yaml에 없으면
        # 바닥 요철이 보통 수 cm 이내인 것을 감안한 기본값 사용.
        z_min = self.profiling_cfg.get('z_min', -0.005)
        z_max = self.profiling_cfg.get('z_max', 0.035)
        suffix = "_aborted" if self.is_aborted else ""
        filtered_filename = self.PCD_FILTERED_FILENAME_TEMPLATE.format(ts=self.collection_start_ts, suffix=suffix)
        filtered_path = os.path.join(self.pointcloud_dir, filtered_filename)

        extract_floor_by_height(pcd_path, filtered_path, z_min=z_min, z_max=z_max)
        return filtered_path

    # ------------------------------------------------------------------
    # 3단계: 히트맵 시각화
    # ------------------------------------------------------------------

    def _run_visualization_stage(self, filtered_pcd_path):
        print("\n[Stage 3/4] Floor Flatness Heatmap Generation")

        grid_size = self.profiling_cfg.get('grid_size', 0.02)
        # 색상 스케일의 절대 범위. floor_extractor의 z_min/z_max와
        # 동일한 값을 쓰는 것을 권장 (필터링 범위 = 색상 표현 범위).
        z_min = self.profiling_cfg.get('z_min', -0.005)
        z_max = self.profiling_cfg.get('z_max', 0.035)
        # [수정] profiling_cfg.get('map_yaml_dir', ...)로 다시 읽으면 폴더명만
        # 있는 미완성 값("maps/grid")이 그대로 들어가 파일을 못 찾는 버그가
        # 있었음. _resolve_directories()가 이미 완전한 파일 경로로 조합해둔
        # self.map_yaml_path 하나만 참조하도록 통일.
        map_yaml_path = self.map_yaml_path
        suffix = "_aborted" if self.is_aborted else ""
        img_filename = self.HEATMAP_FILENAME_TEMPLATE.format(ts=self.collection_start_ts, suffix=suffix)
        img_out_path = os.path.join(self.visualization_dir, img_filename)

        generate_floor_heatmap(
            filtered_pcd_path,
            img_out_path,
            grid_size=grid_size,
            z_min=z_min,
            z_max=z_max,
            map_yaml_dir=map_yaml_path,
        )
        return img_out_path

    # ------------------------------------------------------------------
    # PCD -> CSV 변환
    # ------------------------------------------------------------------

    def export_csv(self, pcd_path=None):
        """PCD를 CSV로 변환하는 기능. run()의 파이프라인 마지막 단계로 항상 호출되지만,
        필요 시 외부에서 단독으로도 호출할 수 있음.

        save_combined_csv가 false면 건너뜀 - 1회당 약 354MB라 반복 측정
        캠페인에서 디스크를 가장 빨리 잡아먹는 산출물이고, 내용 자체는
        combined_*.pcd와 동일함(HISTORY.md §24)."""
        if not self.profiling_cfg.get('save_combined_csv', False):
            print("[*] save_combined_csv=false - CSV 내보내기를 건너뜀.")
            return None

        suffix = "_aborted" if self.is_aborted else ""
        if pcd_path is None:
            pcd_filename = self.PCD_FILENAME_TEMPLATE.format(ts=self.collection_start_ts, suffix=suffix)
            pcd_path = os.path.join(self.pointcloud_dir, pcd_filename)
        csv_filename = self.CSV_FILENAME_TEMPLATE.format(ts=self.collection_start_ts, suffix=suffix)
        csv_path = os.path.join(self.pointcloud_dir, csv_filename)
        return export_pcd_to_csv(pcd_path, csv_path)

    # ------------------------------------------------------------------
    # 자원 정리
    # ------------------------------------------------------------------

    def _shutdown(self):
        if hasattr(self, 'pc_subscription'):
            self.destroy_subscription(self.pc_subscription)
        if hasattr(self, 'tf_listener'):
            # spin_thread=False(기본값)이므로 별도 백그라운드 스레드가 없음.
            # tf 구독 정리만 하면 되고, join으로 기다려야 할 스레드가 없어
            # 이전에 있었던 ExternalShutdownException 레이스 컨디션도 없음.
            self.tf_listener.unregister()
        self.spin_executor.remove_node(self)
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    # ------------------------------------------------------------------
    # 외부 실행 엔트리포인트
    # ------------------------------------------------------------------

    def run(self):
        pcd_path = self._run_collection_stage()
        filtered_pcd_path = self._run_floor_extraction_stage(pcd_path)
        img_out_path = self._run_visualization_stage(filtered_pcd_path)
        csv_path = self.export_csv(pcd_path=pcd_path)
        # mission_executor.py도 eval_run_label이 있으면 stall_report/robot_path csv를
        # 같은 eval_runs/<라벨> 아래에 저장하므로, 여기도 workspace_root가 아니라
        # self.output_root를 넘겨야 서로 어긋나지 않음.
        analyze_and_visualize_stalls(self.output_root, self.mission_exec_cfg)

        print("\n=======================================================")
        if self.is_aborted:
            print("[!] Surface Profiling Pipeline Completed (ABORTED - partial data).")
        else:
            print("[+] Surface Profiling Pipeline Completed Successfully.")
        print(f"    -> Raw PCD: {pcd_path}")
        print(f"    -> Filtered PCD: {filtered_pcd_path}")
        print(f"    -> Heatmap Image: {img_out_path}")
        print(f"    -> CSV Export: {csv_path if csv_path else 'skipped (save_combined_csv=false)'}")
        print("=======================================================\n")

        self._shutdown()

def main(args=None):
    if args is None:
        args = sys.argv

    if not rclpy.ok():
        rclpy.init(args=args)

    profiler = SurfaceProfiler()
    profiler.run()


if __name__ == "__main__":
    main(args=sys.argv)
