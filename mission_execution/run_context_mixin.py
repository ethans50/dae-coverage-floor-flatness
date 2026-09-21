# dae_coverage_floor_flatness/mission_execution/run_context_mixin.py
"""
MissionExecutor의 "실행 맥락" 믹스인 - 입력을 읽어오고 결과를 내보내는 양 끝단.

앞단은 실행 모드(is_sim) 해석, `params.yaml` 파싱, 맵 바운즈/`final_path.json`
로드와 planning-실행 파라미터 대조(`_verify_plan_meta`)이고, 뒷단은 정체 리포트와
주행 경로/시각화 저장 및 ROS 자원 셧다운임. 그 사이의 실제 주행은
`mission_executor.py`(미션 흐름), `nav2_drive_mixin.py`(제어),
`localization_mixin.py`(위치 추정)가 나눠 맡음.

`eval_run_label`을 주면 그 라벨의 스냅샷 폴더에서 경로/맵을 읽고, `run_ts`는
여러 기기에서 실행하는 프로세스들이 산출물 파일명 타임스탬프를 공유하는 데 씀.

믹스인으로 둔 이유는 `nav2_drive_mixin.py`와 같음 - `self._eval_output_dir(...)`
같은 다른 곳의 호출부를 그대로 두기 위함임.
"""

import os
import sys
import csv
import json
import time
import traceback

import yaml
import rclpy
from rclpy.parameter import Parameter
from nav2_simple_commander.robot_navigator import TaskResult

try:
    from utils.map_utils import get_map_bounds
    from utils.visualizer import visualize_paths, visualize_planned_wall_proximity, visualize_stall_points
    from utils.stall_logger import write_stall_report
except ImportError:
    from mission_execution.utils.map_utils import get_map_bounds
    from mission_execution.utils.visualizer import visualize_paths, visualize_planned_wall_proximity, visualize_stall_points
    from mission_execution.utils.stall_logger import write_stall_report


class RunContextMixin:
    """설정/경로 로드와 결과 저장. MissionExecutor에 믹스인함."""

    def _resolve_run_mode(self):
        """
        'is_sim' ROS 2 파라미터를 선언하고 읽음. launch 파일이 주입하지 않으면
        기본값 False(Real-world)로 동작함.

        use_sim_time은 launch 파일이 is_sim과 함께 주입하는 것이 표준이지만,
        혹시 누락되더라도 안전하게 동작하도록 is_sim 값으로부터 use_sim_time을
        다시 추론해 자기 자신에게 강제 적용함.
        """
        self.declare_parameter('is_sim', False)
        self.is_sim = self.get_parameter('is_sim').get_parameter_value().bool_value

        # 알고리즘 비교 실험용 - launch argument로 라벨을 주면 이번 미션의
        # 출력물(robot_path/stall_report/drive_debug csv, 시각화 png)을
        # <workspace_root>/eval_runs/<라벨>/ 아래로 모아 저장함. 빈 문자열(기본값)이면
        # 평소처럼 workspace_root 바로 아래 flat 경로에 저장함 - _eval_output_dir 참고.
        self.declare_parameter('eval_run_label', '')
        self.eval_run_label = self.get_parameter('eval_run_label').get_parameter_value().string_value

        # 여러 기기에 나눠 실행될 때 산출물 파일명 타임스탬프를 맞추기 위한
        # 공유 값. 양쪽 launch에 같은 값을 넘기면 각자 시계 대신 이 문자열을
        # 씀. 빈 문자열이면 각자 찍음.
        self.declare_parameter('run_ts', '')
        run_ts_param = self.get_parameter('run_ts').get_parameter_value().string_value
        self._shared_run_ts = None
        if run_ts_param:
            try:
                self._shared_run_ts = (
                    run_ts_param,
                    time.mktime(time.strptime(run_ts_param, '%Y-%m-%d_%H-%M-%S')),
                )
            except ValueError:
                print(f"[!] CRITICAL ERROR: run_ts='{run_ts_param}' 형식이 잘못됨 - "
                      "'YYYY-MM-DD_HH-MM-SS' 형식이어야 함.")
                sys.exit(1)

        if self.is_sim:
            self.get_logger().info("Running Simulation(Gazebo) Mode.")
        else:
            self.get_logger().info("Running Real-world Mode.")

        # use_sim_time 강제 동기화 (launch 파일이 누락했을 경우의 안전장치)
        use_sim_time_param = self.get_parameter_or(
            'use_sim_time', Parameter('use_sim_time', Parameter.Type.BOOL, self.is_sim)
        )
        if use_sim_time_param.value != self.is_sim:
            self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, self.is_sim)])

    def _load_config(self):
        try:
            from ament_index_python.packages import get_package_share_directory
            package_share_dir = get_package_share_directory('dae_coverage_floor_flatness')
            config_path = os.path.join(package_share_dir, 'config', 'params.yaml')
        except Exception:
            # mission_execution 내에서 실행 시 프로젝트 루트의 config로 fallback 탐색
            base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
            config_path = os.path.join(base_dir, "config", "params.yaml")

        print(f"[*] Resolving parameters from: {config_path}")
        if not os.path.exists(config_path):
            print(f"[!] Critical Error: params.yaml file not found at {config_path}")
            sys.exit(1)

        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)

        self.global_cfg = self.config.get('global', {})
        self.workspace_root = os.path.expanduser(
            self.global_cfg.get('workspace_root', '~/dae_floor_maps')
        )
        self.env_cfg = self.config.get('environment_modeling', {})
        self.mission_exec_cfg = self.config.get('mission_execution', {})
        # 실행 시점에 직접 쓰이진 않지만, planning 시점에 쓰인 값(final_path_meta.json)과
        # 대조하기 위해서만 참조함 - _verify_plan_meta 참고.
        self.mission_planner_cfg = self.config.get('mission_planner', {})

    def _eval_output_dir(self, rel_path):
        """이번 미션의 출력물 저장 경로를 계산함. eval_run_label이 비어있으면
        평소와 동일하게 workspace_root 바로 아래(rel_path)를 씀 - 평소 실행은
        이 함수를 거쳐도 결과가 전혀 달라지지 않음. 라벨이 있으면
        workspace_root/eval_runs/<라벨>/rel_path로 리다이렉트함.
        map_from_dae.yaml/final_path.json처럼 다른 실행이 참조해야 하는 입력성
        경로는 이 함수를 쓰지 않음 - 순수 출력 전용 경로에만 사용할 것."""
        if self.eval_run_label:
            return os.path.join(self.workspace_root, 'eval_runs', self.eval_run_label, rel_path)
        return os.path.join(self.workspace_root, rel_path)

    def _load_map_bounds(self):
        grid_dir = os.path.join(self.workspace_root, self.env_cfg.get('output_grid_dir', 'maps/grid'))
        yaml_path = os.path.normpath(os.path.join(grid_dir, "map_from_dae.yaml"))
        self.map_yaml_path = yaml_path

        if not os.path.exists(yaml_path):
            print(f"[!] CRITICAL ERROR: Map bounds data '{yaml_path}' not found!")
            sys.exit(1)

        self.map_bounds = get_map_bounds(yaml_path)

    def _load_final_path(self):
        """final_path.json을 읽어옴.

        eval_run_label이 있으면 eval_runs/<라벨>/ 스냅샷을 확정적으로 읽음 -
        flat 경로(analytics/metrics/)의 원본은 생성할 때마다 덮어써져서
        조합을 번갈아 생성하면 다른 조합의 경로를 읽게 됨. 라벨이 없으면
        평소대로 flat만 읽음. map_from_dae.yaml은 조합과 무관하게 동일해
        계속 flat 전용임(_eval_output_dir 참고)."""
        if self.eval_run_label:
            metric_dir = os.path.join(
                self.workspace_root, 'eval_runs', self.eval_run_label,
                self.mission_exec_cfg.get('input_metric_dir', 'analytics/metrics')
            )
        else:
            metric_dir = os.path.join(self.workspace_root, self.mission_exec_cfg.get('input_metric_dir', 'analytics/metrics'))
        cache_file = os.path.normpath(os.path.join(metric_dir, "final_path.json"))

        if not os.path.exists(cache_file):
            print(f"[!] CRITICAL ERROR: Mission route '{cache_file}' not found!")
            if self.eval_run_label:
                print(f"[-] eval_run_label='{self.eval_run_label}'로 실행했지만 해당 스냅샷이 없습니다. "
                      f"먼저 'python3 run_generation_pipeline.py --snapshot-label {self.eval_run_label}'로 생성하세요.")
            else:
                print("[-] Please run 'run_generation_pipeline.py' on your workstation first.")
            sys.exit(1)

        print(f"[*] Found pre-generated path at {cache_file}. Loading...")
        with open(cache_file, 'r') as f:
            self.final_path = json.load(f)
        print(f"[*] Path loaded. Total raw waypoints: {len(self.final_path)}")

        self._verify_plan_meta(metric_dir)

    def _verify_plan_meta(self, metric_dir):
        """
        final_path.json이 planning 시점(run_generation_pipeline.py -> MissionPlanner.plan())에
        실제로 사용한 파라미터 값을, 지금 이 노드가 params.yaml에서 읽은 실행 시점 값과
        대조함. mission_planner.py가 plan() 마지막에 함께 저장하는 사이드카
        'final_path_meta.json'을 읽어 비교함.

        boundary_repass_distance_m/enable_boundary_repass는 final_path.json의
        좌표 자체(transit이 실제로 시작하는 지점)에 기하학적으로 반영되므로, 두
        시점의 값이 어긋나면 planning된 transit 시작점과 실제 repass 후 로봇 위치가
        조용히 달라짐 - robot_width/path_safety_margin도 경로 형상 자체에
        반영되는 같은 범주의 값임. 이 일치를 사람이 매번 기억할 필요 없도록
        여기서 자동으로 대조하고, 어긋나면 다른 CRITICAL ERROR들과 동일하게
        즉시 중단시킴.

        사이드카 파일이 없으면(예: 이 검증 로직 추가 이전에 생성된 오래된
        final_path.json) 대조 자체를 건너뛰고 경고만 남김 - 하위 호환을 위해
        미션을 막지는 않음.
        """
        meta_file = os.path.normpath(os.path.join(metric_dir, "final_path_meta.json"))
        if not os.path.exists(meta_file):
            print(f"[!] Warning: '{meta_file}' not found - cannot verify plan/runtime "
                  f"parameter consistency (older final_path.json?). Proceeding without check.")
            return

        with open(meta_file, 'r') as f:
            plan_meta = json.load(f)

        runtime_values = {
            'robot_width': self.env_cfg.get('robot_width', 0.28),
            'path_safety_margin': self.mission_planner_cfg.get('path_safety_margin', 0.20),
            'boundary_repass_distance_m': self.mission_exec_cfg.get('boundary_repass_distance_m', 1.5),
            'enable_boundary_repass': self.mission_exec_cfg.get('enable_boundary_repass', True),
        }

        mismatches = []
        for key, runtime_val in runtime_values.items():
            plan_val = plan_meta.get(key)
            if plan_val is None:
                continue
            if isinstance(plan_val, bool) or isinstance(runtime_val, bool):
                mismatch = bool(plan_val) != bool(runtime_val)
            else:
                mismatch = abs(float(plan_val) - float(runtime_val)) > 1e-6
            if mismatch:
                mismatches.append((key, plan_val, runtime_val))

        if mismatches:
            print(f"\n[!!! CRITICAL ERROR !!!] final_path.json was planned with different "
                  f"parameters than this executor's current params.yaml:")
            for key, plan_val, runtime_val in mismatches:
                print(f"    - {key}: planned={plan_val}, current params.yaml={runtime_val}")
            print("[-] The planned transit start points / path geometry no longer match what "
                  "this executor would produce. Either revert params.yaml to the planned values, "
                  "or re-run run_generation_pipeline.py to regenerate final_path.json before "
                  "executing this mission.")
            sys.exit(1)

        print("[+] Plan/runtime parameter consistency verified (boundary_repass, robot_width, "
              "path_safety_margin match final_path_meta.json).")

    # ------------------------------------------------------------------
    # 결과 저장
    # ------------------------------------------------------------------

    def _write_stall_report(self):
        """execute_mission() 동안 쌓인 self._stall_events(5초 이상 진행이 멈춘
        구간, StallWatcher 참고)만 따로 CSV 한 파일에 출력함 - 다른 로그와
        섞이지 않게 해서 grep/정렬만으로 어디서 얼마나/왜(recoveries 발동 여부)
        지연됐는지 바로 확인할 수 있게 하기 위함. 성공/실패/취소와 무관하게
        항상 호출됨."""
        if self._mission_start_wall_time is None:
            return

        log_dir = self._eval_output_dir(self.mission_exec_cfg.get('stall_log_dir', 'analytics/logs'))
        try:
            os.makedirs(log_dir, exist_ok=True)
            log_path = os.path.join(log_dir, f"stall_report_{int(self._mission_start_wall_time)}.csv")
            write_stall_report(
                self._stall_events, log_path, self._mission_start_wall_time,
                threshold_sec=self._stall_threshold_sec(),
            )
            print(f"[*] Stall report ({len(self._stall_events)} stall(s) >= "
                  f"{self._stall_threshold_sec():.1f}s) saved to: {log_path}")
        except Exception as e:
            print(f"[-] Failed to write stall report: {e}")
            traceback.print_exc()

    def _save_mission_results(self):
        # FollowWaypoints는 coverage 지점마다 별도의 goal로 나뉘어 순차 전송되므로,
        # navigator.getResult()는 "마지막으로 보낸 세그먼트" 하나의 결과만 반영함.
        # 미션 전체의 성공/실패는 execute_mission()에서 추적한 self.mission_succeeded를
        # 우선 참조하고, result는 로그 참고용으로만 사용함.
        result = self.navigator.getResult()
        overall_success = self.mission_succeeded

        # 주행 지연(stall) 리포트는 성공/실패/취소와 무관하게 항상 남김 -
        # "끝까지 완주는 했지만 중간중간 오래 멈췄던 구간"을 디버깅하는 것이
        # 목적이므로, 오히려 실패한 실행에서도 마지막 stall이 실패 원인일 수
        # 있어 더 중요함. 다른 로그와 섞이지 않도록 별도 파일 하나에만 씀.
        self._write_stall_report()

        if overall_success:
            print("[+] Mission Successfully Completed!")
            self._notify_surface_profiling_stop(success=True, message="Mission completed successfully.")

            # CSV 데이터 저장. 파일명의 타임스탬프는 저장 시점(time.time())이
            # 아니라 self._mission_start_wall_time(미션 시작 시각)을 씀 -
            # stall_report_<epoch>.csv/drive_debug_<날짜-시각>.csv와 같은 값을
            # 공유하게 되어(run_ts가 주어졌다면 그 값), 한 미션의 산출물
            # 전체가 같은 타임스탬프로 묶임.
            # stall_report_analyzer.py의 매칭은 파일명 숫자가 아니라 CSV 내용의
            # timestamp 컬럼을 대조하므로 이 값을 바꿔도 그 로직엔 영향 없음.
            run_ts_epoch = int(self._mission_start_wall_time)
            output_path_dir = self._eval_output_dir(self.mission_exec_cfg.get('output_path_dir', 'analytics/paths'))
            os.makedirs(output_path_dir, exist_ok=True)
            csv_filename = os.path.join(output_path_dir, f"robot_path_{run_ts_epoch}.csv")

            try:
                with open(csv_filename, mode='w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(["timestamp", "x", "y"])
                    writer.writerows(self.path_history)
                print(f"[+] Successfully saved robot path history to '{csv_filename}'.")

                # 결과 시각화(PNG) 저장
                vis_dir = self._eval_output_dir(self.mission_exec_cfg.get('visualization_dir', 'visualization/mission_execution'))
                os.makedirs(vis_dir, exist_ok=True)
                img_out_path = os.path.join(vis_dir, f"robot_path_{run_ts_epoch}_plot.png")

                visualize_paths(csv_filename, self.final_path, img_out_path)

                risk_img_path = os.path.join(vis_dir, f"robot_path_{run_ts_epoch}_wall_risk.png")
                visualize_planned_wall_proximity(
                    self.final_path, self.map_yaml_path, risk_img_path,
                    robot_radius_m=self.env_cfg.get('robot_width', 0.15),
                )

                stall_img_path = os.path.join(vis_dir, f"robot_path_{run_ts_epoch}_stall.png")
                visualize_stall_points(csv_filename, stall_img_path)

            except Exception as e:
                print(f"[-] Failed to save outputs due to error: {e}")
                traceback.print_exc()

        elif result == TaskResult.CANCELED:
            print(f"\n[!] Mission was canceled! (last segment result={result})")
            self._notify_surface_profiling_stop(success=False, message="Mission was canceled.")
        else:
            print(f"\n[-] Mission failed! (last segment result={result})")
            self._notify_surface_profiling_stop(success=False, message="Mission failed.")

        # ROS 2 자원 안전 셧다운 (데드락 방지: subscription들은 execute_mission/_initialize_localization
        # 단계에서 이미 정리되었으므로 여기서는 executor/노드/컨텍스트 종료만 수행함)
        self.spin_executor.remove_node(self)
        self.destroy_node()
        if self.navigator is not None:
            self.navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
