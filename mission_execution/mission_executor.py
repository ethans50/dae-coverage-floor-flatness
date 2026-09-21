# dae_coverage_floor_flatness/mission_execution/mission_executor.py

import os
import sys
import copy
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from nav2_simple_commander.robot_navigator import BasicNavigator

# 경량 유틸리티 모듈
try:
    from utils.ros_utils import create_pose_stamped, teleport_gazebo_entity
    from utils.nav2_utils import apply_nav2_monkey_patches
    from utils.boundary_repass import BoundaryRepassController
    from utils.mission_logger import MissionLogger
    from utils.controller_switch import ControllerSwitcher, transit_bt_path
    from nav2_drive_mixin import Nav2DriveMixin
    from localization_mixin import LocalizationMixin
    from run_context_mixin import RunContextMixin
except ImportError:
    from mission_execution.utils.ros_utils import create_pose_stamped, teleport_gazebo_entity
    from mission_execution.utils.nav2_utils import apply_nav2_monkey_patches
    from mission_execution.utils.boundary_repass import BoundaryRepassController
    from mission_execution.utils.mission_logger import MissionLogger
    from mission_execution.utils.controller_switch import ControllerSwitcher, transit_bt_path
    from mission_execution.nav2_drive_mixin import Nav2DriveMixin
    from mission_execution.localization_mixin import LocalizationMixin
    from mission_execution.run_context_mixin import RunContextMixin


class MissionExecutor(Nav2DriveMixin, LocalizationMixin, RunContextMixin, Node):
    """
    Nav2 기반 미션 실행을 담당하는 ROS 2 노드.

    is_sim 여부는 ROS 2 파라미터(launch argument)로 주입받음. 예:
    ros2 launch dae_coverage_floor_flatness mission_execution.launch.py is_sim:=true

    주행 방식: final_path.json 전체를 방향(heading)이 바뀌는 지점 기준으로만
    재분할함(_split_into_straight_subsegments). coverage/transit 구분 없이
    모든 직선 sub-segment에서 동일하게 처리함(_execute_capture_subsegment):
    제자리 회전(Spin, 절대각 차이 기반) -> 캡처가 아직 꺼져 있으면 즉시 캡처
    시작 신호 -> 구간 끝점까지 여러 점을 한 번에 통과하는 직선
    주행(NavigateThroughPoses/goThroughPoses)하며 계속 캡처 -> 도착 시 정지.
    coverage뿐 아니라 transit 구간도 연속으로 측정해야 라이다 blind zone이
    충분히 메꿔지므로 두 종류를 다르게 처리하지 않음.

    캡처 종료는 record_pcd=True 구간(coverage)이 끝나는 시점, 즉 매
    "coverage exit" 경계마다 boundary_repass.BoundaryRepassController.
    run_exit_repass()가 담당함: 도착 -> 유턴 -> 왔던 방향으로 일정 거리
    되짚어 재통과(캡처 유지) -> 캡처 종료. 그 지점을 실제로 양방향(접근+
    멀어짐)으로 지나쳐야 얕은 각도에서 라이다 blind zone이 메워지는데, 정상
    도착만으로는 접근 방향 하나만 확보되므로 되짚기로 반대 방향 시야를
    인위적으로 만듦. 되짚기가 끝나면 로봇은 원래 coverage 종료 지점보다
    조금 안쪽에 있게 되지만, 그 다음 sub-segment(transit)의 정상 주행이
    로봇의 현재 위치에서부터 알아서 경로를 짜므로 별도 복귀 동작은 필요
    없음. execute_mission() 참고.
    """

    def __init__(self):
        super().__init__('mission_executor_node')

        print("\n=======================================================")
        print("[*] Nav2 Mission Executor (Jetson / Sim Controller)")
        print("=======================================================\n")

        # self.spin_executor
        # self(MissionExecutor) 전용 SingleThreadedExecutor.
        # navigator(BasicNavigator)는 nav2_simple_commander 라이브러리 내부에서
        # 자체적으로 rclpy.spin_once(navigator, ...)를 호출하므로, 이중 spin 충돌을
        # 피하기 위해 별도 executor에 등록하지 않고 독립적으로 관리함.
        self.spin_executor = SingleThreadedExecutor()
        self.spin_executor.add_node(self)

        # 상태 변수 초기화
        self.config = None
        self.global_cfg = None
        self.env_cfg = None
        self.mission_exec_cfg = None
        self.workspace_root = None
        self.map_bounds = None
        self.final_path = None
        self.navigator = None

        # FollowWaypoints는 coverage 지점마다 여러 세그먼트(goal)로 나뉘어 전송되므로,
        # 미션 전체의 성공 여부는 navigator.getResult()(마지막 세그먼트만 반영)가 아닌
        # 이 플래그로 별도 추적함.
        self.mission_succeeded = False

        # 캡처 상태 추적. coverage(record_pcd=True) sub-segment가 끝나는
        # 시점마다 execute_mission()이 boundary_repass.run_exit_repass()를 불러
        # 되짚기 후 여기서 명시적으로 끔 - _execute_capture_subsegment 자체는
        # 캡처를 스스로 끄지 않음.
        self._capture_active = False

        # 미션 전체의 진짜 첫/마지막 지점은 "지나쳐야 채워진다" 원칙의 전제(양방향
        # 통과)를 구조적으로 만족할 수 없음 - boundary_repass.py 모듈 docstring 참고.
        self._boundary_repass = BoundaryRepassController(self)

        # 주행 모니터링 관련 상태
        self.current_amcl_x = None
        self.current_amcl_y = None
        self.last_valid_amcl_pose = None
        self.max_allowed_jump = 0.60

        # 연속 점프 감지용 상태. 단발성 AMCL 보정(긴 직선 주행 후 누적된
        # dead-reckoning 오차가 한 번에 정상화되는 경우 등)까지 비상 상황으로
        # 취급해 미션을 중단시키면 너무 과민하므로, 짧은 시간 안에 여러 번
        # 반복될 때만 진짜 비상(로컬라이제이션 붕괴/텔레포트)으로 간주함.
        self.amcl_jump_timestamps = []
        self.amcl_jump_window_sec = 5.0     # 이 시간 안에
        self.amcl_jump_count_threshold = 3  # 이만큼 반복되면 비상으로 격상함
        self.path_history = []
        self._last_record_time = 0.0
        self.sub_amcl_check = None

        # 주행 중 5초 이상 진행이 멈추는 구간을 디버깅용으로 별도 기록함
        # (얼마나/왜 - number_of_recoveries 변화로 nav2 recovery 발동 여부까지
        # 함께 남김). execute_mission()의 모든 blocking 대기 루프에서 공용으로
        # 쌓이며, _save_mission_results()에서 이것만 따로 파일에 출력함.
        self._stall_events = []
        self._mission_start_wall_time = None

        # 주기 디버그 로그(mission_execution/utils/mission_logger.py, 기본 3초 간격) 연동용
        # 상태. execute_mission()의 각 단계 전환마다 _current_context를 갱신하고,
        # 4개의 blocking 대기 루프(_spin_action/_backup_for_clearance/
        # _navigate_to_pose_blocking/_navigate_through_poses_blocking)가
        # _nav_status를 매 폴링마다 갱신함 - MissionLogger는 별도 스레드에서
        # 이 두 값을 읽기만 함(도입 배경은 mission_logger.py 모듈 docstring 참고).
        self._current_context = None
        self._nav_status = {}
        self._mission_logger = None

        # _direct_cmd_vel_rotate(nav2 behavior_server를 우회하는 제자리 회전
        # 최종 폴백)용 - 평소엔 발행하지 않고, 그 함수 안에서만 씀.
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # AMCL 초기화 검증 관련 상태
        self.verified_amcl_x = None
        self.verified_amcl_y = None
        self.verified_amcl_yaw = None
        self.initial_pose = None
        self.sub_verify = None

        # surface_profiling(노트북) 측 PCD 수집 종료를 알리는 서비스 클라이언트.
        # 정상 종료와 비정상(실패/타임아웃) 종료를 서로 다른 서비스로 분리해서 호출함.
        self.stop_collection_success_client = self.create_client(
            Trigger, '/surface_profiling/stop_collection_success'
        )
        self.stop_collection_abort_client = self.create_client(
            Trigger, '/surface_profiling/stop_collection_abort'
        )

        # surface_profiling(노트북) 측 지점별(Waypoint) 캡처 시작/종료 서비스 클라이언트.
        # coverage 웨이포인트에 정지할 때마다 한 쌍(start -> stop)씩 호출함.
        self.start_capture_client = self.create_client(
            Trigger, '/surface_profiling/start_waypoint_capture'
        )
        self.stop_capture_client = self.create_client(
            Trigger, '/surface_profiling/stop_waypoint_capture'
        )

        # 1. ROS 2 파라미터로 시뮬레이션 모드 여부 확보 (launch argument로 주입됨)
        self._resolve_run_mode()

        # 2. params.yaml 설정 파일 파싱
        self._load_config()

        # coverage/transit 구간별 컨트롤러 파라미터 전환(utils/controller_switch.py).
        # mission_exec_cfg를 값으로 받으므로 _load_config() 뒤에 만들어야 함.
        self.controller_switch = ControllerSwitcher(self, self.spin_executor, self.mission_exec_cfg)

        # 3. 글로벌 맵 바운즈 확보
        self._load_map_bounds()

        # 4. 이전 파이프라인에서 생성된, 보간/샘플링이 끝난 JSON 원본 경로 파일 로드
        self._load_final_path()

    # ------------------------------------------------------------------
    # ROS 환경 셋업
    # ------------------------------------------------------------------

    def _setup_ros_environment(self):
        """
        BasicNavigator 인스턴스 생성, Gazebo 환경일 경우 초기 텔레포트 인터페이스 수행.

        rclpy.init()은 이 메서드에서 호출하지 않음. 이 노드(self) 자신을 포함한
        전체 ROS 2 컨텍스트는 main()에서 이미 단일하게 초기화되어 있다고 가정함.
        """
        print("[*] Connecting to Nav2 Server...")

        self.navigator = BasicNavigator()

        # BasicNavigator 레벨 몽키 패치.
        # navigator 인자는 호출 시점 일관성을 위해 받지만, 패치는 클래스 자체에 적용되므로
        # 인스턴스 유무와 무관하게 한 번만 적용되면 모든 BasicNavigator 인스턴스에 영향을 줌.
        apply_nav2_monkey_patches(self.navigator)

        # 미션이 실제로 시작해야 하는 물리적 지점 계산(_compute_mission_start_pose
        # 참고 - boundary_repass가 켜져 있으면 진짜 coverage 시작점이 아니라
        # 그보다 진행방향으로 앞선 러닝스타트 지점). sim/real 양쪽에서 이 하나의
        # 값을 그대로 재사용함(AMCL 초기 위치 힌트(_initialize_localization)와
        # 반드시 동일해야 함 - 어긋나면 AMCL이 틀린 방향을 정답이라 믿고 위치만
        # 수렴해버릴 수 있음).
        self._mission_start_pose = self._compute_mission_start_pose()

        if self.is_sim:
            temp_pose = copy.deepcopy(self._mission_start_pose)
            temp_pose.pose.position.z = 0.05

            teleport_gazebo_entity(temp_pose)
            print("[+] Gazebo Teleportation Successful (position + orientation matched to mission start pose).")
        else:
            print("[*] Real-world Mode: Skipping Gazebo Robot location Initializing.")

        if self.is_sim:
            print("[*] Waiting for Gazebo /clock synchronization...")
            while self.navigator.get_clock().now().nanoseconds == 0:
                rclpy.spin_once(self.navigator, timeout_sec=0.01)
            print("[+] Gazebo clock synchronization done.")

        print("[*] Waiting for Nav2 nodes to become fully Active...")
        self.navigator.waitUntilNav2Active()
        print("[+] Nav2 is now fully Active. Securing subscription margin...")
        time.sleep(4.0)

        # coverage run 시작 시 '제자리 회전'을 위해 현재 로봇의 실시간 헤딩(map->base_link)이
        # 필요함. /amcl_pose 토픽(저주파, AMCL 보정 시점에만 갱신)이 아니라 TF buffer를
        # 직접 조회하는 이유는, TF는 odom 기반으로 계속 보간되어 훨씬 더 실시간에 가까운
        # 값을 주기 때문임(회전 판단 시점의 정확도가 중요).
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    # ------------------------------------------------------------------
    # 경로 샘플링 (클램핑 전용 — 보간/샘플링은 mission_planner.py에서 완료됨)
    # ------------------------------------------------------------------

    def _prepare_goal_poses(self):
        target_margin = self.env_cfg.get('robot_width', 0.28) * 5

        goal_poses = []
        out_of_bounds_count = 0

        safe_bounds = {
            'min_x': self.map_bounds['min_x'] + target_margin,
            'max_x': self.map_bounds['max_x'] - target_margin,
            'min_y': self.map_bounds['min_y'] + target_margin,
            'max_y': self.map_bounds['max_y'] - target_margin
        }

        for wp_dict in self.final_path:
            pose_stamped = create_pose_stamped(self.navigator, wp_dict, safe_bounds)
            goal_poses.append(pose_stamped)

            if (abs(pose_stamped.pose.position.x - wp_dict['pose']['position']['x']) > 0.01 or
                    abs(pose_stamped.pose.position.y - wp_dict['pose']['position']['y']) > 0.01):
                out_of_bounds_count += 1

        if out_of_bounds_count > 0:
            print(f"[!] Warning: {out_of_bounds_count} waypoints were nudged into the safe map zone.")

        return goal_poses

    # ------------------------------------------------------------------
    # 미션 실행 메인 루프
    # ------------------------------------------------------------------

    def execute_mission(self):
        """
        '/amcl_pose' 모니터링 Subscription은 self(MissionExecutor) 노드에 생성하며,
        self.spin_executor(SingleThreadedExecutor)로 spin함. navigator.isTaskComplete()/
        getFeedback() 등 BasicNavigator 자체의 내부 동작은 nav2_simple_commander
        라이브러리 구현상 navigator 자신을 별도로 spin해야 하므로, 모니터링 루프
        안에서는 self.executor와 navigator를 각각 독립적으로 spin함.

        방향전환 기반 통합 상태기계:
        coverage(F2C 스와스)와 transit(A* 커넥터) 모두 바닥을 연속으로 측정해야
        blind zone(라이다 최소 측정거리 사각지대)이 충분히 메꿔짐.
        전체 final_path를 "방향(heading)이 바뀌는
        지점"만 기준으로 재분할해서, 모든 직선 구간에서 동일한 패턴을
        반복함:

            제자리 회전(Spin, 직전 구간과의 절대각 차이) -> 캡처 시작(이미
            켜져 있으면 생략) -> 구간 끝점까지 직선 주행하며 계속 캡처 -> 도착.

        구간 경계는 각 웨이포인트에 이미 기록된 orientation(translator.py가
        진행방향 기준으로 계산해둔 값)을 연속 비교해서 찾음 —
        direction_change_threshold_deg를 넘는 지점마다 새 구간 시작. coverage
        세그먼트 하나(F2C 스와스)는 태생적으로 직선이라 항상 구간 하나 그대로
        유지되고, A* 경로로 꺾임이 있는 transit은 꺾이는 지점마다 자동으로
        잘게 쪼개져서 각각 정지-측정됨. 좁은 방/복도가 단일 점으로 축약되는
        경우도 이 분할 로직에서 자연스럽게 길이 1짜리 구간으로 처리되어
        별도의 특수 케이스 코드가 필요 없음.

        캡처 종료는 sub-segment가 끝나는 시점에 곧바로 일어나지 않음.
        record_pcd=True(coverage) sub-segment가 끝났고, 그 다음 sub-segment가
        없거나 record_pcd=False(transit)로 바뀌는 지점 — 즉 "coverage exit"
        경계마다, self._boundary_repass.run_exit_repass()를 불러 도착 ->
        유턴 -> 왔던 방향으로 되짚어 재통과(캡처 유지) -> 캡처 종료를
        수행함. 미션의 진짜 마지막 지점도 "다음 sub-segment가 없는" 경우로
        자연히 이 조건에 포함되므로 별도 특수 처리가 필요 없음. 되짚기가
        끝나면 로봇이 원래 exit 지점보다 약간 안쪽에 있게 되지만, 이어지는
        transit sub-segment의 주행이 로봇의 실제 현재 위치부터 경로를 새로
        짜므로 복귀 동작 없이 정상 진행됨 - 단, mission_planner.py가 매
        transit A* 경로를 항상 직전 세그먼트의 마지막 점에서부터 탐색해서
        만들기 때문에, 모든 세그먼트 경계에는 "직전 세그먼트의 마지막 점과
        좌표가 같은" 구조적 중복점이 원래부터 존재함(repass와 무관하게
        final_path.json 자체의 특성). 이 중복점은 새 목적지 정보가 없어
        항상 안전하게 건너뛸 수 있으므로, 직전 세그먼트 마지막 점과 좌표가
        같은 sub-segment 첫 점은 매번 건너뜀(루프 앞머리 참고) - 단, 이
        스킵은 record_pcd=False(transit) sub-segment로만 한정함(coverage
        sub-segment에 적용하면 코너 없는 2점짜리 스와스가 1점으로 줄어
        캡처가 통째로 빠지는 회귀가 생김).

        각 구간은 별도 goal로 순차 전송되므로, 미션 전체의 성공/실패는
        self.mission_succeeded 플래그로 별도 추적함(navigator.getResult()는
        마지막으로 전송된 goal 하나의 결과만 반영하기 때문).
        """
        env_text = "in Gazebo" if self.is_sim else "to Real-world Robot"
        print(f"[*] Executing Mission {env_text} (Direction-Change-Based Continuous Capture)...")

        if self._shared_run_ts is not None:
            run_ts_str, self._mission_start_wall_time = self._shared_run_ts
            print(f"[*] run_ts='{run_ts_str}' (epoch={int(self._mission_start_wall_time)}) - "
                  "이 값을 미션 시작 시각으로 써서 drive_debug/stall_report/robot_path 파일명을 통일함.")
        else:
            run_ts_str = time.strftime('%Y-%m-%d_%H-%M-%S')
            self._mission_start_wall_time = time.time()

        drive_debug_log_dir = self._eval_output_dir(
            self.mission_exec_cfg.get('drive_debug_log_dir', 'analytics/logs'))
        drive_debug_log_path = os.path.join(
            drive_debug_log_dir, f"drive_debug_{run_ts_str}.csv")
        self._mission_logger = MissionLogger(
            self, drive_debug_log_path,
            interval_sec=self.mission_exec_cfg.get('drive_debug_interval_sec', 3.0),
            stall_threshold_sec=self.mission_exec_cfg.get('drive_debug_stall_threshold_sec', 9.0),
        )
        self._mission_logger.start()

        goal_poses = self._prepare_goal_poses()
        sub_segments = self._split_into_straight_subsegments(goal_poses)
        print(f"[*] Total waypoints: {len(goal_poses)}, split into {len(sub_segments)} "
              f"straight sub-segments (coverage + transit measured continuously).")

        self.last_valid_amcl_pose = (self.initial_pose.pose.position.x, self.initial_pose.pose.position.y)
        self.amcl_jump_timestamps = []
        self.sub_amcl_check = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_monitor_callback, 10
        )

        self.mission_succeeded = True

        first_s, first_e = sub_segments[0]
        if self.final_path[first_s]['header'].get('record_pcd', True):
            prepass_node_id = self.final_path[first_s]['header'].get('node_id')
            self._current_context = (f"node {prepass_node_id} prepass" if prepass_node_id is not None
                                      else "prepass")
            self._boundary_repass.run_start_prepass(goal_poses[first_s:first_e])

        for seg_idx, (s, e) in enumerate(sub_segments):
            seg_poses = goal_poses[s:e]
            seg_header = self.final_path[s]['header']
            seg_type_prefix = seg_header['task_type'].split('_')[0]
            record_pcd = seg_header.get('record_pcd', True)

            if seg_type_prefix == 'coverage':
                node_id = seg_header.get('node_id')
                self._current_context = f"node {node_id} coverage" if node_id is not None \
                    else "coverage (node id unknown - regenerate path)"
            else:
                from_id, to_id = seg_header.get('from_node_id'), seg_header.get('to_node_id')
                self._current_context = f"transit node {from_id} -> node {to_id}" \
                    if (from_id is not None or to_id is not None) \
                    else "transit (node id unknown - regenerate path)"

            if seg_idx > 0 and not record_pcd and len(seg_poses) > 1:
                # mission_planner.py가 세그먼트 경계마다 남기는 구조적
                # 중복점(직전 세그먼트의 마지막 점과 좌표가 같고 orientation만
                # 다름)을 제거함. 로봇이 이미 그 자리에 있어 방문 자체가
                # no-op이고, repass로 뒤로 물러난 경우엔 후진 불가(min_vel_x=0.0)
                # 때문에 goThroughPoses가 즉시 FAILED가 됨.
                # transit(record_pcd=False)에만 적용함 - coverage에도 적용하면
                # 2점짜리 스와스가 1점으로 줄어 캡처가 통째로 빠짐.
                prev_last_pose = goal_poses[sub_segments[seg_idx - 1][1] - 1]
                if (abs(seg_poses[0].pose.position.x - prev_last_pose.pose.position.x) < 1e-3
                        and abs(seg_poses[0].pose.position.y - prev_last_pose.pose.position.y) < 1e-3):
                    seg_poses = seg_poses[1:]

            print(f"\n[>>>] Sub-segment {seg_idx + 1}/{len(sub_segments)}: "
                f"origin_type='{seg_type_prefix}', record_pcd={record_pcd}, "
                f"points={len(seg_poses)} (global idx {s}~{e - 1})")

            is_genuine_single = self.final_path[s]['header']['task_type'].endswith('_single')
            seg_label = f"seg{seg_idx + 1}/{len(sub_segments)}:{seg_type_prefix}"
            ok = self._execute_capture_subsegment(
                seg_poses, record_pcd=record_pcd, is_genuine_single=is_genuine_single,
                seg_type=seg_type_prefix, label=seg_label,
            )

            if not ok:
                print(f"[-] Sub-segment {seg_idx + 1} failed. Aborting mission.")
                self.mission_succeeded = False
                break

            # coverage exit 경계: 방금 끝난 sub-segment가 record_pcd=True였고,
            # 다음 sub-segment가 없거나(=미션 진짜 마지막 지점) record_pcd=False로
            # 바뀐다면(=다음이 transit) 여기가 exit임. 매번 여기서 되짚기 후
            # 캡처를 끔 - 뒤이은 transit sub-segment는 로봇의 실제 위치부터
            # 알아서 경로를 짜므로 별도 복귀 동작이 필요 없음(다음 sub-segment의
            # 구조적 중복 첫 점은 위 루프 앞머리에서 이미 걸러짐).
            is_last_segment = (seg_idx == len(sub_segments) - 1)
            next_record_pcd = None if is_last_segment else \
                self.final_path[sub_segments[seg_idx + 1][0]]['header'].get('record_pcd', True)
            is_coverage_exit = record_pcd and (is_last_segment or not next_record_pcd)

            if is_coverage_exit and self._capture_active:
                exit_node_id = seg_header.get('node_id')
                self._current_context = (f"node {exit_node_id} end repass" if exit_node_id is not None
                                          else "end repass (node id unknown - regenerate path)")
                self._boundary_repass.run_exit_repass(seg_poses)

        if self._capture_active:
            # 루프가 break로 중단됐다면(세그먼트 실패), 로봇의 실제 위치가 planning과
            # 다를 수 있으므로 추가 주행 없이 즉시 캡처만 종료함. 정상 종료라면
            # 위 루프에서 마지막 coverage exit이 이미 repass로 캡처를 껐을 것이므로
            # 이 분기에 도달하지 않음.
            self._call_capture_service(self.stop_capture_client, "stop_waypoint_capture")
            self._capture_active = False

        if self.sub_amcl_check is not None:
            self.destroy_subscription(self.sub_amcl_check)
            self.sub_amcl_check = None

        self._current_context = "mission ended"
        if self._mission_logger is not None:
            self._mission_logger.stop()
            print(f"[*] Mission debug log saved: {self._mission_logger.log_path}")

        if not self.mission_succeeded and not self.navigator.isTaskComplete():
            self.navigator.cancelTask()

    def _split_into_straight_subsegments(self, goal_poses):
        """
        전체 경로(goal_poses)를 방향(heading)이 바뀌는 지점, 그리고 record_pcd가
        바뀌는 지점마다 분할함. 각 점에 이미 기록된 orientation(진행방향 기준)을
        연속 비교해서 yaw변화량이 direction_change_threshold_deg(기본 15도)를
        넘으면 그 지점을 새 구간의 시작으로 삼음. 반환값은
        [(start_idx, end_idx_exclusive), ...].

        note: min_rotation_deg(_rotate_in_place_to에서 사용, 기본 3도)보다 이
        threshold를 더 크게 잡는 이유는, 미세한 리샘플링/부동소수점 오차로
        생기는 각도 잡음까지 매번 별도 구간(=매번 정지)으로 나눠버리면 불필요한
        정지가 과도하게 늘어나기 때문임.

        record_pcd 경계에서도 반드시 분할해야 하는 이유: narrow 버킷(스와스 1개)
        노드는 order_swaths_by_entry가 진입 방향에 맞춰 스와스 시작점을 고르기
        때문에, 진입 transit의 마지막 heading과 coverage 스와스의 heading이
        구조적으로 일치하는 경우가 흔함. heading만으로 분할하면 이 경계가 안
        잘려서 transit(record_pcd=False)과 coverage(record_pcd=True)가 한
        sub-segment로 합쳐지고, execute_mission()이 병합된 그룹의 record_pcd를
        맨 앞 점 하나로만 판단하므로 coverage 구간 전체의 캡처가 조용히
        통째로 사라짐.
        """
        n = len(goal_poses)
        if n <= 1:
            return [(0, n)]

        threshold = math.radians(self.mission_exec_cfg.get('direction_change_threshold_deg', 15.0))

        segments = []
        seg_start = 0
        prev_yaw = self._quaternion_to_yaw(goal_poses[0].pose.orientation)
        prev_record_pcd = self.final_path[0]['header'].get('record_pcd', True)

        for i in range(1, n):
            curr_yaw = self._quaternion_to_yaw(goal_poses[i].pose.orientation)
            dyaw = curr_yaw - prev_yaw
            dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))  # -pi ~ +pi 정규화

            curr_record_pcd = self.final_path[i]['header'].get('record_pcd', True)

            if abs(dyaw) > threshold or curr_record_pcd != prev_record_pcd:
                segments.append((seg_start, i))
                seg_start = i

            prev_yaw = curr_yaw
            prev_record_pcd = curr_record_pcd

        segments.append((seg_start, n))

        # 코너와 코너가 바로 이웃해서 생기는 고립된 1점짜리 sub-segment(진짜 F2C 단일점 방이 '_single' 태그가 아닌 경우)는
        # 별도로 세워서 처리하지 않고 다음 sub-segment 맨 앞에 편입시킴 - 그러면 그 코너점이
        # goThroughPoses의 경유점 중 하나로 포함되어, 혼자 남겨지는 상황을 방지함.
        # 단, record_pcd가 다음 sub-segment와 다르면 병합하지 않음 - 병합하면 그 1점의 캡처
        # 여부가 이웃 세그먼트의 flag로 조용히 덮어써지기 때문.
        merged_segments = []
        i = 0
        while i < len(segments):
            s, e = segments[i]
            is_trivial_single = (
                (e - s == 1)
                and not self.final_path[s]['header']['task_type'].endswith('_single')
                and i + 1 < len(segments)
                and self.final_path[s]['header'].get('record_pcd', True)
                    == self.final_path[segments[i + 1][0]]['header'].get('record_pcd', True)
            )
            if is_trivial_single:
                _, next_e = segments[i + 1]
                merged_segments.append((s, next_e))
                i += 2
            else:
                merged_segments.append((s, e))
                i += 1

        return merged_segments

    # ------------------------------------------------------------------
    # 직선 sub-segment 실행 (회전 -> [필요시] 캡처 시작 -> 직선 주행+캡처).
    # coverage/transit 구분 없이 모든 직선 구간에 동일하게 적용됨. 캡처
    # 종료는 이 함수의 책임이 아님 - coverage exit 경계에서
    # execute_mission()이 boundary_repass.run_exit_repass()를 통해 명시적으로
    # 끔(그 안에서 되짚기 왕복까지 마친 뒤 종료).
    # ------------------------------------------------------------------

    def _execute_capture_subsegment(self, seg_poses, record_pcd=True, is_genuine_single=True, seg_type='transit', label='sub-segment'):
        mode = 'coverage' if seg_type == 'coverage' else 'transit'
        self.controller_switch.set_angular_dist_threshold(mode)
        self.controller_switch.set_speed_limit(mode)
        # transit은 RPP(FollowPathTransit)를 쓰는 전용 BT로, coverage는 nav2 기본 BT(DWB)로 주행함.
        # 빈 문자열이면 nav2가 기본 BT를 씀.
        bt_through = transit_bt_path('through_poses') if mode == 'transit' else ''
        bt_to_pose = transit_bt_path('to_pose') if mode == 'transit' else ''
        capture_sec_single = self.mission_exec_cfg.get('active_capture_seconds', 2.0)
        is_single_point = (len(seg_poses) == 1)

        # 1. 제자리 회전
        if is_single_point and not is_genuine_single:
            # 코너점이 직전 sub-segment의 마지막 점과 좌표가 같아 구조적 중복점
            # 스킵(execute_mission() 참고)에 걸리면, 그 코너점이 담당하던 큰
            # 방향 전환이 사라진 채 이 점 하나만 남을 수 있음 - 회전 없이 바로
            # goToPose만 쏘면 Nav2가 회전+이동을 동시에 처리해야 해서 벽/코너
            # 근처에서 반복 stall을 일으킴(실측 확인).
            # _rotate_in_place_to는 목표가 현재 위치와 같거나 회전량이
            # min_rotation_deg 미만이면 스스로 스킵하므로 안전하게 항상 먼저 호출함.
            if not self._rotate_in_place_to(seg_poses[0], label=label):
                print("[!] Warning: In-place rotation to isolated corner point failed or skipped. Proceeding anyway.")
            if not self._navigate_to_pose_blocking(seg_poses[0], label=label, behavior_tree=bt_to_pose):
                print("[!] Warning: failed to reach isolated corner point. Proceeding anyway.")
        elif not is_single_point:
            if not self._rotate_in_place_to(self._pick_rotation_aim_pose(seg_poses), label=label):
                print("[!] Warning: In-place rotation failed or skipped. Proceeding anyway.")

        # 2. 캡처 시작 신호.
        #    이미 캡처가 켜져 있으면(같은 노드 안에서 coverage sub-segment가
        #    연달아 이어지는 경우, 예: 스와스 중간의 90도 코너) 재시작하지 않고
        #    그대로 이어감 - 캡처 창이 끊기지 않아야 한 창에 계속 쌓임.
        started = self._capture_active

        if record_pcd and not self._capture_active:
            started = self._call_capture_service(self.start_capture_client, "start_waypoint_capture")  # 측정 누락 방지를 위해 캡처 신호를 주행보다 먼저 보냄
            if started:
                # 정지 대기는 넣지 않음 - 가시 링만 진해질 뿐 사각지대는 그대로이므로
                # 캡처 시작 직후 바로 이동함.
                print("  [Capture] Capture started, moving immediately (no stationary settle).")
            else:
                print("[!] Skipping this sub-segment's capture window (start signal failed).")
            self._capture_active = started
        elif not record_pcd:
            print("  [Capture] record_pcd=False — skipping capture, driving through.")

        # 3. 주행
        ok = True
        if not is_single_point:
            print(f"  [Drive] Straight sub-segment: {len(seg_poses)} points "
                  f"({'continuous capture' if self._capture_active else 'no capture'} while moving).")
            ok = self._navigate_through_poses_blocking(seg_poses, label=label, behavior_tree=bt_through)
        elif started:
            print("  [Drive] Single-point sub-segment: staying in place for capture.")
            self._spin_sleep(capture_sec_single)
        elif record_pcd:
            print("  [Drive] Single-point sub-segment: capture start failed, skipping dwell.")
        else:
            print("  [Drive] Single-point sub-segment: record_pcd=False, skipping dwell entirely.")

        return ok

    # ------------------------------------------------------------------
    # 캡처 시퀀스 보조 유틸 (settle 대기, surface_profiler 서비스 호출)
    # ------------------------------------------------------------------

    def _spin_sleep(self, duration_sec):
        """AMCL/네트워크 통신이 끊기지 않도록 spin을 유지하면서 duration_sec만큼 대기함."""
        end_time = time.time() + duration_sec
        while time.time() < end_time:
            self.spin_executor.spin_once(timeout_sec=0.05)
            if self.navigator is not None:
                rclpy.spin_once(self.navigator, timeout_sec=0.01)
            time.sleep(0.02)

    def _call_capture_service(self, client, label):
        """
        surface_profiler.py의 start/stop_waypoint_capture Trigger 서비스를 호출함.
        best-effort: 서버가 없거나 응답이 없어도 미션 자체를 막지 않고 경고만 남김.
        """
        service_name = client.srv_name
        if not client.wait_for_service(timeout_sec=2.0):
            print(f"[!] Warning: Service '{service_name}' not available. "
                  f"Is surface_profiler.py running on the notebook? Skipping {label}.")
            return False

        request = Trigger.Request()
        future = client.call_async(request)

        spin_start = time.time()
        while not future.done() and (time.time() - spin_start) < 5.0:
            self.spin_executor.spin_once(timeout_sec=0.1)

        if future.done() and future.result() is not None:
            response = future.result()
            print(f"[*] {label}: success={response.success}, message='{response.message}'")
            return response.success
        else:
            print(f"[!] Warning: No response from '{service_name}' within timeout.")
            return False

    # ------------------------------------------------------------------
    # surface_profiling(노트북) 측에 PCD 수집 종료 신호 전달
    # ------------------------------------------------------------------

    def _notify_surface_profiling_stop(self, success: bool, message: str = ""):
        """
        노트북에서 구동 중인 SurfaceProfiler에게 수집 종료를 알림.
        success=True면 정상 종료 서비스, False면 비정상(즉시 강제 종료) 서비스를 호출함.
        서버가 아직 떠 있지 않거나 응답이 없어도 미션 자체의 종료를 막지는 않음
        (호출은 best-effort로 처리하고, 결과만 로그로 남김).
        """
        client = self.stop_collection_success_client if success else self.stop_collection_abort_client
        service_name = client.srv_name

        if not client.wait_for_service(timeout_sec=3.0):
            print(f"[!] Warning: Service '{service_name}' not available. "
                  f"Is surface_profiler.py running on the notebook? Skipping notification.")
            return

        request = Trigger.Request()
        future = client.call_async(request)

        spin_start = time.time()
        while not future.done() and (time.time() - spin_start) < 5.0:
            self.spin_executor.spin_once(timeout_sec=0.1)

        if future.done() and future.result() is not None:
            response = future.result()
            print(f"[*] Notified '{service_name}': success={response.success}, message='{response.message}'")
        else:
            print(f"[!] Warning: No response from '{service_name}' within timeout.")

    # ------------------------------------------------------------------
    # 외부 실행 엔트리포인트
    # ------------------------------------------------------------------

    def run(self):
        self._setup_ros_environment()
        self._initialize_localization()
        self.execute_mission()
        self._save_mission_results()

def main(args=None):
    if args is None:
        args = sys.argv

    if not rclpy.ok():
        rclpy.init(args=args)

    mission_executor = MissionExecutor()
    mission_executor.run()

if __name__ == "__main__":
    main(args=sys.argv)