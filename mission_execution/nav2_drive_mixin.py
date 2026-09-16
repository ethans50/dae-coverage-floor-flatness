# dae_coverage_floor_flatness/mission_execution/nav2_drive_mixin.py
"""
MissionExecutor의 nav2 액션 래퍼와 정체 복구 escalation을 담은 믹스인.

Spin/BackUp/NavigateToPose/NavigateThroughPoses 호출과, 그것들이 막혔을 때의
3단계 escalation(nav2 액션 -> 후진 후 재시도 -> /cmd_vel 직접 발행)이 전부
여기 있음. 단계별 동작과 안전성 근거는 DETAILS.md §3 참고.

믹스인으로 분리한 이유: mission_executor.py 비대화를 막으면서도
`self._rotate_in_place_to(...)` 같은 기존 호출부(boundary_repass.py 포함)를
그대로 두기 위함임 - 별도 협력 객체로 빼면 모든 호출부가 바뀜.

MissionExecutor 쪽에 다음이 있다고 전제함: `navigator`, `spin_executor`,
`mission_exec_cfg`, `_stall_events`, `_cmd_vel_pub`, `_check_amcl_jump()`,
`_get_current_pose_from_tf()`, `_get_current_yaw_from_tf()`.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav2_simple_commander.robot_navigator import TaskResult

try:
    from utils.stall_logger import StallWatcher
except ImportError:
    from mission_execution.utils.stall_logger import StallWatcher


class Nav2DriveMixin:
    """nav2 주행/회전 액션 래퍼. MissionExecutor에 믹스인으로 합쳐 쓰는 용도임."""

    def _spin_action(self, delta_yaw, label='rotate'):
        """delta_yaw(rad)만큼 제자리 회전(nav2_msgs/action/Spin)을 1회 시도하고 성공 여부를 반환함."""
        spin_time_allowance = self.mission_exec_cfg.get('spin_time_allowance_sec', 15.0)
        self.navigator.spin(spin_dist=delta_yaw, time_allowance=int(spin_time_allowance))

        stall_watcher = StallWatcher(f"{label} [rotate]", stall_threshold_sec=self._stall_threshold_sec())
        try:
            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self.navigator, timeout_sec=0.01)
                self.spin_executor.spin_once(timeout_sec=0.0)
                if self._check_amcl_jump():
                    self.navigator.cancelTask()
                    self._stall_events.extend(stall_watcher.finalize())
                    return False
                feedback = self.navigator.getFeedback()
                angular_dist = getattr(feedback, 'angular_distance_traveled', None) if feedback else None
                stall_watcher.update(angular_dist)
                self._set_nav_status('Spin', label=label, progress_kind='angular_distance_traveled',
                                      progress_value=angular_dist)
                time.sleep(0.05)
            self._stall_events.extend(stall_watcher.finalize())

            result = self.navigator.getResult()
            if result != TaskResult.SUCCEEDED:
                print(f"[!] Warning: Spin action did not succeed (result={result}).")
                return False
            return True
        finally:
            self._set_nav_status(None)

    def _backup_for_clearance(self, label='rotate'):
        """
        Spin이 실패했을 때 벽에서 살짝 물러나 여유를 만듦(nav2_msgs/action/BackUp).

        벽에 바짝 붙은 자리에서는 Spin의 사전 충돌 체크(simulate_ahead_time)가
        정지 상태만으로도 걸려 실패하므로, 뒤로 조금 물러나 여유를 만듦.
        정체 복구 3단계 중 2단계임(DETAILS.md §3 참고).
        """
        backup_dist = self.mission_exec_cfg.get('spin_retry_backup_dist_m', 0.15)
        backup_speed = self.mission_exec_cfg.get('spin_retry_backup_speed_mps', 0.05)
        backup_time_allowance = self.mission_exec_cfg.get('backup_time_allowance_sec', 10.0)

        self.navigator.backup(backup_dist=backup_dist, backup_speed=backup_speed,
                               time_allowance=int(backup_time_allowance))
        stall_watcher = StallWatcher(f"{label} [backup]", stall_threshold_sec=self._stall_threshold_sec())
        try:
            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self.navigator, timeout_sec=0.01)
                self.spin_executor.spin_once(timeout_sec=0.0)
                if self._check_amcl_jump():
                    self.navigator.cancelTask()
                    self._stall_events.extend(stall_watcher.finalize())
                    return False
                feedback = self.navigator.getFeedback()
                dist_traveled = getattr(feedback, 'distance_traveled', None) if feedback else None
                stall_watcher.update(dist_traveled)
                self._set_nav_status('BackUp', label=label, progress_kind='distance_traveled',
                                      progress_value=dist_traveled)
                time.sleep(0.05)
            self._stall_events.extend(stall_watcher.finalize())
            return self.navigator.getResult() == TaskResult.SUCCEEDED
        finally:
            self._set_nav_status(None)

    def _direct_cmd_vel_rotate(self, target_yaw, label='rotate'):
        """
        Nav2 Spin(behavior_server)이 backup 후 재시도까지 실패했을 때의 최종
        폴백 - nav2를 아예 거치지 않고 /cmd_vel을 직접 발행해 제자리 회전시킴.
        costmap 기반 사전 충돌 체크를 하지 않음.

        충돌 체크 없이 돌려도 된다고 보는 근거와 남겨둔 안전장치(AMCL jump 감지,
        direct_rotate_timeout_sec)는 DETAILS.md §3의 3단계 escalation 표 참고.
        """
        angular_speed = self.mission_exec_cfg.get('direct_rotate_speed_rad_s', 0.3)
        timeout_sec = self.mission_exec_cfg.get('direct_rotate_timeout_sec', 20.0)
        tolerance_rad = math.radians(self.mission_exec_cfg.get('min_rotation_deg', 3.0))

        print("  [Rotate] Escalating to direct /cmd_vel rotation (bypassing nav2 behavior_server)...")

        start_time = time.time()
        success = False
        try:
            while time.time() - start_time < timeout_sec:
                self.spin_executor.spin_once(timeout_sec=0.0)
                if self._check_amcl_jump():
                    print("  [Rotate] AMCL jump detected during direct rotation - aborting.")
                    break

                _, _, current_yaw = self._get_current_pose_from_tf()
                if current_yaw is None:
                    break

                remaining = math.atan2(math.sin(target_yaw - current_yaw), math.cos(target_yaw - current_yaw))
                self._set_nav_status('DirectCmdVelSpin', label=label, progress_kind='remaining_delta_rad',
                                      progress_value=abs(remaining))

                if abs(remaining) < tolerance_rad:
                    success = True
                    break

                twist = Twist()
                twist.angular.z = math.copysign(angular_speed, remaining)
                self._cmd_vel_pub.publish(twist)
                time.sleep(0.05)
        finally:
            self._cmd_vel_pub.publish(Twist())  # 정지
            self._set_nav_status(None)

        if success:
            print("  [Rotate] Direct /cmd_vel rotation succeeded.")
        else:
            print("  [Rotate] Direct /cmd_vel rotation failed (timeout or TF/AMCL issue). Proceeding anyway.")
        return success

    def _direct_cmd_vel_creep_forward(self, label='drive'):
        """
        _navigate_through_poses_blocking이 취소+후진+재시도(2단계)까지 실패했을
        때의 최종 폴백 - nav2를 아예 거치지 않고 /cmd_vel을 직접 발행해 현재
        heading 그대로 짧게 직진시킴. costmap 기반 사전 충돌 체크를 하지 않음.

        _direct_cmd_vel_rotate와 같은 근거로 안전하다고 봄(DETAILS.md §3의
        3단계 escalation 표 참고). 여기에 더해 거리/속도를 짧고 느리게
        (기본 0.2m @ 0.05m/s) 제한해 실제로 막힌 공간이었을 때의 피해를
        줄이고, AMCL jump 감지와 전체 시간제한을 자체 안전장치로 유지함.
        """
        creep_speed = self.mission_exec_cfg.get('direct_creep_speed_mps', 0.05)
        creep_distance = self.mission_exec_cfg.get('direct_creep_distance_m', 0.2)
        timeout_sec = self.mission_exec_cfg.get('direct_creep_timeout_sec', 15.0)

        print(f"  [Drive] Escalating to direct /cmd_vel forward creep "
              f"({creep_distance}m @ {creep_speed}m/s, bypassing nav2)...")

        start_x, start_y, _ = self._get_current_pose_from_tf()
        if start_x is None:
            print("  [Drive] Direct creep aborted: TF unavailable.")
            return False

        start_time = time.time()
        traveled = 0.0
        try:
            while time.time() - start_time < timeout_sec and traveled < creep_distance:
                self.spin_executor.spin_once(timeout_sec=0.0)
                if self._check_amcl_jump():
                    print("  [Drive] AMCL jump detected during direct creep - aborting.")
                    break

                current_x, current_y, _ = self._get_current_pose_from_tf()
                if current_x is None:
                    break
                traveled = math.hypot(current_x - start_x, current_y - start_y)
                self._set_nav_status('DirectCmdVelCreep', label=label, progress_kind='distance_traveled',
                                      progress_value=traveled)

                twist = Twist()
                twist.linear.x = creep_speed
                self._cmd_vel_pub.publish(twist)
                time.sleep(0.05)
        finally:
            self._cmd_vel_pub.publish(Twist())  # 정지
            self._set_nav_status(None)

        print(f"  [Drive] Direct /cmd_vel creep moved {traveled:.2f}m "
              f"(target {creep_distance}m) - resuming nav2 for the remaining path.")
        return traveled > 0.0

    def _pick_rotation_aim_pose(self, seg_poses):
        """
        sub-segment 주행 전 제자리 회전에서 "어느 점을 향해 돌 것인가"를 고름 -
        마지막 점(seg_poses[-1])이 아니라 실제로 다음에 주행할 점을 봐야 함.

        마지막 점을 조준하면 중간 점들을 건너뛰고 코너를 미리 질러버려,
        조준선이 통로가 아니라 벽을 향할 수 있음(HISTORY.md §22).

        현재 위치와 거의 겹치는 점을 조준하면 atan2가 노이즈가 되므로
        (coverage sub-segment의 첫 점은 직전 구간 끝점과 같은 좌표일 수 있음),
        rotate_aim_min_dist_m 이상 떨어진 첫 점을 고르고, 그런 점이 없으면
        기존 동작대로 마지막 점으로 폴백함.
        """
        min_dist = self.mission_exec_cfg.get('rotate_aim_min_dist_m', 0.15)
        current_x, current_y, _ = self._get_current_pose_from_tf()
        if current_x is None:
            print("[!] Warning: TF lookup failed while picking rotation aim - falling back to the last point.")
            return seg_poses[-1]

        for pose in seg_poses:
            dist = math.hypot(pose.pose.position.x - current_x,
                              pose.pose.position.y - current_y)
            if dist >= min_dist:
                return pose
        return seg_poses[-1]

    def _rotate_in_place_to(self, target_pose, label='rotate'):
        """
        현재 위치에서 target_pose(위치)를 향하도록 회전함.

        target_pose.orientation을 그대로 쓰지 않음 - 그 값은 "target_pose에
        도착한 뒤 다음 지점을 향해야 할 방향"으로 저장된 값이라(translator.py의
        forward-looking 방식), 아직 target_pose에 도착 전인 지금 그 방향을 미리
        향하면 코너 직전 지점에서 코너를 건너뛰고 그 다음 방향을 미리 보게 됨.

        따라서 "현재 위치 -> target_pose 위치"를 atan2로 직접 계산해서 목표각으로 씀.

        실패 시 Spin -> BackUp 후 Spin 재시도 -> _direct_cmd_vel_rotate 순으로
        에스컬레이션함(DETAILS.md §3 표). 2단계에서 물러난 자리는 사방이 트여
        있으므로 "away 방향"이 아니라 바로 최종 목표각으로 한 번에 회전함 -
        원래 지점으로의 복귀는 다음 구간의 직선 주행이 담당함.
        """
        current_x, current_y, current_yaw = self._get_current_pose_from_tf()
        if current_yaw is None:
            return False

        dx = target_pose.pose.position.x - current_x
        dy = target_pose.pose.position.y - current_y
        if math.hypot(dx, dy) < 1e-3:
            print("  [Rotate] Target is at current position. Skipping spin.")
            return True

        target_yaw = math.atan2(dy, dx)
        delta_yaw = target_yaw - current_yaw
        delta_yaw = math.atan2(math.sin(delta_yaw), math.cos(delta_yaw))

        min_rotation_rad = math.radians(self.mission_exec_cfg.get('min_rotation_deg', 3.0))
        if abs(delta_yaw) < min_rotation_rad:
            print(f"  [Rotate] Already aligned (delta={math.degrees(delta_yaw):.1f}°). Skipping spin.")
            return True

        print(f"  [Rotate] current={math.degrees(current_yaw):.1f}°, "
            f"target={math.degrees(target_yaw):.1f}° (toward next goal), delta={math.degrees(delta_yaw):.1f}°")

        if self._spin_action(delta_yaw, label=label):
            return True

        print("  [Rotate] Spin failed (likely collision pre-check near wall) - "
              "backing up for clearance and retrying...")
        if not self._backup_for_clearance(label=label):
            print("  [Rotate] Backup also failed too - escalating to direct /cmd_vel rotation.")
            return self._direct_cmd_vel_rotate(target_yaw, label=label)

        current_x, current_y, current_yaw = self._get_current_pose_from_tf()
        if current_yaw is None:
            return False
        dx = target_pose.pose.position.x - current_x
        dy = target_pose.pose.position.y - current_y
        retry_target_yaw = math.atan2(dy, dx)
        retry_delta_yaw = math.atan2(math.sin(retry_target_yaw - current_yaw), math.cos(retry_target_yaw - current_yaw))
        print(f"  [Rotate] Retrying with clearance: current={math.degrees(current_yaw):.1f}°, "
              f"target={math.degrees(retry_target_yaw):.1f}°, delta={math.degrees(retry_delta_yaw):.1f}°")
        if self._spin_action(retry_delta_yaw, label=label):
            return True

        print("  [Rotate] Spin retry after backup also failed - escalating to direct /cmd_vel rotation.")
        return self._direct_cmd_vel_rotate(retry_target_yaw, label=label)

    def _stall_threshold_sec(self):
        return self.mission_exec_cfg.get('stall_log_threshold_sec', 5.0)

    def _set_nav_status(self, action, label=None, progress_kind=None, progress_value=None, recoveries=None):
        """MissionLogger(백그라운드 스레드)가 읽을 "지금 어떤 nav2 액션이 떠
        있고 그 진행 지표가 얼마인지"를 통째로 새 dict로 재할당함(스레드
        세이프성 근거는 mission_logger.py 모듈 docstring 참고). action=None은
        "지금 어떤 nav2 액션도 실행 중이 아님(idle)"을 뜻함."""
        self._nav_status = {
            'action': action,
            'label': label,
            'progress_kind': progress_kind,
            'progress_value': progress_value,
            'number_of_recoveries': recoveries,
        }

    # ------------------------------------------------------------------
    # 직선 주행 (nav2_msgs/action/NavigateToPose, coverage run의 끝점으로 1회 전송)
    # ------------------------------------------------------------------

    def _navigate_to_pose_blocking(self, pose, label='drive-to-pose', behavior_tree='', _retry=False):
        """
        nav2 기본 recovery BT는 스스로 포기하는 시점이 없어 좁은 곳에 막히면
        무한히 재시도하므로, "진행 없음이 nav_stuck_cancel_sec 이상 지속되면
        취소" 워치독을 자체적으로 둠 - StallWatcher(사후 리포트용)와 별개로
        지금 당장 개입하기 위한 추적임. 취소 후 한 번은 후진
        (_backup_for_clearance) 뒤 같은 목표를 재전송함(_retry=True).
        그래도 막히면 세그먼트 실패로 처리함 - 단일 목표 주행에는 3단계
        (direct cmd_vel) 폴백을 두지 않음(DETAILS.md §3 표 참고).
        """
        self.navigator.goToPose(pose, behavior_tree=behavior_tree)

        stall_watcher = StallWatcher(f"{label} [drive]", stall_threshold_sec=self._stall_threshold_sec())
        stuck_cancel_sec = self.mission_exec_cfg.get('nav_stuck_cancel_sec', 45.0)
        last_debug_print_time = time.time()
        last_progress_value = None
        last_progress_time = time.time()
        try:
            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self.navigator, timeout_sec=0.01)
                self.spin_executor.spin_once(timeout_sec=0.0)
                current_time = time.time()

                if self._check_amcl_jump():
                    self.navigator.cancelTask()
                    self._stall_events.extend(stall_watcher.finalize())
                    return False

                feedback = self.navigator.getFeedback()
                remaining = getattr(feedback, 'distance_remaining', None) if feedback else None
                recoveries = getattr(feedback, 'number_of_recoveries', None) if feedback else None
                stall_watcher.update(remaining, recoveries=recoveries)
                self._set_nav_status('NavigateToPose', label=label, progress_kind='distance_remaining',
                                      progress_value=remaining, recoveries=recoveries)

                if remaining is not None and (last_progress_value is None
                                               or abs(remaining - last_progress_value) > 0.05):
                    last_progress_value = remaining
                    last_progress_time = current_time

                if current_time - last_progress_time >= stuck_cancel_sec:
                    print(f"[!] {label}: stuck {stuck_cancel_sec:.0f}s+ with no real progress "
                          f"(distance_remaining={remaining}, recoveries={recoveries}) - canceling nav2 task.")
                    self.navigator.cancelTask()
                    self._stall_events.extend(stall_watcher.finalize())
                    if _retry:
                        print(f"  [!] {label}: retry also got stuck. Giving up.")
                        return False
                    print(f"  [!] {label}: backing up and retrying once...")
                    self._backup_for_clearance(label=label)
                    return self._navigate_to_pose_blocking(pose, label=label, behavior_tree=behavior_tree, _retry=True)

                if current_time - last_debug_print_time >= 1.0:
                    if remaining is not None:
                        print(f"  ├─ Driving swath... distance remaining: {remaining:.2f}m")
                    last_debug_print_time = current_time

                time.sleep(0.05)
            self._stall_events.extend(stall_watcher.finalize())

            result = self.navigator.getResult()
            if result != TaskResult.SUCCEEDED:
                print(f"[-] Swath drive ended without SUCCEEDED (result={result}).")
                return False
            return True
        finally:
            self._set_nav_status(None)

    def _navigate_through_poses_blocking(self, seg_poses, label='drive-through-poses', behavior_tree='', _retry=False, _creeped=False):
        """
        seg_poses 전체를 NavigateThroughPoses(goThroughPoses)로 한 번에 전달함.
        goToPose(end_pose)만 보내면 중간 지점(특히 코너 꼭짓점)을 글로벌 플래너가
        반드시 지나가야 할 이유가 없어 코너를 넓게 잘라가며 지나가는 문제가
        생김. NavigateThroughPoses는 리스트의 모든 (x,y)를 반드시 통과해야
        하는 지점으로 취급하므로 코너 꼭짓점(seg_poses[0])을 실제로 스치듯
        지나가도록 강제할 수 있음.

        주의: 중간 지점들의 orientation은 글로벌 플래너가 강제하지 않음
        (위치만 통과 지점으로 취급됨). 최종 목표(seg_poses[-1])의 orientation만
        도착 시 정렬 대상이 됨.

        진행 없음이 nav_stuck_cancel_sec 이상 지속되면 취소하고,
        goThroughPoses -> _backup_for_clearance 후 재시도 ->
        _direct_cmd_vel_creep_forward 후 마지막 재시도 순으로 에스컬레이션함
        (DETAILS.md §3 표 참고). 이마저 막히면 최종 포기.
        """
        self.navigator.goThroughPoses(seg_poses, behavior_tree=behavior_tree)

        stall_watcher = StallWatcher(f"{label} [drive]", stall_threshold_sec=self._stall_threshold_sec())
        stuck_cancel_sec = self.mission_exec_cfg.get('nav_stuck_cancel_sec', 45.0)
        last_debug_print_time = time.time()
        last_progress_value = None
        last_progress_time = time.time()
        try:
            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self.navigator, timeout_sec=0.01)
                self.spin_executor.spin_once(timeout_sec=0.0)
                current_time = time.time()

                if self._check_amcl_jump():
                    self.navigator.cancelTask()
                    self._stall_events.extend(stall_watcher.finalize())
                    return False

                feedback = self.navigator.getFeedback()
                remaining = getattr(feedback, 'distance_remaining', None) if feedback else None
                n_left = getattr(feedback, 'number_of_poses_remaining', None) if feedback else None
                recoveries = getattr(feedback, 'number_of_recoveries', None) if feedback else None
                stall_watcher.update(remaining, recoveries=recoveries)
                self._set_nav_status('NavigateThroughPoses', label=label, progress_kind='distance_remaining',
                                      progress_value=remaining, recoveries=recoveries)

                if remaining is not None and (last_progress_value is None
                                               or abs(remaining - last_progress_value) > 0.05):
                    last_progress_value = remaining
                    last_progress_time = current_time

                if current_time - last_progress_time >= stuck_cancel_sec:
                    print(f"[!] {label}: stuck {stuck_cancel_sec:.0f}s+ with no real progress "
                          f"(distance_remaining={remaining}, recoveries={recoveries}) - canceling nav2 task.")
                    self.navigator.cancelTask()
                    self._stall_events.extend(stall_watcher.finalize())
                    if _creeped:
                        print(f"  [!] {label}: still stuck after direct cmd_vel creep. Giving up.")
                        return False
                    if _retry:
                        print(f"  [!] {label}: retry also got stuck - escalating to direct /cmd_vel forward creep...")
                        self._direct_cmd_vel_creep_forward(label=label)
                        return self._navigate_through_poses_blocking(
                            seg_poses, label=label, behavior_tree=behavior_tree, _retry=True, _creeped=True)
                    print(f"  [!] {label}: backing up and retrying once...")
                    self._backup_for_clearance(label=label)
                    return self._navigate_through_poses_blocking(
                        seg_poses, label=label, behavior_tree=behavior_tree, _retry=True)

                if current_time - last_debug_print_time >= 1.0:
                    # if remaining is not None:
                    #     print(f"  ├─ Driving through {len(seg_poses)} points... "
                    #         f"distance remaining: {remaining:.2f}m, poses left: {n_left}")
                    last_debug_print_time = current_time

                time.sleep(0.05)
            self._stall_events.extend(stall_watcher.finalize())

            result = self.navigator.getResult()
            if result != TaskResult.SUCCEEDED:
                print(f"[-] Through-poses drive ended without SUCCEEDED (result={result}).")
                return False
            return True
        finally:
            self._set_nav_status(None)
