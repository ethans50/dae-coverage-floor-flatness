# dae_coverage_floor_flatness/mission_execution/localization_mixin.py
"""
MissionExecutor의 위치 추정(AMCL/TF) 관련 믹스인.

AMCL 초기 위치 주입과 수렴 대기, 미션 시작 지점 계산, 주행 중 AMCL pose 모니터링,
점프(순간 이동) 감지, `map->base_link` TF 조회를 담당함. 실제 주행 제어는
`nav2_drive_mixin.py`가 하고 여기서는 "로봇이 지금 어디에 있다고 보는가"만 다룸.

믹스인으로 둔 이유는 `nav2_drive_mixin.py`와 같음 - `self._check_amcl_jump()`,
`self._quaternion_to_yaw(...)` 같은 다른 곳의 호출부(`utils/mission_logger.py` 포함)를
그대로 두기 위함임.

MissionExecutor 쪽에 다음이 있다고 전제함: `navigator`, `spin_executor`,
`tf_buffer`, `mission_exec_cfg`, `env_cfg`, `is_sim`, `map_bounds`,
`goal_poses`, `path_history`, `current_amcl_x/y`, `last_valid_amcl_pose`,
`max_allowed_jump`.
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped


class LocalizationMixin:
    """AMCL 초기화/모니터링과 TF 기반 현재 pose 조회. MissionExecutor에 믹스인함."""

    def _target_verification_callback(self, msg):
        self.verified_amcl_x = msg.pose.pose.position.x
        self.verified_amcl_y = msg.pose.pose.position.y
        self.verified_amcl_yaw = self._quaternion_to_yaw(msg.pose.pose.orientation)

    def _initialize_localization(self):
        """
        AMCL 초기 위치 수렴 스테이지 제어.
        수렴 실패/타임아웃 시 안전하게 시스템을 다운시키고 에러 코드로 종료함.

        '/amcl_pose' Subscription은 self(MissionExecutor) 노드에 생성함.
        따라서 이 단계의 spin은 self를 기준으로 수행함.
        """
        print("[*] Entering Robust AMCL Initialization Stage...")

        self.sub_verify = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._target_verification_callback, 10
        )
        self.initial_pose = self._mission_start_pose

        start_verify_time = time.time()
        is_localization_safe = False
        publish_interval = 0.5
        last_publish_time = 0.0

        print("[*] Dynamically injecting Initial Pose until AMCL responds...")
        while time.time() - start_verify_time < 20.0:
            self.spin_executor.spin_once(timeout_sec=0.05)
            current_time = time.time()

            if self.verified_amcl_x == 0.0 or self.verified_amcl_x is None:
                if current_time - last_publish_time >= publish_interval:
                    self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
                    self.initial_pose.pose.position.z = 0.0
                    print(f"  └─> [Pulse] Sending Initial Pose. Stamp Sec: {self.initial_pose.header.stamp.sec}")
                    self.navigator.setInitialPose(self.initial_pose)
                    last_publish_time = current_time
            else:
                dx = self.verified_amcl_x - self.initial_pose.pose.position.x
                dy = self.verified_amcl_y - self.initial_pose.pose.position.y
                error_dist = (dx ** 2 + dy ** 2) ** 0.5

                target_yaw = self._quaternion_to_yaw(self.initial_pose.pose.orientation)
                yaw_error = math.atan2(
                    math.sin(self.verified_amcl_yaw - target_yaw),
                    math.cos(self.verified_amcl_yaw - target_yaw)
                )
                yaw_error_deg = abs(math.degrees(yaw_error))
                max_yaw_error_deg = self.mission_exec_cfg.get('initial_pose_max_yaw_error_deg', 30.0)

                if error_dist > 0.5:
                    print(f"\n[!!! CRITICAL INITIALIZATION BLOCKED !!!] AMCL initialized in the WRONG ROOM!")
                    print(f"[-] Target: ({self.initial_pose.pose.position.x:.2f}, {self.initial_pose.pose.position.y:.2f})")
                    print(f"[-] AMCL Refused and went to: ({self.verified_amcl_x:.2f}, {self.verified_amcl_y:.2f})")
                    self.navigator.cancelTask()
                    self._notify_surface_profiling_stop(success=False, message="AMCL initialized in the wrong room.")
                    self.destroy_subscription(self.sub_verify)
                    self.spin_executor.remove_node(self)
                    self.destroy_node()
                    self.navigator.destroy_node()
                    rclpy.shutdown()
                    sys.exit(1)
                elif yaw_error_deg > max_yaw_error_deg:
                    # 위치는 맞아도 heading이 크게 어긋나면 map->odom TF 자체가 회전된 채로
                    # 굳어져서, 이후 주행 전체가 일관되게 삐뚤어져 보임(위치 오차 체크만으로는
                    # 못 잡음) - 물리적 배치 heading이 출력된 값과 실제로 다를 때 재현됨.
                    print(f"\n[!!! CRITICAL INITIALIZATION BLOCKED !!!] AMCL position matched but "
                          f"HEADING is off by {yaw_error_deg:.1f}° (max allowed: {max_yaw_error_deg:.1f}°)!")
                    print(f"[-] Target heading: {math.degrees(target_yaw):.1f}°, "
                          f"AMCL converged heading: {math.degrees(self.verified_amcl_yaw):.1f}°")
                    print(f"[-] Check that the robot was physically placed facing the printed runway heading.")
                    self.navigator.cancelTask()
                    self._notify_surface_profiling_stop(success=False, message="AMCL initialized with wrong heading.")
                    self.destroy_subscription(self.sub_verify)
                    self.spin_executor.remove_node(self)
                    self.destroy_node()
                    self.navigator.destroy_node()
                    rclpy.shutdown()
                    sys.exit(1)
                else:
                    print(f"\n[+] AMCL Successfully aligned within safe zone "
                          f"(Error: {error_dist:.3f}m, Yaw error: {yaw_error_deg:.1f}°).")
                    is_localization_safe = True
                    break
            time.sleep(0.05)

        if not is_localization_safe:
            print("[-] localization verification Failed.")
            self._notify_surface_profiling_stop(success=False, message="Localization verification timed out.")
            self.destroy_subscription(self.sub_verify)
            self.spin_executor.remove_node(self)
            self.destroy_node()
            self.navigator.destroy_node()
            rclpy.shutdown()
            sys.exit(1)

        wait_sec = self.mission_exec_cfg.get('post_localization_wait_sec', 3.0)
        if wait_sec > 0:
            print(f"[*] Holding position for {wait_sec:.1f}s to let AMCL settle...")
            wait_start = time.time()
            while time.time() - wait_start < wait_sec:
                self.spin_executor.spin_once(timeout_sec=0.05)
                time.sleep(0.05)

        if self.is_sim:
            print("[*] Clearing costmaps explicitly after simulation teleport & AMCL convergence...")
            self.navigator.clearAllCostmaps()

        self.destroy_subscription(self.sub_verify)
        print("[+] Initialization Stage Cleared. Moving to Path Sampling...")

    def _compute_mission_start_pose(self):
        """미션이 실제로 시작해야 하는 물리적 지점(로봇을 스폰/배치해야 할
        곳)을 계산해서 반환함.

        boundary_repass가 켜져 있으면, 미션의 진짜 첫 coverage 지점(p0)이
        아니라 거기서 진행방향으로 boundary_repass_distance_m만큼(첫
        sub-segment 길이의 90%로 clamp) 앞선 '러닝스타트' 지점을 반환함 -
        거기서부터 캡처를 켠 채로 p0까지 주행해 들어가는 것 자체가 미션의
        첫 동작이 됨(run_start_prepass가 이 지점에서 p0로 들어가는 동작만
        수행함). enable_boundary_repass=false이거나
        첫 sub-segment가 너무 짧으면 p0 그대로 반환함.

        반환값의 orientation은 p0를 향하는 방향(첫 sub-segment 진행방향의
        반대)임. sim에서는 이 pose가 그대로 Gazebo 텔레포트 좌표가 되고,
        real-world에서는 AMCL 초기 위치 힌트로 쓰이므로 실제 로봇도 이
        좌표/방향에 물리적으로 배치돼야 함 - 아래에서 명확히 출력함.
        """
        goal_poses = self._prepare_goal_poses()
        sub_segments = self._split_into_straight_subsegments(goal_poses)
        first_s, first_e = sub_segments[0]
        seg_poses = goal_poses[first_s:first_e]
        p0 = seg_poses[0]

        enabled = (
            self.mission_exec_cfg.get('enable_boundary_repass', True)
            and self.final_path[first_s]['header'].get('record_pcd', True)
        )
        runway_pose = self._boundary_repass.compute_runway_pose(seg_poses) if enabled else None

        if runway_pose is None:
            print(f"[*] Mission start pose = true coverage start point p0 "
                  f"({p0.pose.position.x:.2f}, {p0.pose.position.y:.2f}) "
                  f"(boundary repass disabled, or first segment too short for a runway).")
            return p0

        q = runway_pose.pose.orientation
        facing_deg = math.degrees(2.0 * math.atan2(q.z, q.w)) % 360
        d = math.hypot(runway_pose.pose.position.x - p0.pose.position.x,
                        runway_pose.pose.position.y - p0.pose.position.y)
        print(f"[*] Mission start pose = boundary-repass runway point "
              f"({runway_pose.pose.position.x:.2f}, {runway_pose.pose.position.y:.2f}), "
              f"facing {facing_deg:.0f}° toward the true coverage start "
              f"({p0.pose.position.x:.2f}, {p0.pose.position.y:.2f}), {d:.2f}m away.")
        if not self.is_sim:
            print("[!] REAL-WORLD: place the robot physically at this runway point/heading "
                  "BEFORE starting this executor (not at the coverage start point) - "
                  "AMCL initializes from this pose.")

        return runway_pose

    # ------------------------------------------------------------------
    # 주행 모니터링 콜백
    # ------------------------------------------------------------------

    def _amcl_monitor_callback(self, msg):
        self.current_amcl_x = msg.pose.pose.position.x
        self.current_amcl_y = msg.pose.pose.position.y
        curr_time = time.time()

        # 5Hz 샘플링 (약 0.5초 간격으로 기록)
        if curr_time - self._last_record_time >= 0.5:
            self.path_history.append([curr_time, self.current_amcl_x, self.current_amcl_y])
            self._last_record_time = curr_time

    # ------------------------------------------------------------------
    # AMCL 점프 감지 (여러 모니터링 루프에서 공용으로 재사용)
    # ------------------------------------------------------------------

    def _check_amcl_jump(self):
        """
        직전에 기록된 AMCL pose 대비 순간 이동 거리가 임계치(self.max_allowed_jump)를
        넘으면 '점프 후보'로 기록함. 하지만 단발성 점프(예: 긴 직선 구간을 도는
        동안 누적된 dead-reckoning 오차가 AMCL의 정상적인 재정렬로 한 번에 보정되는
        경우)는 실제로는 위험이 아니라 오히려 위치 추정이 더 정확해진 것이므로,
        그것만으로 미션을 중단시키지 않음. amcl_jump_window_sec 안에
        amcl_jump_count_threshold번 이상 반복될 때만 진짜 비상(로컬라이제이션
        붕괴, 텔레포트 등)으로 간주해 True를 반환함.

        중요: last_valid_amcl_pose는 점프 판정 여부와 무관하게 '항상' 현재 값으로
        갱신함 - 판정 순간에만 갱신을 건너뛰면, AMCL이 이미 새로운 위치에
        안정적으로 자리잡은 뒤에도 계속 옛날 기준점과 비교해 '같은 점프'를
        영원히 재판정하는 고착 상태가 생길 수 있음(비상 상황에서 절대 복구되지
        않고 미션이 항상 중단됨).
        """
        if self.current_amcl_x is None or self.current_amcl_y is None:
            return False

        is_emergency = False

        if self.last_valid_amcl_pose is not None:
            dx = self.current_amcl_x - self.last_valid_amcl_pose[0]
            dy = self.current_amcl_y - self.last_valid_amcl_pose[1]
            jump_distance = (dx ** 2 + dy ** 2) ** 0.5

            if jump_distance > self.max_allowed_jump:
                now = time.time()
                # 윈도우 밖으로 벗어난 오래된 기록은 버림
                self.amcl_jump_timestamps = [
                    t for t in self.amcl_jump_timestamps if now - t <= self.amcl_jump_window_sec
                ]
                self.amcl_jump_timestamps.append(now)

                if len(self.amcl_jump_timestamps) >= self.amcl_jump_count_threshold:
                    print(f"\n[!!! CRITICAL EMERGENCY !!!] AMCL jumped {len(self.amcl_jump_timestamps)} times "
                          f"within {self.amcl_jump_window_sec:.1f}s (latest: {jump_distance:.3f}m). "
                          f"Treating as localization failure.")
                    is_emergency = True
                else:
                    print(f"[*] AMCL correction observed: {jump_distance:.3f}m "
                          f"({len(self.amcl_jump_timestamps)}/{self.amcl_jump_count_threshold} within "
                          f"{self.amcl_jump_window_sec:.1f}s window). Treating as a normal re-localization, "
                          f"not aborting.")

        # 점프 판정 여부와 무관하게 항상 갱신함
        self.last_valid_amcl_pose = (self.current_amcl_x, self.current_amcl_y)

        return is_emergency

    # ------------------------------------------------------------------
    # TF(map->base_link) 기반 현재 pose 조회
    # ------------------------------------------------------------------

    def _quaternion_to_yaw(self, q):
        # 평면 회전만 다루므로 x=y=0을 가정하고 z, w만으로 yaw를 계산함.
        return 2.0 * math.atan2(q.z, q.w)

    def _lookup_base_link_transform(self):
        """
        map->base_link TF를 조회함. self.spin_executor는 백그라운드 스레드 없이
        여러 blocking 루프 안에서 수동으로 spin_once되는 방식이라(예:
        _rotate_in_place_to의 대기 루프), tf2_ros.Buffer의 built-in timeout
        (콜백 알림으로 깨어나는 방식)이 여기서는 작동하지 않음 - 이 호출
        스레드가 곧 유일한 spin 주체라서, timeout만 넘기면 그 시간 동안 아무
        콜백도 못 돌고 그냥 실패함.

        run_start_prepass가 첫 액션으로 이 조회를 호출하는데, AMCL 수렴 직후
        시점과 매우 가까워서 self.tf_buffer/tf_listener가 생성된 지 얼마 안 돼
        버퍼에 이 조회 시각까지의 이력이 아직 없어 ExtrapolationException
        ("Requested time ... but the earliest data is at time ...")이 간헐적으로
        발생할 수 있음. spin_once를 직접
        반복 펌핑하며 짧게 재시도해 흡수함.
        """
        timeout_sec = self.mission_exec_cfg.get('tf_lookup_retry_sec', 1.0)
        deadline = time.time() + timeout_sec
        last_err = None
        while True:
            try:
                return self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            except Exception as e:
                last_err = e
                if time.time() >= deadline:
                    print(f"[!] TF lookup failed for map->base_link after "
                          f"{timeout_sec:.1f}s retry: {last_err}")
                    return None
                self.spin_executor.spin_once(timeout_sec=0.05)
                time.sleep(0.02)

    def _get_current_yaw_from_tf(self):
        trans = self._lookup_base_link_transform()
        if trans is None:
            return None
        return self._quaternion_to_yaw(trans.transform.rotation)

    def _get_current_pose_from_tf(self):
        """TF(map->base_link)에서 현재 (x, y, yaw)를 함께 읽어옴."""
        trans = self._lookup_base_link_transform()
        if trans is None:
            return None, None, None
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        yaw = self._quaternion_to_yaw(trans.transform.rotation)
        return x, y, yaw
