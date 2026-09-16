# dae_coverage_floor_flatness/mission_execution/utils/controller_switch.py
"""
coverage/transit 구간별로 nav2 제어 방식을 바꾸는 유틸.

바꾸는 것은 두 종류임 - (1) 어느 BT XML로 주행할지(= 어느 controller_id를
쓸지), (2) controller_server의 속도/회전 임계 파라미터. yaml 정적값은 재시작
없이는 못 바꾸므로 (2)는 `/controller_server/set_parameters`로 실행 중에
전환함. coverage/transit이 서로 다른 컨트롤러를 쓰는 구조 자체는
DETAILS.md §3, 도입 경위는 HISTORY.md §23 참고.
"""

import os
import time

_BT_FILENAMES = {
    'through_poses': 'navigate_through_poses_transit.xml',
    'to_pose': 'navigate_to_pose_transit.xml',
}

_bt_dir = None  # 지연 해석 캐시


def transit_bt_path(kind):
    """
    transit 구간 전용 BT XML의 절대경로를 돌려줌(kind: 'through_poses' | 'to_pose').

    이 BT들은 nav2 기본 BT와 딱 두 가지만 다름 - controller_id가
    FollowPathTransit(Regulated Pure Pursuit)이고, through_poses 쪽은
    RemovePassedGoals radius가 0.3임(HISTORY.md §22/§23). coverage 구간은
    이 함수를 쓰지 않고 기본 BT(DWB)를 그대로 써서 측정 구간의 제어 거동을
    그대로 유지함.

    파일을 못 찾으면 빈 문자열을 돌려줌 - nav2는 빈 값이면 기본 BT를 쓰므로,
    빌드가 덜 된 상황에서도 미션이 죽지 않고 기존 동작으로 떨어짐.
    """
    global _bt_dir
    if _bt_dir is None:
        try:
            from ament_index_python.packages import get_package_share_directory
            base = get_package_share_directory('dae_coverage_floor_flatness')
        except Exception:
            base = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
        _bt_dir = os.path.join(base, 'behavior_trees')

    path = os.path.join(_bt_dir, _BT_FILENAMES[kind])
    if not os.path.exists(path):
        print(f"[!] Warning: transit BT not found at {path} - falling back to nav2's default BT (DWB).")
        return ''
    return path


class ControllerSwitcher:
    """
    controller_server의 double 파라미터를 coverage/transit 모드에 맞춰 전환함.

    같은 모드로 연속 호출되면 서비스 호출 자체를 건너뜀(모드별 캐시). 서비스가
    안 떠 있으면 경고만 남기고 False를 돌려줌 - 그 경우 nav2는 yaml 정적값을
    그대로 쓰므로 미션은 계속 진행됨.
    """

    def __init__(self, node, spin_executor, mission_exec_cfg):
        self.node = node
        self.spin_executor = spin_executor
        self.cfg = mission_exec_cfg
        self._param_client = None  # 지연 생성이라 None으로 시작함
        self._current_speed_mode = None
        self._current_angular_mode = None

    def set_angular_dist_threshold(self, mode):
        """
        FollowPath.angular_dist_threshold를 coverage/transit에 따라 바꿈.
        coverage(정밀 측정 구간)는 yaml 기본값을 유지해 정확한 제자리 회전을
        쓰고, transit(코너/문지방 통과 구간)은 사실상 무제한에 가깝게 풀어서
        RotationShimController의 강제 제자리 회전을 비활성화함 - 좁은 공간에서
        제자리 회전이 벽에 막혀 못 빠져나오는 문제의 대응임.

        transit이 RPP 전용 BT를 쓰게 된 뒤로 transit 쪽 값은 실주행에 영향이
        없지만, BT 파일을 못 찾아 기본 BT(DWB)로 폴백하는 경우를 위해 호출은
        그대로 둠(HISTORY.md §23).
        """
        if mode == self._current_angular_mode:
            return True

        value = (self.cfg.get('coverage_angular_dist_threshold', 0.780) if mode == 'coverage'
                 else self.cfg.get('transit_angular_dist_threshold', 3.140))
        ok = self._apply_doubles(['FollowPath.angular_dist_threshold'], value,
                                 tag='Threshold', what='angular_dist_threshold',
                                 unit='rad', mode=mode)
        if ok:
            self._current_angular_mode = mode
        return ok

    def set_speed_limit(self, mode):
        """
        주행 속도를 coverage/transit에 따라 바꿈. coverage(측정 정밀도가 검증된
        구간)는 0.16m/s를 유지하고, transit(캡처 없이 이동만 하는 구간)은 TB3
        Waffle 모터 스펙상 최대 선속도까지 올려 총 소요시간을 줄임(HISTORY.md §21).

        건드리는 파라미터가 모드별로 다름 - coverage는 DWB의
        `FollowPath.max_vel_x`/`max_speed_xy`, transit은 RPP의
        `FollowPathTransit.desired_linear_vel`임(HISTORY.md §23).
        """
        if mode == self._current_speed_mode:
            return True

        if mode == 'coverage':
            names = ['FollowPath.max_vel_x', 'FollowPath.max_speed_xy']
            value = self.cfg.get('coverage_speed_limit_mps', 0.16)
        else:
            names = ['FollowPathTransit.desired_linear_vel']
            value = self.cfg.get('transit_speed_limit_mps', 0.26)

        ok = self._apply_doubles(names, value, tag='Speed', what='speed limit',
                                 unit='m/s', mode=mode)
        if ok:
            self._current_speed_mode = mode
        return ok

    def _apply_doubles(self, names, value, tag, what, unit, mode):
        """names의 double 파라미터를 전부 value로 설정함. 두 전환 메서드의 공통 배관임."""
        from rcl_interfaces.srv import SetParameters
        from rcl_interfaces.msg import Parameter as RclParameter, ParameterValue, ParameterType

        if self._param_client is None:
            self._param_client = self.node.create_client(
                SetParameters, '/controller_server/set_parameters'
            )

        if not self._param_client.wait_for_service(timeout_sec=2.0):
            print(f"[!] Warning: /controller_server/set_parameters unavailable. "
                  f"{what} switch skipped - nav2 will keep using the static yaml value.")
            return False

        params = [
            RclParameter(
                name=name,
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
            )
            for name in names
        ]
        future = self._param_client.call_async(SetParameters.Request(parameters=params))

        spin_start = time.time()
        while not future.done() and (time.time() - spin_start) < 2.0:
            self.spin_executor.spin_once(timeout_sec=0.1)

        ok = (future.done() and future.result() is not None
              and all(r.successful for r in future.result().results))
        if ok:
            print(f"  [{tag}] {', '.join(names)} -> {value:.3f} {unit} (mode={mode})")
        else:
            print(f"  [!] Warning: Failed to set {what} (mode={mode}).")
        return ok
