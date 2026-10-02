#!/usr/bin/env python3
# dae-coverage-floor-flatness/mission_execution/imu_tilt_broadcaster.py
"""base_footprint->base_link TF를 발행함. use_imu_tilt 에 따라 IMU roll/pitch 를 싣거나 회전 0 을 싣음.

base_footprint는 Nav2/AMCL이 평면으로 가정하는 기준 프레임이라 URDF에 두지 않고
(urdf/turtlebot3_waffle.urdf.xacro 참고), 이 노드가 base_footprint->base_link 변환을 직접
발행함. URDF 조인트 대신 tf2 브로드캐스트로 처리하는 이유는 물리엔진(Gazebo)에 액추에이터
없는 자유도를 새로 만들지 않기 위함임 - odom->base_footprint를 diff_drive 플러그인이 URDF
밖에서 발행하는 것과 같은 패턴. yaw는 넣지 않음 - base_footprint의 yaw는 이미 odom/AMCL
체인이 담당하므로 여기서 더하면 이중 반영됨.

모드(파라미터 use_imu_tilt, 실행 중 `ros2 param set`으로 전환 가능):
  true  /imu 방향값에서 바이어스를 뺀 roll/pitch 를 IMU 메시지마다 발행함.
  false 회전 0 을 일정 주기로 발행함. IMU 상태와 무관하게 TF 체인이 끊기지 않으며, 같은 노드가
        같은 방식(동적 TF)으로 계속 발행하므로 모드를 바꿔도 다른 노드를 재시작할 필요가 없음.

바이어스는 params.yaml 의 imu_mount_correction_rpy_deg_{sim,real}(is_sim 파라미터로 선택)에서 읽음.
현재 모드와 바이어스, 읽은 설정 파일은 시작 로그와 /imu_tilt/status(latched, JSON 문자열)로
알려서 측정 기록에 남길 수 있게 함.
"""

import datetime
import json
import math
import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations
import yaml

# base_footprint -> base_link 고정 오프셋. urdf/turtlebot3_waffle.urdf.xacro에서
# 제거된 이전 base_joint의 origin xyz 값과 일치해야 함.
BASE_LINK_Z_OFFSET = 0.010

# 회전 0 발행 주기(Hz). Nav2/AMCL이 요구하는 TF 갱신 빈도보다 충분히 높게 둠.
IDENTITY_PUBLISH_HZ = 50.0
IMU_STALE_WARN_SEC = 1.0


def _load_config():
    """params.yaml 을 읽어 (설정 dict, 파일 경로)를 반환함."""
    try:
        from ament_index_python.packages import get_package_share_directory
        package_share_dir = get_package_share_directory('dae-coverage-floor-flatness')
        config_path = os.path.join(package_share_dir, 'config', 'params.yaml')
    except Exception:
        base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        config_path = os.path.join(base_dir, "config", "params.yaml")
    with open(config_path, 'r') as f:
        return yaml.safe_load(f), config_path


class ImuTiltBroadcaster(Node):
    def __init__(self):
        super().__init__('imu_tilt_broadcaster')
        self.declare_parameter('use_imu_tilt', False)
        self.declare_parameter('is_sim', False)
        self.tilt_on = bool(self.get_parameter('use_imu_tilt').value)
        is_sim = bool(self.get_parameter('is_sim').value)

        cfg, self.config_path = _load_config()
        mission_cfg = cfg.get('mission_execution', {})
        key = 'imu_mount_correction_rpy_deg_sim' if is_sim else 'imu_mount_correction_rpy_deg_real'
        roll_bias_deg, pitch_bias_deg = mission_cfg.get(key, [0.0, 0.0])
        self.bias_deg = [float(roll_bias_deg), float(pitch_bias_deg)]
        self.roll_bias = math.radians(roll_bias_deg)
        self.pitch_bias = math.radians(pitch_bias_deg)
        self.config_key = key
        self.last_imu_time = None

        self.broadcaster = TransformBroadcaster(self)
        self.status_pub = self.create_publisher(
            String, '/imu_tilt/status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(Imu, '/imu', self._imu_cb, 50)
        self.create_timer(1.0 / IDENTITY_PUBLISH_HZ, self._identity_timer_cb)
        self.create_timer(5.0, self._watchdog_cb)
        self.add_on_set_parameters_callback(self._on_params)

        mtime = datetime.datetime.fromtimestamp(os.path.getmtime(self.config_path)).isoformat(timespec='seconds')
        self.config_mtime = mtime
        self.get_logger().info(
            f"IMU TF mode={'on' if self.tilt_on else 'off'}  {key}=({roll_bias_deg}, {pitch_bias_deg}) "
            f"- subtracted before publishing base_footprint->base_link  "
            f"config={self.config_path} (modified {mtime})")
        self._publish_status()

    def _publish_status(self):
        msg = String()
        msg.data = json.dumps({
            'mode': 'on' if self.tilt_on else 'off',
            'bias_rpy_deg': self.bias_deg,
            'bias_key': self.config_key,
            'config_path': self.config_path,
            'config_mtime': self.config_mtime,
        })
        self.status_pub.publish(msg)

    def _on_params(self, params):
        for p in params:
            if p.name == 'use_imu_tilt':
                if not isinstance(p.value, bool):
                    return SetParametersResult(successful=False, reason='use_imu_tilt must be bool')
                self.tilt_on = p.value
                self.get_logger().info(f"IMU TF mode -> {'on' if self.tilt_on else 'off'}")
                self._publish_status()
        return SetParametersResult(successful=True)

    def _send(self, stamp, roll, pitch):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'base_footprint'
        t.child_frame_id = 'base_link'
        t.transform.translation.z = BASE_LINK_Z_OFFSET
        qx, qy, qz, qw = tf_transformations.quaternion_from_euler(roll, pitch, 0.0)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.broadcaster.sendTransform(t)

    def _identity_timer_cb(self):
        if not self.tilt_on:
            self._send(self.get_clock().now().to_msg(), 0.0, 0.0)

    def _imu_cb(self, msg: Imu):
        self.last_imu_time = self.get_clock().now()
        if not self.tilt_on:
            return
        q = msg.orientation
        # 펌웨어가 orientation을 안 채우면 관례상 (0,0,0,0)으로 옴 - 이 경우 발행 안 함.
        if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0:
            return
        roll, pitch, _ = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self._send(msg.header.stamp, roll - self.roll_bias, pitch - self.pitch_bias)

    def _watchdog_cb(self):
        # on 모드에서 IMU 가 끊기면 TF 도 멈추므로 눈에 띄게 알림.
        if not self.tilt_on:
            return
        if self.last_imu_time is None or \
                (self.get_clock().now() - self.last_imu_time).nanoseconds * 1e-9 > IMU_STALE_WARN_SEC:
            self.get_logger().warn("use_imu_tilt=true 이나 /imu 가 들어오지 않음 - base_footprint->base_link TF 가 멈춰 있음")


def main(args=None):
    rclpy.init(args=args)
    node = ImuTiltBroadcaster()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():  # 종료 신호로 이미 shutdown 된 경우 다시 호출하면 예외가 남
            rclpy.shutdown()


if __name__ == '__main__':
    main()
