#!/usr/bin/env python3
# dae_coverage_floor_flatness/mission_execution/imu_tilt_broadcaster.py
"""IMU 실측 roll/pitch를 base_footprint->base_link TF로 주입함.

base_footprint는 Nav2/AMCL이 평면으로 가정하는 기준 프레임이라 URDF에 두지 않고
(urdf/turtlebot3_waffle.urdf.xacro 참고), 이 노드가 IMU 방향값으로 직접
base_footprint->base_link 변환을 발행함. URDF 조인트 대신 tf2 브로드캐스트로
처리하는 이유는 물리엔진(Gazebo)에 액추에이터 없는 자유도를 새로 만들지 않기
위함임 - odom->base_footprint를 diff_drive 플러그인이 URDF 밖에서 발행하는 것과
같은 패턴. yaw는 넣지 않음 - base_footprint의 yaw는 이미 odom/AMCL 체인이
담당하므로 여기서 더하면 이중 반영됨.
"""

import math
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations
import yaml

# base_footprint -> base_link 고정 오프셋. urdf/turtlebot3_waffle.urdf.xacro에서
# 제거된 이전 base_joint의 origin xyz 값과 일치해야 함.
BASE_LINK_Z_OFFSET = 0.010


def _load_config():
    try:
        from ament_index_python.packages import get_package_share_directory
        package_share_dir = get_package_share_directory('dae_coverage_floor_flatness')
        config_path = os.path.join(package_share_dir, 'config', 'params.yaml')
    except Exception:
        base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        config_path = os.path.join(base_dir, "config", "params.yaml")
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


class ImuTiltBroadcaster(Node):
    def __init__(self):
        super().__init__('imu_tilt_broadcaster')
        cfg = _load_config().get('mission_execution', {})
        roll_bias_deg, pitch_bias_deg = cfg.get('imu_mount_correction_rpy_deg', [0.0, 0.0])
        self.roll_bias = math.radians(roll_bias_deg)
        self.pitch_bias = math.radians(pitch_bias_deg)
        self.get_logger().info(
            f"IMU mount correction rpy_deg=({roll_bias_deg}, {pitch_bias_deg}) - "
            "subtracted before publishing base_footprint->base_link")

        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Imu, '/imu', self._imu_cb, 50)

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        # 펌웨어가 orientation을 안 채우면 관례상 (0,0,0,0)으로 옴 - 이 경우 발행 안 함.
        if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0:
            return
        roll, pitch, _ = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        roll -= self.roll_bias
        pitch -= self.pitch_bias

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'base_footprint'
        t.child_frame_id = 'base_link'
        t.transform.translation.z = BASE_LINK_Z_OFFSET
        qx, qy, qz, qw = tf_transformations.quaternion_from_euler(roll, pitch, 0.0)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ImuTiltBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
