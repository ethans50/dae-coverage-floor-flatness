#!/usr/bin/env python3
# surface_profiling/test/check_imu_noise.py
"""
IMU 융합을 실제로 구현하기 전에 반드시 먼저 실행할 진단 스크립트.

확인하는 것:
  1. /imu 토픽이 실제로 발행되고 있는가 (메시지 타입, 주기)
  2. TurtleBot3 OpenCR 펌웨어가 이미 자세 추정(orientation 필드)을 채워 보내는가,
     아니면 raw accel/gyro만 오는가 (전자면 별도 AHRS 필터 없이 그대로 쓸 수 있음)
  3. 로봇을 평평한 바닥에 완전히 정지시킨 상태에서 roll/pitch의 평균/표준편차
     (정지 상태의 잡음 바닥 - 이게 목표 결함 크기(mm~cm 단위 z)가 유발하는
     실제 기울기보다 크면, IMU 보정이 신호보다 잡음을 더 키울 수 있음)

사용법:
  1. 로봇을 평평하고 흔들림 없는 바닥에 완전히 정지시켜 둠(주행 X).
  2. 실행: python3 check_imu_noise.py --duration 20
  3. 출력된 topic 발행 여부, orientation 채움 여부, roll/pitch std를 그대로 공유할 것 -
     이 값에 따라 IMU 융합 아키텍처(필터 종류, 신뢰 여부)가 달라짐.
"""

import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import tf_transformations


class ImuNoiseChecker(Node):
    def __init__(self, topic):
        super().__init__('check_imu_noise')
        self.rows = []  # (t, roll, pitch, ax, ay, az, gx, gy, gz)
        self.orientation_filled_count = 0
        self.msg_count = 0
        self.t_first = None
        self.t_last = None
        self.create_subscription(Imu, topic, self._cb, 50)

    def _cb(self, msg: Imu):
        now = time.time()
        if self.t_first is None:
            self.t_first = now
        self.t_last = now
        self.msg_count += 1

        q = msg.orientation
        # 펌웨어가 orientation을 안 채우면 관례상 (0,0,0,0)으로 옴 - 단위 쿼터니언(w=1 포함)이 아님.
        orientation_filled = not (q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0)
        if orientation_filled:
            self.orientation_filled_count += 1
            roll, pitch, _ = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        else:
            roll, pitch = float('nan'), float('nan')

        a, g = msg.linear_acceleration, msg.angular_velocity
        self.rows.append((now, roll, pitch, a.x, a.y, a.z, g.x, g.y, g.z))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--topic', default='/imu')
    ap.add_argument('--duration', type=float, default=20.0, help='정지 상태로 로깅할 시간(초)')
    args = ap.parse_args()

    rclpy.init()
    node = ImuNoiseChecker(args.topic)

    print(f"[*] '{args.topic}' 구독 중 - {args.duration}초 대기 (로봇을 완전히 정지시켜 둘 것)")
    t0 = time.time()
    while time.time() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.1)
    rclpy.shutdown()

    if node.msg_count == 0:
        print(f"[-] '{args.topic}'에서 메시지를 하나도 못 받음.")
        print("    확인할 것: 1) 토픽 이름이 다른가(`ros2 topic list`로 확인),")
        print("               2) OpenCR/turtlebot3_node가 떠 있는가,")
        print("               3) 이 스크립트를 실행한 기기가 로봇과 같은 ROS2 도메인/네트워크에 있는가")
        return

    rate_hz = node.msg_count / (node.t_last - node.t_first) if node.t_last > node.t_first else 0.0
    print(f"\n[*] 수신 메시지 수: {node.msg_count}, 평균 주기: {rate_hz:.1f} Hz")
    print(f"[*] orientation 필드가 채워진 메시지 비율: "
          f"{node.orientation_filled_count}/{node.msg_count} "
          f"({100 * node.orientation_filled_count / node.msg_count:.0f}%)")

    rows = np.array(node.rows)
    roll_deg = np.degrees(rows[:, 1])
    pitch_deg = np.degrees(rows[:, 2])
    ax, ay, az = rows[:, 3], rows[:, 4], rows[:, 5]
    gx, gy, gz = rows[:, 6], rows[:, 7], rows[:, 8]

    if node.orientation_filled_count > 0:
        print("\n[*] 펌웨어 orientation 기반 roll/pitch (정지 상태, 잡음 바닥):")
        print(f"    roll : mean {np.nanmean(roll_deg):+.3f} deg, std {np.nanstd(roll_deg):.4f} deg")
        print(f"    pitch: mean {np.nanmean(pitch_deg):+.3f} deg, std {np.nanstd(pitch_deg):.4f} deg")
    else:
        print("\n[!] orientation 필드가 비어 있음 - 펌웨어가 자세 추정을 안 하고 raw accel/gyro만 보냄.")
        print("    별도 AHRS 필터(예: imu_filter_madgwick)를 얹어야 roll/pitch를 얻을 수 있음.")

    print("\n[*] 참고용 raw 센서 표준편차 (진동/잡음 수준 확인):")
    print(f"    linear_acceleration std (m/s^2): x={ax.std():.4f} y={ay.std():.4f} z={az.std():.4f}")
    print(f"    angular_velocity    std (rad/s): x={gx.std():.4f} y={gy.std():.4f} z={gz.std():.4f}")

    print("\n[*] 판단 기준: 목표 결함 높이(예: 1cm)를 로봇 축간거리(TB3 Waffle 약 0.287m)에 걸친")
    print("    기울기로 환산하면 atan(0.01/0.287) ≈ 2.0 deg. 위 roll/pitch std가 이 값의")
    print("    1/5(0.4 deg) 이상이면, IMU 보정이 신호보다 잡음을 더 키울 수 있음 - 신중히 판단할 것.")


if __name__ == '__main__':
    main()
