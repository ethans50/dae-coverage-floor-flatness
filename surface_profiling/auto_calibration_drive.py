#!/usr/bin/env python3
# surface_profiling/auto_calibration_drive.py
"""
라이다 장착 보정용 4방향(0/90/180/270도) 데이터를 한 번의 명령으로 자동 수집함.

각 방향에서: 캡처 시작 -> 후진 -> 정지 -> 전진 -> 정지 -> 후진(원위치 복귀) -> 정지
-> 캡처 종료를 반복함. 방향 사이에는 개루프로 약 90도 회전한 뒤, 정면 벽면 각도를
라이다로 재측정해 미세 정렬하는 폐루프 보정을 거침. 전진/후진/회전 명령은 SSH로
로봇(Jetson)에서 직접 발행함 - cmd_vel 발행 위치를 기존 수동 절차와 동일하게
유지하기 위함. 캡처 시작/종료는 이 스크립트가 노트북에서 직접 서비스로 호출함.

사용 전 필수 확인:
  1. --detect-only로 먼저 실행해 벽 각도 검출기가 실제 방에서 타당한 값을
     내는지, 로봇을 손으로 살짝 돌려보며 부호가 맞는지 확인함.
  2. --dry-run으로 전체 절차(캡처 시작/종료, 회전 판단)만 먼저 훑어봄.
  3. 로봇 전후 각 방향으로 --reverse-m/--forward-m만큼 이동할 공간이 실제로
     비어 있는지 확인함(장애물 회피 로직 없음).

사용 예:
  python3 auto_calibration_drive.py --host 192.168.0.10 --password 1234 \
    --reverse-m 1.5 --forward-m 3.0
"""

import argparse
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from utils.ssh_robot import JetsonSession  # noqa: E402
from utils.wall_alignment import estimate_front_wall_angle_error  # noqa: E402


class CalibrationDriver(Node):
    def __init__(self, args):
        super().__init__('auto_calibration_drive')
        self.args = args
        self.latest_points = None
        self.latest_stamp = None
        self.create_subscription(PointCloud2, '/velodyne_points', self._pc_cb, 5)

        self.start_cli = self.create_client(Trigger, '/surface_profiling/start_waypoint_capture')
        self.stop_cli = self.create_client(Trigger, '/surface_profiling/stop_waypoint_capture')
        self.finish_cli = self.create_client(Trigger, '/surface_profiling/stop_collection_success')
        for cli, name in [(self.start_cli, 'start_waypoint_capture'),
                           (self.stop_cli, 'stop_waypoint_capture'),
                           (self.finish_cli, 'stop_collection_success')]:
            if not cli.wait_for_service(timeout_sec=10.0):
                raise RuntimeError(f"[-] surface_profiling의 {name} 서비스 연결 실패 - 노드가 떠 있는지 확인")

    def _pc_cb(self, msg):
        raw = pc2.read_points(msg, skip_nans=True, field_names=('x', 'y', 'z'))
        self.latest_points = np.array([(p[0], p[1], p[2]) for p in raw], dtype=np.float32)
        self.latest_stamp = time.time()

    def wait_fresh_scan(self, max_wait=3.0):
        """지금 시점 이후 새로 들어오는 스캔 하나를 기다려 반환함(직전 캐시가 아님)."""
        t0 = time.time()
        seen_before = self.latest_stamp
        while time.time() - t0 < max_wait:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_stamp is not None and self.latest_stamp != seen_before:
                return self.latest_points
        return self.latest_points

    def call_trigger(self, client, label):
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        result = future.result()
        if result is None or not result.success:
            print(f"[!] {label} 서비스 호출 실패: {result}")
        else:
            print(f"[*] {label}: {result.message}")

    def detect_wall_angle(self):
        pts = self.wait_fresh_scan()
        if pts is None or len(pts) == 0:
            return None, 0
        return estimate_front_wall_angle_error(pts)

    def align_to_wall(self, jetson):
        """정면 벽에 수직으로 서도록 폐루프로 미세 회전함."""
        deadline = time.time() + self.args.align_timeout_s
        tol_rad = np.radians(self.args.align_tolerance_deg)
        for it in range(self.args.align_max_iters):
            if time.time() > deadline:
                print(f"[!] 정렬 타임아웃({self.args.align_timeout_s}s) - 현재 상태로 진행")
                return False
            angle_error, n = self.detect_wall_angle()
            if angle_error is None:
                print(f"[!] 벽면 점 부족(n={n}) - 재시도")
                time.sleep(0.5)
                continue
            print(f"[*] [정렬 {it + 1}] 벽 각도 오차 = {np.degrees(angle_error):+.2f} deg (점 {n}개)")
            if abs(angle_error) < tol_rad:
                print(f"[*] 정렬 완료 (오차 {np.degrees(angle_error):.3f} deg < 허용치 {self.args.align_tolerance_deg} deg)")
                return True
            step = float(np.clip(angle_error, -self.args.align_max_step_rad, self.args.align_max_step_rad))
            duration = abs(step) / self.args.rotate_speed
            if not self.args.dry_run:
                jetson.rotate(np.sign(step) * self.args.rotate_speed, duration)
            else:
                print(f"[dry-run] 회전 {np.degrees(step):+.2f} deg 생략")
            time.sleep(0.3)  # 회전 후 새 스캔이 들어올 시간
        print("[!] 최대 반복 횟수 도달 - 현재 상태로 진행")
        return False

    def run_heading_pass(self, jetson, heading_idx):
        print(f"\n=== 방향 {heading_idx + 1}/4: 후진 -> 전진 -> 후진(원위치) 캡처 ===")
        self.call_trigger(self.start_cli, 'start_waypoint_capture')
        time.sleep(1.0)  # 가감속 안정화 대기(문서 권장 절차와 동일)
        if not self.args.dry_run:
            jetson.drive_distance(-self.args.reverse_m, self.args.speed)
            jetson.drive_distance(self.args.forward_m, self.args.speed)
            jetson.drive_distance(-self.args.reverse_m, self.args.speed)
        else:
            print(f"[dry-run] 후진 {self.args.reverse_m}m -> 전진 {self.args.forward_m}m "
                  f"-> 후진 {self.args.reverse_m}m 생략")
        time.sleep(1.0)
        self.call_trigger(self.stop_cli, 'stop_waypoint_capture')

    def run(self):
        jetson = JetsonSession(self.args.host, self.args.user, self.args.password)
        print("[*] Jetson SSH 연결 완료 - 시계 동기화")
        jetson.sync_clock()

        for i in range(4):
            self.run_heading_pass(jetson, i)
            if i < 3:
                print(f"\n=== 방향 {i + 1} -> {i + 2}: 90도 회전 (개루프) ===")
                coarse_duration = np.radians(90) / self.args.rotate_speed
                if not self.args.dry_run:
                    jetson.rotate(self.args.rotate_speed, coarse_duration)
                else:
                    print("[dry-run] 90도 회전 생략")
                print("=== 정면 벽면에 맞춰 미세 정렬 (폐루프) ===")
                self.align_to_wall(jetson)

        self.call_trigger(self.finish_cli, 'stop_collection_success')
        jetson.stop()
        jetson.close()
        print("\n[*] 4방향 캘리브레이션 수집 완료. frames_*.npz를 analyze_z_bias.py로 분석할 것.")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default=os.environ.get('ROBOT_HOST'),
                     help='Jetson IP. 안 주면 ROBOT_HOST 환경변수 사용')
    ap.add_argument('--user', default='waffle')
    ap.add_argument('--password', default=os.environ.get('SSH_PASSWORD'),
                     help='Jetson/노트북 공용 SSH 비밀번호. 안 주면 SSH_PASSWORD 환경변수 사용')
    ap.add_argument('--reverse-m', type=float, default=1.5, help='왕복 각 구간의 후진 거리(m)')
    ap.add_argument('--forward-m', type=float, default=3.0, help='왕복 중앙 구간의 전진 거리(m)')
    ap.add_argument('--speed', type=float, default=0.16, help='m/s, coverage_speed_limit_mps 기본값과 동일')
    ap.add_argument('--rotate-speed', type=float, default=0.3, help='rad/s, 회전 각속도')
    ap.add_argument('--align-tolerance-deg', type=float, default=0.3)
    ap.add_argument('--align-timeout-s', type=float, default=30.0)
    ap.add_argument('--align-max-iters', type=int, default=15)
    ap.add_argument('--align-max-step-rad', type=float, default=np.radians(15),
                     help='한 번에 회전 보정할 최대 각도(rad) - 과도한 한 번의 회전 방지')
    ap.add_argument('--dry-run', action='store_true',
                     help='SSH로 실제 이동/회전 명령을 보내지 않고 절차만 출력함(캡처 서비스는 실제로 호출됨)')
    ap.add_argument('--detect-only', action='store_true',
                     help='주행 없이 벽 각도 검출값만 반복 출력함 - 부호/타당성 검증용')
    args = ap.parse_args()

    if args.host is None:
        ap.error('--host 또는 ROBOT_HOST 환경변수가 필요함')
    if args.password is None:
        ap.error('--password 또는 SSH_PASSWORD 환경변수가 필요함')

    rclpy.init()
    node = CalibrationDriver(args)
    try:
        if args.detect_only:
            print("[*] 벽 각도 검출 전용 모드 - Ctrl+C로 종료")
            while True:
                angle_error, n = node.detect_wall_angle()
                if angle_error is None:
                    print(f"[!] 벽면 점 부족 (n={n})")
                else:
                    print(f"[*] 벽 각도 오차 = {np.degrees(angle_error):+.2f} deg (점 {n}개)")
                time.sleep(0.5)
        else:
            node.run()
    except KeyboardInterrupt:
        print("\n[!] 중단됨")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
