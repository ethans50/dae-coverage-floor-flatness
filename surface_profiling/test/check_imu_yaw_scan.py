#!/usr/bin/env python3
# surface_profiling/test/check_imu_yaw_scan.py
"""
2D 라이다(/scan) 스캔 정합으로 IMU/오도메트리의 yaw 오차를 검증함.

2D 라이다는 수직 벽에 대해 차체 roll/pitch 에 1차로 둔감하므로(범위 변화가 각도의 제곱 수준) tilt 기준으로는
쓰지 못함. 대신 스캔 두 장 사이의 회전량(ICP)은 yaw 변화의 독립 기준이 되므로, 같은 구간에서
  - IMU 자이로 z 적분(OpenCR 필터가 쓰는 센서),
  - IMU orientation 의 yaw 차,
  - 휠 오도메트리 yaw 차
를 비교해 스케일 오차(%)와 바이어스(deg/min)를 구함. 정지 구간은 드리프트 기준이고, 짧은 회전을 여러
각도/속도/방향으로 섞어야 일반화됨.

서브커맨드:
  wiggle   짧은 제자리 회전을 여러 각도·속도·방향으로 자동 수행하며 /scan,/imu,/odom 을 기록함(알짜 회전 0).
  record   Ctrl-C 까지 기록만 함(정지 상태 드리프트 측정, 직접 주행 중 기록용).
  analyze  기록을 읽어 스캔 정합 후 회귀 결과를 출력하고 그래프를 저장함.

특징 없는 열린 공간에서는 ICP 가 불안정하므로 쌍마다 (점 수, 잔차, 초기값 변경 시 일관성)을 검사해
걸러내고, 통과 비율이 낮으면 경고함. 로봇 주변 반경 1m 이상이 비어 있어야 하며 벽/가구가 사방에 있는 곳이 좋음.

사용 예:
  python3 check_imu_yaw_scan.py wiggle --label room1
  python3 check_imu_yaw_scan.py record --label static
  python3 check_imu_yaw_scan.py analyze scan_imu_room1_<ts>.npz
"""

import argparse
import math
import os
import sys
import time

import numpy as np

OUT_DIR = os.path.expanduser('~/dae_floor_maps/analytics/imu_check')


# ---------------------------------------------------------------- 기록 ----

class _Session:
    """/scan,/imu,/odom 을 기록하고, 필요하면 /odom yaw 를 보며 제자리 회전도 수행함."""

    def __init__(self, cmd_topic, scan_topic, odom_topic):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu, LaserScan
        import tf_transformations

        self.rclpy, self.Twist = rclpy, Twist
        self.scans, self.imu, self.odom = [], [], []
        self.meta = None
        self.unw, self._prev = 0.0, None
        sess = self

        class N(Node):
            def __init__(self):
                super().__init__('check_imu_yaw_scan')
                self.pub = self.create_publisher(Twist, cmd_topic, 10)
                self.create_subscription(LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data)
                self.create_subscription(Imu, '/imu', self.on_imu, 50)
                self.create_subscription(Odometry, odom_topic, self.on_odom, 20)

            def on_scan(self, m):
                if sess.meta is None:
                    sess.meta = (m.angle_min, m.angle_increment, m.range_min, m.range_max)
                r = np.asarray(m.ranges, dtype=np.float32)
                sess.scans.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, r))

            def on_imu(self, m):
                q = m.orientation
                if q.x == q.y == q.z == q.w == 0.0:
                    return
                r, p, y = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
                sess.imu.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                                 math.degrees(r), math.degrees(p), math.degrees(y), m.angular_velocity.z))

            def on_odom(self, m):
                q = m.pose.pose.orientation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                if sess._prev is not None:
                    sess.unw += (yaw - sess._prev + math.pi) % (2 * math.pi) - math.pi
                sess._prev = yaw
                sess.odom.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                                  m.pose.pose.position.x, m.pose.pose.position.y, sess.unw))

        rclpy.init()
        self.node = N()

    def spin(self, sec):
        t0 = time.time()
        while self.rclpy.ok() and time.time() - t0 < sec:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def send(self, w):
        t = self.Twist()
        t.angular.z = float(w)
        self.node.pub.publish(t)

    def stop(self):
        for _ in range(10):
            self.send(0.0)
            self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def wait_data(self, timeout=8.0):
        t0 = time.time()
        while time.time() - t0 < timeout and not (self.scans and self.imu and self._prev is not None):
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
        return bool(self.scans and self.imu and self._prev is not None)

    def turn(self, deg, speed, tol_deg=1.5, min_speed=0.12):
        """/odom yaw 변화가 deg 에 닿을 때까지 제자리 회전(목표 근처 감속)."""
        sign, target = (1.0 if deg >= 0 else -1.0), math.radians(abs(deg))
        start, t0 = self.unw, time.time()
        while self.rclpy.ok() and time.time() - t0 < abs(target) / speed * 2 + 8:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            remaining = target - sign * (self.unw - start)
            if remaining <= math.radians(tol_deg):
                break
            ramp = max(0.3, min(1.0, (time.time() - t0) / 1.0))
            self.send(sign * min(speed * ramp, max(min_speed, 1.5 * remaining)))
        self.stop()

    def close(self):
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()

    def save(self, label):
        os.makedirs(OUT_DIR, exist_ok=True)
        n = min(len(r) for _, r in self.scans)
        out = os.path.join(OUT_DIR, f"scan_imu_{label}_{time.strftime('%Y-%m-%d_%H-%M-%S')}.npz")
        np.savez(out, scan_t=np.array([t for t, _ in self.scans]),
                 scan_r=np.stack([r[:n] for _, r in self.scans]), meta=np.array(self.meta),
                 imu=np.array(self.imu), odom=np.array(self.odom))
        print(f"[+] 저장: {out}  (scan {len(self.scans)}, imu {len(self.imu)}, odom {len(self.odom)})")


def cmd_wiggle(a):
    s = _Session(a.cmd_topic, a.scan_topic, a.odom_topic)
    if not s.wait_data():
        s.close()
        sys.exit('[-] /scan, /imu, /odom 중 수신되지 않는 토픽이 있음')
    angles = [25, -25, 50, -50, 90, -90]
    total = sum(sum(abs(x) for x in angles) / math.degrees(r) + a.pause * len(angles) for r in a.rates) * a.repeat
    print(f"[*] 짧은 회전 {len(angles) * len(a.rates) * a.repeat}회(각도 {angles}deg, 속도 {a.rates} rad/s) - "
          f"약 {total:.0f}s. 로봇 주변 반경 1m 이상을 비울 것. 중단은 Ctrl-C")
    try:
        s.spin(a.pause)
        for _ in range(a.repeat):
            for rate in a.rates:
                for ang in angles:
                    s.turn(ang, rate)
                    s.spin(a.pause)
                print(f"  속도 {rate} rad/s 완료", flush=True)
    except KeyboardInterrupt:
        pass
    s.stop()
    s.spin(1.0)
    s.save(a.label)
    s.close()


def cmd_record(a):
    s = _Session(a.cmd_topic, a.scan_topic, a.odom_topic)
    if not s.wait_data():
        s.close()
        sys.exit('[-] /scan, /imu, /odom 중 수신되지 않는 토픽이 있음')
    print("[*] 기록 중 - 끝나면 Ctrl-C")
    try:
        while s.rclpy.ok():
            s.spin(5.0)
            print(f"  scan {len(s.scans)} imu {len(s.imu)} odom {len(s.odom)}", flush=True)
    except KeyboardInterrupt:
        pass
    s.save(a.label)
    s.close()


# ---------------------------------------------------------------- 분석 ----

def _points(r, meta, margin=0.02):
    amin, inc, rmin, rmax = meta
    ang = amin + inc * np.arange(len(r))
    ok = np.isfinite(r) & (r > max(rmin, 0.12)) & (r < rmax * (1 - margin))
    return np.c_[r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])]


def _icp(src, dst, yaw0, tree, iters=40):
    """src(점 집합 j)를 dst(점 집합 i) 프레임으로 옮기는 SE2 를 구함. 반환: (yaw, t, rms, 내점 비율) 또는 None."""
    R = np.array([[math.cos(yaw0), -math.sin(yaw0)], [math.sin(yaw0), math.cos(yaw0)]])
    t = np.zeros(2)
    sched = [0.6, 0.4, 0.25, 0.15] + [0.1] * (iters - 4)
    rms, inl = 1.0, 0.0
    for md in sched:
        p = src @ R.T + t
        d, idx = tree.query(p)
        m = d < md
        if m.sum() < 30:
            return None
        A, B = src[m], dst[idx[m]]
        ca, cb = A.mean(0), B.mean(0)
        U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
        Rn = Vt.T @ U.T
        if np.linalg.det(Rn) < 0:
            Vt[-1] *= -1
            Rn = Vt.T @ U.T
        tn = cb - Rn @ ca
        rms, inl = float(np.sqrt((d[m] ** 2).mean())), float(m.mean())
        if np.abs(Rn - R).max() < 1e-7 and np.abs(tn - t).max() < 1e-6:
            R, t = Rn, tn
            break
        R, t = Rn, tn
    return math.atan2(R[1, 0], R[0, 0]), t, rms, inl


def _unwrap_deg(y):
    return np.degrees(np.unwrap(np.radians(y)))


def cmd_analyze(a):
    from scipy.spatial import cKDTree

    path = a.npz if os.path.exists(a.npz) else os.path.join(OUT_DIR, a.npz)
    z = np.load(path)
    st, sr, meta = z['scan_t'], z['scan_r'], tuple(z['meta'])
    imu, odom = z['imu'], z['odom']
    ti = imu[:, 0]
    imu_yaw = _unwrap_deg(imu[:, 3])
    gyro_int = np.concatenate([[0.0], np.cumsum(0.5 * (imu[1:, 4] + imu[:-1, 4]) * np.diff(ti))]) * 180 / math.pi
    to, odom_yaw = odom[:, 0], np.degrees(odom[:, 3])
    if np.any(np.diff(to) <= 0):
        sys.exit('[-] /odom 타임스탬프가 증가하지 않음(비어 있거나 중복) - 오도메트리 헤더 stamp 확인')

    pts = [_points(r, meta) for r in sr]
    trees = [cKDTree(p) if len(p) >= 30 else None for p in pts]
    rows, rej = [], {'점 부족': 0, 'dt 범위 밖': 0, '정합 실패': 0, '잔차 큼': 0, '초기값 민감(특징 부족)': 0, '오도메트리와 불일치': 0}
    for i in range(len(st) - a.gap):
        j = i + a.gap
        dt = st[j] - st[i]
        if not (0.15 <= dt <= 1.5):
            rej['dt 범위 밖'] += 1
            continue
        if trees[i] is None or trees[j] is None:
            rej['점 부족'] += 1
            continue
        g = math.radians(np.interp(st[j], to, odom_yaw) - np.interp(st[i], to, odom_yaw))
        res = []
        for off in (0.0, math.radians(3), -math.radians(3)):
            r = _icp(pts[j], pts[i], g + off, trees[i])
            if r is None:
                break
            res.append(r)
        if len(res) < 3:
            rej['정합 실패'] += 1
            continue
        yaws = np.array([r[0] for r in res])
        if np.ptp(yaws) > math.radians(a.multistart_tol_deg):
            rej['초기값 민감(특징 부족)'] += 1
            continue
        if res[0][2] > a.max_rms or res[0][3] < a.min_inlier:
            rej['잔차 큼'] += 1
            continue
        if abs(res[0][0] - g) > math.radians(a.max_guess_dev_deg):
            rej['오도메트리와 불일치'] += 1
            continue
        d_scan = math.degrees(res[0][0])
        d_gyro = np.interp(st[j], ti, gyro_int) - np.interp(st[i], ti, gyro_int)
        d_imu = np.interp(st[j], ti, imu_yaw) - np.interp(st[i], ti, imu_yaw)
        d_odom = math.degrees(g)
        rows.append((st[i], dt, d_scan, d_gyro, d_imu, d_odom, res[0][2]))
    R = np.array(rows)
    tot = len(st) - a.gap
    print(f"[*] 스캔 {len(st)}장, 쌍 {tot}개 중 채택 {len(R)}개 ({100 * len(R) / max(tot, 1):.0f}%). 제외 사유: "
          + ", ".join(f"{k} {v}" for k, v in rej.items() if v))
    if len(R) < 30:
        sys.exit("[-] 채택 쌍이 30개 미만 - 특징이 부족한 공간이거나 스캔 범위 밖임(벽이 3m 이내인 곳에서 재측정)")
    if len(R) < 0.3 * tot:
        print("[!] 채택 비율이 30% 미만 - 공간 특징이 부족해 결과가 일부 구간에만 해당함")

    dt, ds = R[:, 1], R[:, 2]
    rate = np.abs(ds) / dt
    names = {'gyro z 적분': 3, 'IMU orientation yaw': 4, '휠 오도메트리': 5}
    print(f"\n전체 회귀  Δsrc = s·Δscan + b·Δt   (Δscan: 스캔 정합 yaw 변화, 기준)")
    allfit = {}
    for nm, c in names.items():
        y = R[:, c]
        X = np.c_[ds, dt]
        k = np.ones(len(y), bool)
        for _ in range(2):  # 3σ 이상치 1회 제거
            coef, *_ = np.linalg.lstsq(X[k], y[k], rcond=None)
            r = y - X @ coef
            k = np.abs(r) < 3 * r[k].std()
        allfit[nm] = coef
        print(f"  {nm:20s} 스케일 오차 {100 * (coef[0] - 1):+.2f}%  바이어스 {coef[1] * 60:+.3f} deg/min  "
              f"잔차 std {r[k].std():.3f} deg  (n={k.sum()})")

    print("\n회전 속도 구간별(정지 구간은 드리프트, 회전 구간은 전체 회귀의 바이어스를 뺀 뒤 스케일): [|rate| deg/s]")
    bins = [(0, 1, '정지(<1)'), (1, 8, '느림(1~8)'), (8, 20, '중간(8~20)'), (20, 1e9, '빠름(>20)')]
    for nm, c in names.items():
        print(f"  {nm}")
        for lo, hi, lab in bins:
            m = (rate >= lo) & (rate < hi)
            if m.sum() < 10:
                print(f"    {lab:10s} n={m.sum():4d}  (표본 부족)")
                continue
            if lo == 0:
                e = (R[m, c] - ds[m]) / dt[m] * 60
                print(f"    {lab:10s} n={m.sum():4d}  yaw 드리프트 {e.mean():+.3f}±{e.std() / math.sqrt(m.sum()):.3f} deg/min")
            else:
                for sgn, nmx in ((1, '반시계'), (-1, '시계')):
                    mm = m & (np.sign(ds) == sgn)
                    if mm.sum() < 10:
                        continue
                    yy = R[mm, c] - allfit[nm][1] * dt[mm]  # 바이어스는 방향에 따라 부호가 달라 보이므로 먼저 제거
                    s = (yy * ds[mm]).sum() / (ds[mm] ** 2).sum()
                    print(f"    {lab:10s} {nmx:3s} n={mm.sum():4d}  스케일 오차 {100 * (s - 1):+.2f}%")

    g = allfit['gyro z 적분']
    print("\n해석")
    if abs(g[0] - 1) > 0.02:
        print(f"  - 자이로 z 스케일 오차 {100 * (g[0] - 1):+.1f}% : 회전량이 그만큼 틀리게 적분됨(오도메트리 yaw가 IMU를 융합하면 경로 휨으로 이어짐).")
    else:
        print("  - 자이로 z 스케일은 ±2% 이내.")
    if abs(g[1] * 60) > 0.5:
        print(f"  - 자이로 바이어스 {g[1] * 60:+.2f} deg/min : 정지 중에도 yaw 가 흐름.")
    print("  - 회전 속도/방향별 스케일이 서로 다르면 단일 보정 상수로는 부족함(비선형 또는 필터 지연).")
    print("  - 이 방법은 yaw 만 검증함. roll/pitch 오차는 2D 라이다로 알 수 없으므로 3D 라이다 평면 적합 비교(check_imu_dynamics.py)를 쓸 것.")
    _plot(a, path, R, names)


def _plot(a, path, R, names):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(1, 3, figsize=(15, 4.5))
    for ax, (nm, c) in zip(axs, names.items()):
        ax.scatter(R[:, 2], R[:, c], s=6, alpha=.5)
        lim = np.abs(R[:, 2]).max() * 1.05
        ax.plot([-lim, lim], [-lim, lim], 'k--', lw=.8)
        ax.set_title({'gyro z 적분': 'gyro z integral', 'IMU orientation yaw': 'IMU orientation yaw', '휠 오도메트리': 'wheel odometry'}[nm])
        ax.set_xlabel('scan-matched dyaw [deg]')
        ax.set_ylabel('source dyaw [deg]')
        ax.grid(alpha=.3)
    fig.tight_layout()
    out = path.replace('.npz', '_yaw.png')
    fig.savefig(out, dpi=110)
    print(f"\n[+] 그래프: {out}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    for n, fn, h in (('wiggle', cmd_wiggle, '짧은 회전 여러 번 + 기록'), ('record', cmd_record, '기록만(Ctrl-C로 종료)')):
        p = sub.add_parser(n, help=h)
        p.add_argument('--label', default='run')
        p.add_argument('--cmd-topic', default='/cmd_vel')
        p.add_argument('--scan-topic', default='/scan')
        p.add_argument('--odom-topic', default='/odom')
        if n == 'wiggle':
            p.add_argument('--rates', type=float, nargs='+', default=[0.2, 0.4, 0.7], help='각속도 목록(rad/s)')
            p.add_argument('--repeat', type=int, default=1)
            p.add_argument('--pause', type=float, default=2.0, help='회전 사이 정지(s). 드리프트 기준 구간')
        p.set_defaults(fn=fn)
    an = sub.add_parser('analyze', help='스캔 정합으로 IMU/오도메트리 yaw 비교')
    an.add_argument('npz')
    an.add_argument('--gap', type=int, default=2, help='비교할 스캔 간격(장). LDS 5Hz 면 2장=0.4s')
    an.add_argument('--max-rms', type=float, default=0.04, help='정합 잔차 상한(m)')
    an.add_argument('--min-inlier', type=float, default=0.5)
    an.add_argument('--multistart-tol-deg', type=float, default=0.3, help='초기값을 ±3deg 바꿨을 때 yaw 허용 차')
    an.add_argument('--max-guess-dev-deg', type=float, default=10.0, help='오도메트리 초기값과 정합 yaw 의 허용 차')
    an.set_defaults(fn=cmd_analyze)
    a = ap.parse_args()
    a.fn(a)


if __name__ == '__main__':
    main()
