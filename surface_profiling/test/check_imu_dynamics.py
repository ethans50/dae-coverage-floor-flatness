#!/usr/bin/env python3
# surface_profiling/test/check_imu_dynamics.py
"""
IMU roll/pitch 보정값 측정(정지)과 주행 중 검증을 한 스크립트로 처리함.

서브커맨드 (모두 /imu 의 원시 orientation 을 쓰며 imu_mount_correction 은 차감하기 전 값임):

  static   정지 상태 N초 기록 -> 헤딩별 평균 저장. 4헤딩(90도 간격)이 모이면 평균을
           imu_mount_correction_rpy_deg_real 후보([roll, pitch], deg)로 출력함.
           단일 헤딩은 바닥 기울기가 섞이므로 4헤딩 평균만 바이어스로 취급함.
  rotate   /odom yaw 를 보며 제자리에서 지정한 각도만큼 돌고 스스로 멈춤(헤딩 전환용, 개루프 명령의
           관성 초과 회전을 피함). 로봇 /cmd_vel 을 받을 수 있는 머신(Jetson 권장)에서 실행함.
  sweep    제자리에서 천천히 연속 회전(기본: 반시계 1바퀴 + 시계 1바퀴)하며 /imu 를 기록하고, roll/pitch 를
           헤딩의 함수  b + a1*cos(yaw) + a2*sin(yaw) (+ 시간 추세)로 최소제곱 적합해 바이어스 b 를 구함.
           4헤딩 정지 측정보다 점이 훨씬 많고, 회전 직후 안정화 대기가 없으며, 방향별 차이로 회전 속도에
           따른 편차도 확인함. 로봇 /cmd_vel 을 받을 수 있는 머신에서 실행함.
  record   주행 중 /imu 와 /odom 을 기록(Ctrl-C 로 종료하며 저장).
  analyze  record 결과와 profiler 의 frames_*.npz 를 같은 시각축으로 겹쳐서
           라이다 프레임별 평면 적합 tilt 와 IMU tilt 의 상관/지연/가감속 구간별 편차를 출력함.

사용 예:
  python3 check_imu_dynamics.py static --heading 0 --duration 30
  python3 check_imu_dynamics.py static --report
  python3 check_imu_dynamics.py rotate --deg 90
  python3 check_imu_dynamics.py sweep
  python3 check_imu_dynamics.py record --label tf_off
  python3 check_imu_dynamics.py analyze --imu imu_drive_tf_off_<ts>.npz \
      --frames frames_<ts>.npz --map ~/dae_floor_maps/maps/grid/map_from_dae.yaml --imu-tf off

결과 파일은 ~/dae_floor_maps/analytics/imu_check/ 에 저장됨.
"""

import argparse
import glob
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))

OUT_DIR = os.path.expanduser('~/dae_floor_maps/analytics/imu_check')
HEADINGS = (0, 90, 180, 270)


# ---------------------------------------------------------------- 기록 ----

def _record(duration, with_odom, settle=0.0):
    """/imu(+/odom)를 duration초(0이면 Ctrl-C까지) 기록해 dict로 반환함. 앞 settle초는 버림."""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Imu
    from nav_msgs.msg import Odometry
    import tf_transformations

    imu, odom = [], []

    class Rec(Node):
        def __init__(self):
            super().__init__('check_imu_dynamics')
            self.create_subscription(Imu, '/imu', self.on_imu, 100)
            if with_odom:
                self.create_subscription(Odometry, '/odom', self.on_odom, 100)

        def on_imu(self, m):
            q = m.orientation
            if q.x == q.y == q.z == q.w == 0.0:
                return
            r, p, _ = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            a, g = m.linear_acceleration, m.angular_velocity
            imu.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                        np.degrees(r), np.degrees(p), a.x, a.y, a.z, g.x, g.y, g.z))

        def on_odom(self, m):
            odom.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                         m.twist.twist.linear.x, m.twist.twist.angular.z))

    rclpy.init()
    node = Rec()
    t0, last = time.time(), time.time()
    try:
        while rclpy.ok() and (duration <= 0 or time.time() - t0 < duration):
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.time() - last > 5.0:
                last = time.time()
                print(f"  [{last - t0:5.0f}s] imu={len(imu)} odom={len(odom)}", flush=True)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():  # Ctrl-C 시 rclpy 시그널 핸들러가 이미 shutdown 했을 수 있음
        rclpy.shutdown()

    imu, odom = np.array(imu), np.array(odom)
    if imu.size and settle > 0:
        imu = imu[imu[:, 0] >= imu[0, 0] + settle]
    return {'imu': imu, 'odom': odom}


# ---------------------------------------------------------------- static ----

def cmd_static(a):
    path = os.path.join(OUT_DIR, f'static_{a.tag}.json')
    os.makedirs(OUT_DIR, exist_ok=True)
    store = json.load(open(path)) if os.path.exists(path) else {}

    if not a.report:
        if a.heading is None:
            sys.exit('--heading 이 필요함 (0/90/180/270, 첫 자세를 0으로 두고 90도씩 돌린 값)')
        print(f"[*] heading {a.heading}: 로봇을 완전히 정지시킨 상태. {a.settle:.0f}s 버리고 {a.duration:.0f}s 기록")
        d = _record(a.duration + a.settle, with_odom=False, settle=a.settle)['imu']
        if len(d) < 50:
            sys.exit('[-] /imu 샘플 부족 - 토픽/도메인/네트워크 확인')
        rate = (len(d) - 1) / (d[-1, 0] - d[0, 0])
        store[str(a.heading)] = {
            'roll': float(d[:, 1].mean()), 'pitch': float(d[:, 2].mean()),
            'roll_std': float(d[:, 1].std()), 'pitch_std': float(d[:, 2].std()),
            'n': int(len(d)), 'time': time.strftime('%Y-%m-%d %H:%M:%S')}
        json.dump(store, open(path, 'w'), indent=1)
        print(f"    n={len(d)} ({rate:.1f}Hz)  roll {d[:,1].mean():+.3f}±{d[:,1].std():.3f}  "
              f"pitch {d[:,2].mean():+.3f}±{d[:,2].std():.3f} deg")
        # 기록 구간 안의 추세(deg/min). 정지 중에도 값이 흐르면 평균을 바이어스로 믿을 수 없음.
        t = d[:, 0] - d[0, 0]
        dr = [np.polyfit(t, d[:, k], 1)[0] * 60 for k in (1, 2)]
        print(f"    기록 중 드리프트: roll {dr[0]:+.3f}  pitch {dr[1]:+.3f} deg/min  "
              f"(구간 전체 {dr[0] * t[-1] / 60:+.2f} / {dr[1] * t[-1] / 60:+.2f} deg)")
        if max(abs(dr[0]), abs(dr[1])) * t[-1] / 60 > 0.3:
            print("    [!] 기록 중 0.3deg 넘게 흘렀음 - 회전 직후 필터 과도 응답이 덜 끝났거나 IMU 드리프트임. "
                  "대기 시간을 늘려 재측정할 것.")

    print(f"\n[*] 누적 ({path})")
    for h in HEADINGS:
        s = store.get(str(h))
        print(f"    heading {h:3d}: " + (f"roll {s['roll']:+.3f}  pitch {s['pitch']:+.3f}  ({s['time']})"
                                         if s else "(미측정)"))
    miss = [h for h in HEADINGS if str(h) not in store]
    if miss:
        print(f"\n    아직 {miss} 헤딩이 없음 - 4개가 모여야 바이어스 후보를 계산함.")
        return
    r = np.array([store[str(h)]['roll'] for h in HEADINGS])
    p = np.array([store[str(h)]['pitch'] for h in HEADINGS])
    print(f"\n[*] 4헤딩 평균 -> imu_mount_correction_rpy_deg_real: [{r.mean():.3f}, {p.mean():.3f}]")
    print(f"    헤딩 간 표준편차 roll {r.std():.3f}  pitch {p.std():.3f} deg")
    if max(r.std(), p.std()) > 0.5:
        print("    [!] 헤딩 간 편차가 0.5deg 초과 - 몸체 고정 바이어스가 아니라 바닥 기울기/필터 드리프트/"
              "자기장 등이 섞였을 수 있음. 측정 위치를 바꾸거나 시간을 두고 반복해 재현되는지 먼저 확인.")
    print("    (주의) 이 값은 params.yaml 의 imu_mount_correction_rpy_deg_real 에 넣는 값이며, "
          "라이다 보정(lidar_mount_correction_rpy_deg_real)과는 별개임.")


# ---------------------------------------------------------------- rotate ----

def cmd_rotate(a):
    """/odom yaw 변화량이 목표에 닿을 때까지 제자리 회전 후 정지. 남은 각도에 비례해 감속함."""
    import math
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry

    target = math.radians(abs(a.deg))
    sign = 1.0 if a.deg >= 0 else -1.0
    state = {'prev': None, 'turned': 0.0}

    class Rot(Node):
        def __init__(self):
            super().__init__('check_imu_rotate')
            self.pub = self.create_publisher(Twist, a.cmd_topic, 10)
            self.create_subscription(Odometry, a.odom_topic, self.on_odom, 20)

        def on_odom(self, m):
            q = m.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            if state['prev'] is not None:
                d = (yaw - state['prev'] + math.pi) % (2 * math.pi) - math.pi  # -pi~pi 로 감싼 증분
                state['turned'] += sign * d
            state['prev'] = yaw

        def send(self, w):
            t = Twist()
            t.angular.z = w
            self.pub.publish(t)

    rclpy.init()
    node = Rot()
    t0 = time.time()
    while rclpy.ok() and state['prev'] is None and time.time() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if state['prev'] is None:
        node.destroy_node(); rclpy.shutdown()
        sys.exit(f"[-] {a.odom_topic} 수신 없음 - 토픽/도메인 확인")

    print(f"[*] 제자리 회전 {a.deg:+.0f}deg (최대 {a.speed} rad/s, 목표 근처에서 감속) - 앞 공간 확인")
    t0 = time.time()
    try:
        while rclpy.ok() and time.time() - t0 < a.timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            remaining = target - state['turned']
            if remaining <= math.radians(a.tol_deg):
                break
            node.send(sign * min(a.speed, max(a.min_speed, 1.5 * remaining)))
    except KeyboardInterrupt:
        pass
    for _ in range(10):  # 정지 명령을 여러 번 보내 유실 방지
        node.send(0.0)
        rclpy.spin_once(node, timeout_sec=0.05)
    time.sleep(0.5)
    rclpy.spin_once(node, timeout_sec=0.1)
    print(f"[+] 정지. odom 기준 회전량 {math.degrees(state['turned']):+.1f}deg "
          f"(목표 {abs(a.deg):.0f}deg, 정지 후 관성분 포함)")
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


# ---------------------------------------------------------------- sweep ----

def _fit_leg(rows, settle):
    """한 방향 회전 구간 rows[(t, roll, pitch, yaw_unwrapped)] 적합: y = b + a1 cos(yaw) + a2 sin(yaw) + c*t."""
    t = rows[:, 0] - rows[0, 0]
    r = rows[t >= settle]
    t = r[:, 0] - r[0, 0]
    tm = (t - t.mean()) / 60.0  # 분 단위, 중심화(절편 = 구간 중간 시각의 값)
    X = np.c_[np.ones(len(r)), np.cos(r[:, 3]), np.sin(r[:, 3]), tm]
    out = {}
    for name, col in (('roll', 1), ('pitch', 2)):
        coef, *_ = np.linalg.lstsq(X, r[:, col], rcond=None)
        out[name] = dict(bias=coef[0], amp=float(np.hypot(coef[1], coef[2])), drift=coef[3],
                         resid=float((r[:, col] - X @ coef).std()))
    out['turned_deg'] = float(np.degrees(abs(r[-1, 3] - r[0, 3])))
    out['n'] = len(r)
    out['dur_min'] = float(t[-1] / 60.0)
    return out


def cmd_sweep(a):
    import math
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    import tf_transformations

    st = {'prev': None, 'unw': 0.0, 'leg': -1}
    rows = []

    class Sw(Node):
        def __init__(self):
            super().__init__('check_imu_sweep')
            self.pub = self.create_publisher(Twist, a.cmd_topic, 10)
            self.create_subscription(Odometry, a.odom_topic, self.on_odom, 20)
            self.create_subscription(Imu, '/imu', self.on_imu, 50)

        def on_odom(self, m):
            q = m.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            if st['prev'] is not None:
                st['unw'] += (yaw - st['prev'] + math.pi) % (2 * math.pi) - math.pi
            st['prev'] = yaw

        def on_imu(self, m):
            q = m.orientation
            if st['prev'] is None or st['leg'] < 0 or q.x == q.y == q.z == q.w == 0.0:
                return
            r, p, _ = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            rows.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                         math.degrees(r), math.degrees(p), st['unw'], st['leg']))

        def send(self, w):
            t = Twist()
            t.angular.z = w
            self.pub.publish(t)

    rclpy.init()
    node = Sw()
    t0 = time.time()
    while rclpy.ok() and st['prev'] is None and time.time() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if st['prev'] is None:
        node.destroy_node(); rclpy.shutdown()
        sys.exit(f"[-] {a.odom_topic} 수신 없음 - 토픽/도메인 확인")

    tol = math.radians(a.tol_deg)
    print(f"[*] 연속 회전 {a.turns}바퀴(방향 교대, 각 {a.speed} rad/s ≈ {2 * math.pi / a.speed:.0f}s/바퀴). "
          "로봇 주변 반경 0.5m 이상 비워둘 것. 중단은 Ctrl-C")
    try:
        for leg in range(a.turns):
            sign = 1.0 if leg % 2 == 0 else -1.0
            start, t_leg = st['unw'], time.time()
            st['leg'] = leg
            print(f"  [leg {leg}] {'반시계' if sign > 0 else '시계'} 회전 중...", flush=True)
            while rclpy.ok() and time.time() - t_leg < 2 * math.pi / a.speed * 2 + 10:
                rclpy.spin_once(node, timeout_sec=0.05)
                remaining = 2 * math.pi - sign * (st['unw'] - start)
                if remaining <= tol:
                    break
                ramp = max(0.2, min(1.0, (time.time() - t_leg) / 3.0))  # 시작 가속 완화
                node.send(sign * min(a.speed * ramp, max(a.min_speed, 1.5 * remaining)))
            st['leg'] = -1
            for _ in range(10):
                node.send(0.0)
                rclpy.spin_once(node, timeout_sec=0.05)
            t_w = time.time()
            while time.time() - t_w < a.pause:  # 방향 전환 전 정지
                rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    for _ in range(10):
        node.send(0.0)
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    rows = np.array(rows)
    if rows.size == 0:
        sys.exit('[-] 기록된 /imu 없음')
    os.makedirs(OUT_DIR, exist_ok=True)
    out = os.path.join(OUT_DIR, f"imu_sweep_{time.strftime('%Y-%m-%d_%H-%M-%S')}.npz")
    np.savez(out, rows=rows)
    print(f"[+] 원시 기록 저장: {out}")

    fits = []
    for leg in sorted(set(rows[:, 4].astype(int))):
        lr = rows[rows[:, 4] == leg][:, :4]
        if len(lr) < 200 or abs(lr[-1, 3] - lr[0, 3]) < math.radians(300):
            print(f"  [leg {leg}] 회전이 360deg 미만이거나 샘플 부족 - 제외")
            continue
        f = _fit_leg(lr, a.settle)
        fits.append(f)
        print(f"\n[leg {leg}] {'반시계' if leg % 2 == 0 else '시계'}  회전 {f['turned_deg']:.0f}deg  n={f['n']}")
        for ax in ('roll', 'pitch'):
            g = f[ax]
            print(f"    {ax:5s} 바이어스 {g['bias']:+.3f}  헤딩 의존 진폭 {g['amp']:.3f}  "
                  f"잔차 std {g['resid']:.3f}  드리프트 {g['drift']:+.3f} deg/min")
    if not fits:
        sys.exit('[-] 유효한 회전 구간 없음')

    b = {ax: float(np.mean([f[ax]['bias'] for f in fits])) for ax in ('roll', 'pitch')}
    print(f"\n[*] 평균 -> imu_mount_correction_rpy_deg_real: [{b['roll']:.3f}, {b['pitch']:.3f}]")
    if len(fits) >= 2:
        dr = fits[0]['roll']['bias'] - fits[1]['roll']['bias']
        dp = fits[0]['pitch']['bias'] - fits[1]['pitch']['bias']
        print(f"    반시계-시계 바이어스 차 roll {dr:+.3f}  pitch {dp:+.3f} deg")
        if max(abs(dr), abs(dp)) > 0.2:
            print("    [!] 방향에 따라 바이어스가 0.2deg 넘게 달라짐 - 회전 속도/방향에 의존하는 오차(필터 지연 등)가 있음. "
                  "--speed 를 낮춰 재측정하고, 그래도 남으면 이 IMU 는 회전 중 값을 믿기 어려움.")
    amps = [(f['roll']['amp'], f['pitch']['amp']) for f in fits]
    print("    헤딩 의존 진폭은 바닥 기울기(강체 기울기면 roll/pitch 진폭이 비슷함)와 헤딩 오차가 섞인 값임: "
          + ", ".join(f"({r:.2f}, {p:.2f})" for r, p in amps) + " deg")
    if any(abs(f[ax]['drift']) * f['dur_min'] > 0.3 for f in fits for ax in ('roll', 'pitch')):
        print("    [!] 한 구간 안에서 0.3deg 넘게 흘렀음 - 정지 상태에서도 흐르는 IMU 일 수 있음(static --duration 300 으로 확인).")

# ---------------------------------------------------------------- record ----

def cmd_record(a):
    os.makedirs(OUT_DIR, exist_ok=True)
    print("[*] /imu + /odom 기록 중 - 주행이 끝나면 Ctrl-C")
    d = _record(0, with_odom=True)
    if len(d['imu']) < 50:
        sys.exit('[-] /imu 샘플 부족')
    out = os.path.join(OUT_DIR, f"imu_drive_{a.label}_{time.strftime('%Y-%m-%d_%H-%M-%S')}.npz")
    np.savez(out, imu=d['imu'], odom=d['odom'])
    print(f"[+] 저장: {out}  (imu {len(d['imu'])}, odom {len(d['odom'])})")


# ---------------------------------------------------------------- analyze ----

def _frame_tilts(log, map_path, wall_margin, z_band, r_min, r_max, min_points):
    """프레임별 바닥 평면 z=a*전방+b*좌측+c 적합 -> (stamp, pitch_deg, roll_deg). analyze_z_bias.py 와 같은 정의."""
    import cv2
    from utils.frame_recorder import used_frame_mask
    from utils.heatmap_generator import _load_occupancy_map
    from utils.frame_video import _wall_mask

    off, poses = log['offsets'], log['poses']
    wall_dist = None
    if map_path:
        img, res, ox, oy = _load_occupancy_map(map_path)
        walls = _wall_mask(img)
        dist = cv2.distanceTransform((~walls).astype(np.uint8), cv2.DIST_L2, 5) * res

        def wall_dist(p):
            cols = np.clip(((p[:, 0] - ox) / res).astype(int), 0, walls.shape[1] - 1)
            rows = np.clip(((p[:, 1] - oy) / res).astype(int), 0, walls.shape[0] - 1)
            return dist[rows, cols]

    rows = []
    for i in np.nonzero(used_frame_mask(log) & (off[1:] > off[:-1]))[0]:
        p = log['points'][off[i]:off[i + 1]]
        p = p[np.abs(p[:, 2]) <= z_band]
        if wall_dist is not None:
            p = p[wall_dist(p) > wall_margin]
        dx, dy = p[:, 0] - poses[i, 0], p[:, 1] - poses[i, 1]
        c, s = np.cos(poses[i, 3]), np.sin(poses[i, 3])
        f, l = dx * c + dy * s, -dx * s + dy * c
        k = (np.hypot(f, l) >= r_min) & (np.hypot(f, l) <= r_max)
        if k.sum() < min_points:
            continue
        A = np.c_[f[k], l[k], np.ones(k.sum())]
        coef, *_ = np.linalg.lstsq(A, p[k, 2], rcond=None)
        rows.append((log['stamps'][i], np.degrees(np.arctan(coef[0])), np.degrees(np.arctan(coef[1]))))
    return np.array(rows)


def _speed_accel(odom, win_sec=0.5):
    """odom 에서 속력 s=|v|, 속력 변화율 ds/dt(m/s^2)를 구함. 반환: (t, s, ds)."""
    t, s = odom[:, 0], np.abs(odom[:, 1])
    n = max(1, int(round(win_sec * (len(t) - 1) / (t[-1] - t[0]))))
    ss = np.convolve(s, np.ones(n) / n, mode='same')
    return t, ss, np.gradient(ss, t)


def _phase(s, ds, thr):
    ph = np.full(len(s), 'other', dtype=object)
    ph[(s < 0.01) & (np.abs(ds) < thr)] = 'stop'
    ph[(s >= 0.05) & (np.abs(ds) < thr / 2)] = 'cruise'
    ph[(s >= 0.01) & (ds > thr)] = 'accel'
    ph[(s >= 0.01) & (ds < -thr)] = 'brake'
    return ph


def cmd_analyze(a):
    from utils.frame_recorder import load_frame_log

    z = np.load(a.imu)
    imu, odom = z['imu'], z['odom']
    ti, I = imu[:, 0], {'pitch': imu[:, 2], 'roll': imu[:, 1]}

    print(f"[*] 프레임 로드/평면 적합 중: {a.frames}")
    F = _frame_tilts(load_frame_log(a.frames), a.map, a.wall_margin, a.z_band,
                     a.r_min, a.r_max, a.min_points)
    if len(F) < 30:
        sys.exit('[-] 적합된 프레임이 30개 미만 - 캡처 구간(start_waypoint_capture)이 열렸었는지 확인')
    tf_, L = F[:, 0], {'pitch': F[:, 1], 'roll': F[:, 2]}

    keep = (tf_ >= ti[0] + 1.0) & (tf_ <= ti[-1] - 1.0)
    if keep.sum() < 30:
        sys.exit(f"[-] IMU 시간 범위({ti[0]:.1f}~{ti[-1]:.1f})와 프레임 시각({tf_[0]:.1f}~{tf_[-1]:.1f})이 "
                 "안 겹침 - 라이다 노트북과 Jetson 시계가 크게 어긋났거나 sim time/wall time 혼용.")
    tf_ = tf_[keep]
    L = {k: v[keep] for k, v in L.items()}

    # 가감속 구간 분류(IMU 시각 기준, 프레임 시각 기준 둘 다)
    to, so, dso = _speed_accel(odom)
    ph_i = _phase(np.interp(ti, to, so), np.interp(ti, to, dso), a.acc_thr)
    ph_f = _phase(np.interp(tf_, to, so), np.interp(tf_, to, dso), a.acc_thr)

    print(f"\n[*] 프레임 {len(tf_)}개, IMU {len(ti)}개. 구간별 개수(IMU/프레임):")
    for n in ('stop', 'accel', 'cruise', 'brake'):
        print(f"    {n:7s} {np.sum(ph_i == n):5d} / {np.sum(ph_f == n):4d}")

    res = {}
    for ax in ('pitch', 'roll'):
        print(f"\n===== {ax} =====")
        lags = np.arange(-1.0, 1.0001, 0.05)
        best = None
        for lag in lags:
            x = np.interp(tf_ + lag, ti, I[ax])
            c = np.corrcoef(L[ax] - L[ax].mean(), x - x.mean())[0, 1]
            if best is None or abs(c) > abs(best[1]):
                best = (lag, c)
        x0 = np.interp(tf_, ti, I[ax])
        c0 = np.corrcoef(L[ax] - L[ax].mean(), x0 - x0.mean())[0, 1]
        xb = np.interp(tf_ + best[0], ti, I[ax])
        slope = np.polyfit(xb, L[ax], 1)[0]
        stop_f = ph_f == 'stop'
        noise = L[ax][stop_f].std() if stop_f.sum() > 10 else float('nan')
        print(f"  상관(지연 0): {c0:+.2f}   최적 지연 {best[0]:+.2f}s 에서 {best[1]:+.2f}   "
              f"기울기 L~I: {slope:+.2f}")
        print(f"  IMU 변동 std {I[ax].std():.3f} deg   라이다 적합 std {L[ax].std():.3f} deg   "
              f"정지 프레임의 라이다 적합 잡음 std {noise:.3f} deg")
        print("  구간별 평균 변화량(정지 구간 평균 대비, deg):   IMU / 라이다")
        bi = I[ax][ph_i == 'stop'].mean() if np.any(ph_i == 'stop') else np.nan
        bl = L[ax][stop_f].mean() if stop_f.any() else np.nan
        for n in ('accel', 'cruise', 'brake'):
            mi, mf = ph_i == n, ph_f == n
            si = f"{I[ax][mi].mean() - bi:+.3f}" if mi.sum() > 5 else "  n/a "
            sl = f"{L[ax][mf].mean() - bl:+.3f}" if mf.sum() > 3 else "  n/a "
            print(f"    {n:7s} {si} / {sl}")
        res[ax] = dict(c0=c0, best_c=best[1], lag=best[0], slope=slope, noise=noise, imu_std=I[ax].std(), L_std=L[ax].std())

    print("\n===== 해석 =====")
    for ax, r in res.items():
        if a.imu_tf == 'off':
            if max(abs(r['c0']), abs(r['best_c'])) > 0.6 and 0.7 < abs(r['slope']) < 1.3:
                v = "IMU 가 실제 차체 기울기를 따라감(상관 높음, |기울기|≈1) -> 주입해도 됨"
            elif r['imu_std'] < r['noise']:
                v = "판별 불가: IMU 변동이 라이다 적합 잡음보다 작고 상관도 낮음(동적 틸트가 검출 한계 아래)"
            elif abs(r['c0']) < 0.3:
                v = "IMU 변동이 라이다에 안 보임 -> IMU 아티팩트(가감속 영향)이거나 라이다가 못 잡는 크기"
            else:
                v = "부분 일치 - 구간별 표(가속/제동에서만 어긋나는지)를 볼 것"
        else:
            if abs(r['slope']) < 0.3:
                v = "잔차가 IMU 와 무상관 -> 보상이 동작함"
            elif abs(r['slope']) > 1.5:
                v = "|기울기|≈2 근처 -> 부호 반대/과보상 의심(IMU TF 를 켠 쪽이 더 나쁨)"
            else:
                v = "보상이 부분적 - TF off 주행과 L std 를 비교할 것"
        print(f"  {ax}: {v}")
    if abs(res['pitch']['lag']) > 0.3:
        print(f"  [!] 최적 지연 {res['pitch']['lag']:+.2f}s (양수=IMU가 라이다보다 늦음) - IMU 지연이거나 "
              "두 머신 시계 오프셋일 수 있음(분리 불가).")

    _plot(a, ti, I, tf_, L, to, so)


def _plot(a, ti, I, tf_, L, to, so):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    t0 = ti[0]
    fig, axs = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    axs[0].plot(to - t0, so, 'k')
    axs[0].set_ylabel('speed [m/s]')
    for ax, name in zip(axs[1:], ('pitch', 'roll')):
        ax.plot(ti - t0, I[name] - np.mean(I[name]), lw=0.8, label=f'IMU {name} (mean removed)')
        ax.plot(tf_ - t0, L[name] - np.mean(L[name]), '.', ms=3, label=f'lidar-fit {name} (mean removed)')
        ax.set_ylabel('[deg]')
        ax.legend(loc='upper right')
        ax.grid(alpha=.3)
    axs[2].set_xlabel('time [s]')
    axs[0].set_title(f'IMU vs lidar floor tilt (imu_tf={a.imu_tf})')
    out = os.path.join(OUT_DIR, os.path.basename(a.imu).replace('.npz', '_vs_lidar.png'))
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"\n[+] 그래프: {out}")


def _resolve(p, pattern_dir):
    return p if os.path.exists(p) else (glob.glob(os.path.join(pattern_dir, p)) or [p])[0]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)

    s = sub.add_parser('static', help='정지 상태 헤딩별 roll/pitch 기록')
    s.add_argument('--heading', type=int, choices=HEADINGS)
    s.add_argument('--duration', type=float, default=30.0)
    s.add_argument('--settle', type=float, default=10.0, help='시작 후 버릴 시간(s). 회전 직후 필터 안정화용')
    s.add_argument('--tag', default='default', help='측정 세션 이름(다른 날/다른 바닥이면 바꿈)')
    s.add_argument('--report', action='store_true', help='기록 없이 누적 결과만 출력')
    s.set_defaults(fn=cmd_static)

    ro = sub.add_parser('rotate', help='odom yaw를 보며 제자리 회전 후 자동 정지(헤딩 전환용)')
    ro.add_argument('--deg', type=float, default=90.0, help='회전각(deg). 양수=반시계, 음수=시계')
    ro.add_argument('--speed', type=float, default=0.3, help='최대 각속도(rad/s)')
    ro.add_argument('--min-speed', type=float, default=0.12, help='감속 구간 최소 각속도(rad/s)')
    ro.add_argument('--tol-deg', type=float, default=1.5, help='이 각도 이내로 남으면 정지 명령')
    ro.add_argument('--timeout', type=float, default=20.0)
    ro.add_argument('--cmd-topic', default='/cmd_vel')
    ro.add_argument('--odom-topic', default='/odom')
    ro.set_defaults(fn=cmd_rotate)

    sw = sub.add_parser('sweep', help='연속 회전하며 헤딩 함수로 적합해 바이어스 산출')
    sw.add_argument('--turns', type=int, default=2, help='바퀴 수(방향 교대, 짝수 권장)')
    sw.add_argument('--speed', type=float, default=0.1, help='각속도(rad/s). 낮을수록 필터 지연 영향이 작음')
    sw.add_argument('--min-speed', type=float, default=0.05)
    sw.add_argument('--tol-deg', type=float, default=2.0)
    sw.add_argument('--settle', type=float, default=3.0, help='각 구간 시작 후 적합에서 버릴 시간(s)')
    sw.add_argument('--pause', type=float, default=5.0, help='방향 전환 전 정지 시간(s)')
    sw.add_argument('--cmd-topic', default='/cmd_vel')
    sw.add_argument('--odom-topic', default='/odom')
    sw.set_defaults(fn=cmd_sweep)

    r = sub.add_parser('record', help='주행 중 /imu,/odom 기록(Ctrl-C로 종료)')
    r.add_argument('--label', default='run', help='예: tf_off / tf_on')
    r.set_defaults(fn=cmd_record)

    an = sub.add_parser('analyze', help='IMU 기록과 frames_*.npz 비교')
    an.add_argument('--imu', required=True)
    an.add_argument('--frames', required=True)
    an.add_argument('--map', default=None, help='있으면 벽 근처 점 제외(권장)')
    an.add_argument('--imu-tf', choices=('on', 'off'), required=True,
                    help='frames 수집 때 imu_tilt_broadcaster 가 실제 틸트를 넣고 있었는지')
    an.add_argument('--acc-thr', type=float, default=0.03, help='가/감속 판정 임계(m/s^2)')
    an.add_argument('--wall-margin', type=float, default=0.4)
    an.add_argument('--z-band', type=float, default=0.1)
    an.add_argument('--r-min', type=float, default=1.2)
    an.add_argument('--r-max', type=float, default=3.5)
    an.add_argument('--min-points', type=int, default=300)
    an.set_defaults(fn=cmd_analyze)

    a = ap.parse_args()
    if a.cmd == 'analyze':
        a.imu = _resolve(os.path.expanduser(a.imu), OUT_DIR)
        a.frames = _resolve(os.path.expanduser(a.frames),
                            os.path.expanduser('~/dae_floor_maps/analytics/pointclouds/frames'))
        a.map = os.path.expanduser(a.map) if a.map else None
        sys.path.insert(0, os.path.join(HERE, '..'))
    a.fn(a)


if __name__ == '__main__':
    main()
