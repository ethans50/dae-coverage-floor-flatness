#!/usr/bin/env python3
# scripts/stack.py
"""로봇(Jetson)과 라이다 노트북에 걸친 실행 절차를 노트북 한 곳에서 순서대로 올리고, 단계마다 게이트로 확인함.

각 단계는 "시작 -> 게이트 통과(DDS로 노트북에서 직접 확인) -> 다음 단계" 순서이며, 게이트에 실패하면
그 단계의 로그 끝부분을 보여주고 멈춤. 사람이 터미널 로그를 보고 판단하던 항목을 수치 조건으로 바꾼 것임.

  단계: preflight -> bringup(Jetson) -> nav2(Jetson) -> velodyne(노트북) -> pose -> profiler(노트북)

사용:
  stack.py up [--imu-tf on|off] [--init X Y YAW] [--step] [--clean] [--until 단계]
  stack.py down      양쪽의 모든 프로세스를 scripts/stop_all.sh 로 종료함
  stack.py check     실행 중인 스택의 상태 점검(AMCL 초기 위치가 안 잡힐 때의 진단 포함)
                     [--try-pose X Y YAW]: map 프레임으로 /initialpose 를 발행해 map->odom 이 생기는지 시험함

Jetson 접속 정보는 auto_calibration_drive.py 와 같은 규칙임(ROBOT_HOST, SSH_PASSWORD 환경변수, 사용자 waffle).
Jetson 프로세스는 `bash -ic`(~/.bashrc 의 ROS_DOMAIN_ID, RMW 등 적용)로 띄우고 로그를 ~/stack_logs/ 에 남김.
이 스크립트는 ROS 환경을 source 한 터미널에서 실행해야 함(게이트가 rclpy 로 확인하므로).
"""

import argparse
import collections
import json
import math
import os
import shlex
import subprocess
import sys
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from std_msgs.msg import String

PKG = 'dae-coverage-floor-flatness'
STAGES = ['preflight', 'bringup', 'nav2', 'velodyne', 'pose', 'profiler']
LOG_DIR = '~/stack_logs'
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


class Abort(Exception):
    pass


# ----------------------------------------------------------------------------- 실행 위치(로컬/SSH)

class Runner:
    """셸 문자열을 실행하는 위치. 로컬이면 subprocess, 원격이면 paramiko 를 씀."""

    def __init__(self, name, ws, repo, ssh=None):
        self.name, self.ws, self.repo, self.ssh = name, ws, repo, ssh

    def sh(self, cmd, timeout=60):
        if self.ssh is None:
            r = subprocess.run(['bash', '-c', cmd], capture_output=True, text=True, timeout=timeout)
            return r.returncode, r.stdout + r.stderr
        _, out, err = self.ssh.exec_command(f'bash -c {shlex.quote(cmd)}', timeout=timeout)
        o = out.read().decode(errors='replace') + err.read().decode(errors='replace')
        return out.channel.recv_exit_status(), o

    def start(self, tag, cmd):
        """cmd 를 ROS 환경을 source 한 대화형 셸에서 세션 분리로 실행함(이 스크립트가 끝나도 유지)."""
        inner = f'source {self.ws}/install/setup.bash && {cmd}'
        line = (f'mkdir -p {LOG_DIR}; setsid nohup bash -ic {shlex.quote(inner)} '
                f'> {LOG_DIR}/{tag}.log 2>&1 < /dev/null & echo started')
        code, out = self.sh(line)
        if code != 0 or 'started' not in out:
            raise Abort(f'{self.name}: {tag} 시작 실패: {out.strip()}')

    def tail(self, tag, n=15):
        return self.sh(f"grep -v -e 'no job control' -e 'cannot set terminal process group' {LOG_DIR}/{tag}.log | tail -n {n}")[1]

    def stop_all(self, flag=''):
        script = f'{self.repo}/scripts/stop_all.sh'
        return self.sh(f'test -f {script} && bash {script} {flag} || echo "NO_SCRIPT {script}"', timeout=60)[1]


# ----------------------------------------------------------------------------- DDS 게이트

class Probe:
    """노트북의 rclpy 노드 하나로 토픽 수신율, TF, 라이프사이클, 래치 토픽을 확인함."""

    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('stack_probe')
        self.times = collections.defaultdict(lambda: collections.deque(maxlen=400))
        self.latched = {}
        self.buf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.buf, self.node)
        for topic, typ in [('/scan', LaserScan), ('/imu', Imu), ('/odom', Odometry), ('/velodyne_points', PointCloud2)]:
            self.node.create_subscription(typ, topic, lambda m, t=topic: self.times[t].append(time.monotonic()),
                                          qos_profile_sensor_data)
        self.node.create_subscription(String, '/imu_tilt/status', lambda m: self.latched.__setitem__('imu', m.data), LATCHED)
        self.node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                      lambda m: self.latched.__setitem__('amcl_pose', m), LATCHED)
        self.pose_pub = self.node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    def spin(self, sec=0.3):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def rate(self, topic, window=3.0):
        now = time.monotonic()
        return sum(1 for t in self.times[topic] if now - t <= window) / window

    def tf_ok(self, parent, child):
        return self.buf.can_transform(parent, child, Time(), Duration(seconds=0.0))

    def lifecycle(self, node_name):
        cli = self.node.create_client(GetState, f'{node_name}/get_state')
        if not cli.wait_for_service(timeout_sec=1.0):
            return 'no-service'
        fut = cli.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=2.0)
        return fut.result().current_state.label if fut.done() and fut.result() else 'no-reply'

    def publishers(self, topic):
        return len(self.node.get_publishers_info_by_topic(topic))

    def imu_status(self):
        return json.loads(self.latched['imu']) if 'imu' in self.latched else None

    def publish_initial_pose(self, x, y, yaw):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'  # AMCL 은 map 프레임 값만 받음(RViz Fixed Frame 이 map 이 아니면 다른 프레임으로 나감)
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        m.pose.covariance[0] = m.pose.covariance[7] = 0.25
        m.pose.covariance[35] = 0.07
        self.pose_pub.publish(m)


def ros_param(name, param):
    r = subprocess.run(['ros2', 'param', 'get', name, param], capture_output=True, text=True, timeout=10)
    return (r.stdout.strip().split(': ', 1)[-1]) if r.returncode == 0 else 'N/A'


def gate(probe, label, fn, timeout, log=None):
    """fn() -> (통과 여부, 상세) 를 1초 간격으로 timeout 까지 재시도함. 실패하면 Abort."""
    t0, detail = time.monotonic(), ''
    while True:
        probe.spin(1.0)
        try:
            ok, detail = fn()
        except Exception as e:  # 노드가 아직 없을 때의 일시적 오류는 재시도
            ok, detail = False, f'{type(e).__name__}: {e}'
        if ok:
            print(f'    [PASS] {label}: {detail}')
            return
        if time.monotonic() - t0 > timeout:
            print(f'    [FAIL] {label}: {detail}')
            if log:
                print(f'    --- {log[0].name}/{log[1]} 로그 끝부분 ---\n{log[0].tail(log[1])}')
            raise Abort(f'게이트 실패: {label}')


def rate_gate(probe, topic, lo, hi, log, timeout=20):
    gate(probe, f'{topic} 수신율 {lo}-{hi} Hz', lambda: ((lo <= (r := probe.rate(topic)) <= hi), f'{r:.1f} Hz'),
         timeout, log)


# ----------------------------------------------------------------------------- 단계

def build_cmds(a):
    nav2 = f'ros2 launch {PKG} tb3_waffle_nav2.launch.py use_sim_time:=false use_rviz:=false'
    if a.map:
        nav2 += f' map:={a.map}'
    cmds = {
        'bringup': f'ros2 launch {PKG} real_bringup.launch.py use_imu_tilt:={"true" if a.imu_tf == "on" else "false"}',
        'nav2': nav2,
        'velodyne': 'ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py',
        'profiler': f'ros2 launch {PKG} surface_profiling.launch.py is_sim:=false',
    }
    for kv in a.cmd or []:
        k, v = kv.split('=', 1)
        cmds[k] = v
    return cmds


def has_procs(runner):
    out = runner.stop_all('-l')
    if 'NO_SCRIPT' in out:
        raise Abort(f'{runner.name}: {out.strip()} (git pull 후 재시도)')
    return '(없음)' not in out, out


def run_up(a, jet, lap, probe):
    cmds = build_cmds(a)
    want = STAGES[:STAGES.index(a.until) + 1]

    def stage(name, fn):
        if name not in want:
            return
        print(f'\n[{name}]')
        fn()
        if a.step and name != want[-1]:
            input('    확인했으면 Enter, 중단은 Ctrl-C: ')

    def preflight():
        for r in (jet, lap):
            busy, out = has_procs(r)
            if busy and not a.clean:
                raise Abort(f'{r.name}: 이미 실행 중인 프로세스가 있음(중복 방지). --clean 으로 정리하거나 down 먼저 실행\n{out}')
            if busy:
                r.stop_all()
                print(f'    {r.name}: 기존 프로세스 정리함')
        print('    [PASS] 양쪽 모두 깨끗함')

    def bringup():
        jet.start('bringup', cmds['bringup'])
        log = (jet, 'bringup')
        rate_gate(probe, '/scan', 3, 20, log)
        rate_gate(probe, '/imu', 10, 500, log)
        rate_gate(probe, '/odom', 10, 100, log)
        gate(probe, '/scan /imu /odom 발행자 각 1개(중복 드라이버 없음)', lambda: (
            (n := [probe.publishers(t) for t in ('/scan', '/imu', '/odom')]) == [1, 1, 1], str(n)), 5, log)
        gate(probe, 'TF odom->base_footprint', lambda: (probe.tf_ok('odom', 'base_footprint'), ''), 15, log)
        gate(probe, 'TF base_footprint->base_link', lambda: (probe.tf_ok('base_footprint', 'base_link'), ''), 15, log)
        want_mode = a.imu_tf
        gate(probe, f'IMU TF 모드 == {want_mode}', lambda: (
            (s := probe.imu_status()) is not None and s['mode'] == want_mode,
            f"{s['mode']} bias={s['bias_rpy_deg']} config_mtime={s['config_mtime']}" if s else '/imu_tilt/status 없음'),
            15, log)

    def nav2():
        jet.start('nav2', cmds['nav2'])
        log = (jet, 'nav2')
        for n in ('/map_server', '/amcl'):
            gate(probe, f'{n} lifecycle active', lambda n=n: ((l := probe.lifecycle(n)) == 'active', l), 60, log)

    def velodyne():
        lap.start('velodyne', cmds['velodyne'])
        log = (lap, 'velodyne')
        rate_gate(probe, '/velodyne_points', 8, 12, log)
        gate(probe, '/velodyne_points 발행자 1개(중복 드라이버 없음)',
             lambda: ((n := probe.publishers('/velodyne_points')) == 1, f'{n}개'), 5, log)

    def pose():
        if a.init:
            x, y, yaw = a.init
            print(f'    /initialpose 발행: map 프레임 x={x} y={y} yaw={yaw}')

            def try_pose():
                probe.publish_initial_pose(x, y, yaw)
                probe.spin(0.5)
                return probe.tf_ok('map', 'odom'), 'map->odom 생성됨' if probe.tf_ok('map', 'odom') else 'map->odom 아직 없음'
            gate(probe, 'map->odom (초기 위치 반영)', try_pose, 20, (jet, 'nav2'))
        else:
            print('    RViz 에서 Fixed Frame 을 map 으로 두고 2D Pose Estimate 로 초기 위치를 지정할 것(대기 중...)')
            gate(probe, 'map->odom (수동 초기 위치)', lambda: (probe.tf_ok('map', 'odom'), ''), a.pose_wait, (jet, 'nav2'))
        gate(probe, '/amcl_pose 수신', lambda: ('amcl_pose' in probe.latched, ''), 10, (jet, 'nav2'))

    def profiler():
        lap.start('profiler', cmds['profiler'])
        gate(probe, 'TF map->velodyne_link', lambda: (probe.tf_ok('map', 'velodyne_link'), ''), 30, (lap, 'profiler'))

    for name, fn in [('preflight', preflight), ('bringup', bringup), ('nav2', nav2), ('velodyne', velodyne),
                     ('pose', pose), ('profiler', profiler)]:
        stage(name, fn)
    print(f'\n[+] 완료({want[-1]}까지). 로그: Jetson/노트북 각 {LOG_DIR}/  종료: stack.py down')


def run_check(a, jet, lap, probe):
    probe.spin(4.0)
    print('[토픽 수신율]')
    for t in ('/scan', '/imu', '/odom', '/velodyne_points'):
        print(f'    {t}: {probe.rate(t, 4.0):.1f} Hz  발행자 {probe.publishers(t)}개')
    print('[TF]')
    for p, c in [('odom', 'base_footprint'), ('base_footprint', 'base_link'), ('odom', 'base_scan'),
                 ('map', 'odom'), ('map', 'velodyne_link')]:
        print(f'    {p}->{c}: {"OK" if probe.tf_ok(p, c) else "없음"}')
    s = probe.imu_status()
    print(f'[IMU TF] {"mode=%s bias=%s (%s)" % (s["mode"], s["bias_rpy_deg"], s["bias_key"]) if s else "/imu_tilt/status 없음"}')
    print('[Nav2]')
    for n in ('/map_server', '/amcl'):
        print(f'    {n}: {probe.lifecycle(n)}')
    for p in ('set_initial_pose', 'always_reset_initial_pose', 'initial_pose.x', 'initial_pose.y', 'initial_pose.yaw'):
        print(f'    amcl {p} = {ros_param("/amcl", p)}')
    print(f'    /amcl_pose 수신: {"예" if "amcl_pose" in probe.latched else "아니오"}')
    print('[AMCL 로그 힌트(Jetson ~/stack_logs/nav2.log, stack.py up 으로 띄운 경우만)]')
    print('    ' + jet.sh(f'grep -E "Setting pose|initial pose|Failed to transform|Message Filter dropping" '
                          f'{LOG_DIR}/nav2.log 2>&1 | tail -n 6')[1].strip().replace('\n', '\n    '))
    if a.try_pose:
        x, y, yaw = a.try_pose
        print(f'[시험] /initialpose(map) x={x} y={y} yaw={yaw} 발행')
        for _ in range(10):
            probe.publish_initial_pose(x, y, yaw)
            probe.spin(1.0)
            if probe.tf_ok('map', 'odom'):
                break
        print(f'    map->odom: {"OK" if probe.tf_ok("map", "odom") else "여전히 없음"}  '
              f'/amcl_pose 수신: {"예" if "amcl_pose" in probe.latched else "아니오"}')


# ----------------------------------------------------------------------------- 진입점

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('cmd_name', choices=['up', 'down', 'check'])
    ap.add_argument('--host', default=os.environ.get('ROBOT_HOST'))
    ap.add_argument('--user', default='waffle')
    ap.add_argument('--password', default=os.environ.get('SSH_PASSWORD'))
    ap.add_argument('--ws', default='~/ros2_ws', help='워크스페이스 경로(양쪽 동일)')
    ap.add_argument('--repo', help=f'저장소 경로(양쪽 동일, 기본 <ws>/src/{PKG}). 클론 폴더 이름이 다르면 지정')
    ap.add_argument('--local', action='store_true', help='Jetson 쪽 명령도 이 컴퓨터에서 실행(점검/시험용)')
    ap.add_argument('--imu-tf', choices=['on', 'off'], default='off')
    ap.add_argument('--init', nargs=3, type=float, metavar=('X', 'Y', 'YAW'))
    ap.add_argument('--try-pose', nargs=3, type=float, metavar=('X', 'Y', 'YAW'))
    ap.add_argument('--map', help='Nav2 에 넘길 지도 yaml 경로')
    ap.add_argument('--until', choices=STAGES, default='profiler')
    ap.add_argument('--step', action='store_true', help='단계마다 Enter 로 확인 후 진행')
    ap.add_argument('--clean', action='store_true', help='실행 중인 프로세스가 있으면 먼저 종료')
    ap.add_argument('--pose-wait', type=int, default=180, help='수동 초기 위치 대기 시간(초)')
    ap.add_argument('--cmd', action='append', metavar='단계=명령', help='단계별 실행 명령 덮어쓰기(시험용)')
    a = ap.parse_args()

    ssh = None
    if not a.local:
        if not (a.host and a.password):
            sys.exit('ROBOT_HOST 와 SSH_PASSWORD 환경변수(또는 --host/--password)가 필요함. 이 컴퓨터에서만 시험하려면 --local')
        import paramiko
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(a.host, username=a.user, password=a.password, timeout=10)
    repo = a.repo or f'{a.ws}/src/{PKG}'
    jet, lap = Runner('Jetson', a.ws, repo, ssh), Runner('Laptop', a.ws, repo)

    if a.cmd_name == 'down':
        for r in (jet, lap):
            print(f'[{r.name}]\n{r.stop_all()}')
        return
    probe = Probe()
    try:
        {'up': run_up, 'check': run_check}[a.cmd_name](a, jet, lap, probe)
    except Abort as e:
        print(f'\n[!] 중단: {e}\n    진행 상황 점검: stack.py check   정리: stack.py down')
        sys.exit(1)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
