# surface_profiling/utils/ssh_robot.py
"""Jetson에 SSH로 접속해 cmd_vel 발행 등 원격 명령을 실행하는 얇은 래퍼.
auto_calibration_drive.py가 노트북에서 로봇 터미널(SSH)을 대신 제어해
전진/후진/정지/회전을 사람 개입 없이 순서대로 실행하기 위해 씀.

paramiko의 exec_command는 호출마다 새 비로그인 쉘을 열므로, 매번
ROS 환경을 다시 소싱함(터미널에서 매번 sb를 치는 것과 동일한 이유)."""

import shlex

import paramiko


class JetsonSession:
    def __init__(self, host, user, password, ros_setup='~/ros2_ws/install/setup.bash'):
        self.password = password
        self.ros_setup = ros_setup
        self.client = paramiko.SSHClient()
        self.client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.client.connect(host, username=user, password=password, timeout=10)

    def close(self):
        self.client.close()

    def run(self, cmd, timeout=30, sudo=False):
        """cmd를 ROS 환경 소싱 후 실행함. sudo=True면 비밀번호를 stdin으로 흘려줌."""
        inner = f"sudo -S {cmd}" if sudo else cmd
        full = f"source {self.ros_setup} && {inner}"
        stdin, stdout, stderr = self.client.exec_command(f"bash -lc {shlex.quote(full)}", timeout=timeout)
        if sudo:
            stdin.write(self.password + '\n')
            stdin.flush()
        out = stdout.read().decode(errors='replace')
        err = stderr.read().decode(errors='replace')
        code = stdout.channel.recv_exit_status()
        return code, out, err

    def sync_clock(self):
        """'chr' 별칭(sudo chronyc -a makestep)과 동일한 동작 - TF 시간 동기화용."""
        code, out, err = self.run('chronyc -a makestep', timeout=15, sudo=True)
        if code != 0:
            print(f"[!] chronyc makestep failed (code={code}): {err.strip()}")
        return code == 0

    def publish_twist(self, linear_x=0.0, angular_z=0.0, duration_s=1.0, rate=20, timeout_margin=10):
        """duration_s 동안 (linear_x, angular_z)를 rate[Hz]로 발행하고 끝날 때까지 블로킹함.
        ros2 topic pub --times는 N개 발행 후 스스로 종료하므로 이 호출도 그만큼 걸림."""
        times = max(1, round(duration_s * rate))
        twist = f'{{linear: {{x: {linear_x}}}, angular: {{z: {angular_z}}}}}'
        cmd = f'ros2 topic pub --times {times} -r {rate} /cmd_vel geometry_msgs/msg/Twist "{twist}"'
        return self.run(cmd, timeout=duration_s + timeout_margin)

    def drive_distance(self, distance_m, speed_mps, rate=20):
        """부호 있는 distance_m만큼(음수=후진) speed_mps 크기로 주행 후 자동 정지."""
        direction = 1.0 if distance_m >= 0 else -1.0
        duration_s = abs(distance_m) / speed_mps
        self.publish_twist(linear_x=direction * speed_mps, duration_s=duration_s, rate=rate)
        self.stop()

    def rotate(self, angular_speed, duration_s, rate=20):
        """angular_speed[rad/s]로 duration_s초 회전 후 자동 정지."""
        self.publish_twist(angular_z=angular_speed, duration_s=duration_s, rate=rate)
        self.stop()

    def stop(self):
        cmd = 'ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"'
        return self.run(cmd, timeout=5)
