#!/usr/bin/env bash
# scripts/stop_all.sh
#
# 로봇/시뮬레이션 실행 중 프로세스(드라이버, Nav2, 라이다, Gazebo, RViz, 이 패키지의 노드와
# 보조 스크립트)를 단계적으로 종료함. 로봇(Jetson)과 라이다 노트북에서 각각 실행함.
#
# 종료 순서: SIGCONT(Ctrl-Z로 정지된 것을 깨움) -> SIGINT -> SIGTERM -> SIGKILL.
# 처음부터 SIGKILL을 쓰지 않는 이유는 노드가 시리얼 포트와 GPU 자원을 스스로 정리하게 하기 위함임.
# 프로세스 이름은 단어 경계로 정확히 매칭하므로 경로에 ros2_ws가 들어간 편집기 등은 건드리지 않음.
#
# 사용:
#   stop_all.sh        목록 출력 후 종료
#   stop_all.sh -l     목록만 출력(종료하지 않음)
#   stop_all.sh -f     단계 없이 바로 SIGKILL
#
# 편하게 쓰려면 PATH 위의 이름으로 연결함(예: ln -sf <이 파일> ~/.local/bin/rrr).
# 대상을 늘리려면 아래 NAMES 배열에 실행 파일 이름을 추가함.

LIST_ONLY=0
FORCE=0
for a in "$@"; do
  case "$a" in
    -l) LIST_ONLY=1 ;;
    -f) FORCE=1 ;;
    -h|--help) sed -n '2,19p' "$0"; exit 0 ;;
    *) echo "알 수 없는 옵션: $a (-l | -f | -h)"; exit 2 ;;
  esac
done

NAMES=(
  # 로봇 드라이버 / TF
  turtlebot3_ros ld08_driver hlds_laser_publisher single_lidar_node
  robot_state_publisher joint_state_publisher static_transform_publisher imu_tilt_broadcaster
  # 3D 라이다
  velodyne_driver_node velodyne_transform_node velodyne_laserscan_node
  # Nav2
  map_server amcl lifecycle_manager controller_server planner_server behavior_server
  bt_navigator waypoint_follower smoother_server velocity_smoother collision_monitor
  component_container component_container_isolated component_container_mt
  # 시각화 / 시뮬레이션
  rviz2 gzserver gzclient gazebo spawn_entity.py
  # 이 패키지의 노드
  mission_executor surface_profiler
)
JOIN=$(IFS='|'; echo "${NAMES[*]}")
# ros2 launch/run/CLI, 이름 단어 경계 매칭, 이 패키지의 보조 스크립트(.py)
PAT="(^|[ /])(${JOIN//./\\.})( |$)|ros2 (launch|run|topic|service|lifecycle|bag|action|param) |(check_imu_[a-z_]+|auto_calibration_drive|scan_room_for_calibration)\\.py"

alive() { pgrep -f "$PAT" >/dev/null; }

wait_gone() {
  local i
  for i in $(seq 1 "$1"); do alive || return 0; sleep 1; done
  alive && return 1 || return 0
}

echo "[*] 대상 프로세스:"
pgrep -af "$PAT" | sed 's/^/    /' || true
alive || echo "    (없음)"
[ "$LIST_ONLY" -eq 1 ] && exit 0

if alive; then
  pkill -CONT -f "$PAT" 2>/dev/null
  if [ "$FORCE" -eq 0 ]; then
    pkill -INT -f "$PAT" 2>/dev/null;  wait_gone 3 || { pkill -TERM -f "$PAT" 2>/dev/null; wait_gone 3; }
  fi
  alive && { pkill -KILL -f "$PAT" 2>/dev/null; sleep 0.5; }
fi

if alive; then
  echo "[!] 종료되지 않은 프로세스(D 상태 등):"
  pgrep -af "$PAT" | sed 's/^/    /'
else
  echo "[+] 대상 프로세스 모두 종료됨"
fi

# 시리얼 포트를 아직 잡고 있는 프로세스가 있는지 보고
if command -v fuser >/dev/null 2>&1; then
  for dev in /dev/ttyACM0 /dev/ttyUSB0; do
    [ -e "$dev" ] && fuser -v "$dev" 2>&1 | grep -v "^ *$" | sed "s|^|    ${dev}: |"
  done
fi

# 그래프 캐시(노드 이름 잔상) 정리
command -v ros2 >/dev/null 2>&1 && ros2 daemon stop >/dev/null 2>&1
echo "[+] ros2 daemon 정리 완료"
