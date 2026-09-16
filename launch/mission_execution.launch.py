# launch/mission_execution.launch.py

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Jetson Orin Nano(turtlebot3 Waffle)에서 MissionExecutor(mission_executor.py) 실행

    전제 조건:
    1. turtleBot3 bringup이 이미 실행 중이어야 함.
    2. turtlebot3_navigation2의 navigation2.launch.py(map_server, amcl,
      lifecycle_manager, controller_server, planner_server, bt_navigator 등 포함)가
      이미 실행 중이어야 함.

    이 launch 파일은 그 위에서 mission_executor 노드 하나만 추가로 띄우는 역할만 함.
      (Nav2 스택 중복 실행 방지)

    Terminal Command:
        ros2 launch dae_coverage_floor_flatness mission_execution.launch.py is_sim:=true
        ros2 launch dae_coverage_floor_flatness mission_execution.launch.py is_sim:=false

        # EVAL.md 알고리즘 비교 실험용 - 라벨을 주면 이번 미션의 출력물(csv/png)을
        # <workspace_root>/eval_runs/<라벨>/ 아래로 모아 저장함(안 주면 기존 동작 그대로):
        ros2 launch dae_coverage_floor_flatness mission_execution.launch.py is_sim:=true eval_run_label:=algo1_centroid

        # run_ts까지 같이 주면 drive_debug/stall_report/robot_path csv 파일명이
        # surface_profiling.launch.py 쪽 combined/raw pcd 파일명과 통일됨(같은
        # 값을 양쪽 launch 명령에 사람이 직접 동일하게 입력해야 함,
        # eval_run_label과 동일한 패턴) - 안 주면(기본값) 기존처럼 각자 알아서
        # 시각을 찍음. 형식은 'YYYY-MM-DD_HH-MM-SS'(예: `$(date +%Y-%m-%d_%H-%M-%S)`):
        ros2 launch dae_coverage_floor_flatness mission_execution.launch.py is_sim:=true eval_run_label:=algo1_centroid run_ts:=2026-09-14_14-17-31
    """

    is_sim_arg = DeclareLaunchArgument(
        'is_sim',
        default_value='false',
        description='true: Gazebo Simulation Mode, false: Real-world Mode'
    )
    eval_run_label_arg = DeclareLaunchArgument(
        'eval_run_label',
        default_value='',
        description='EVAL.md 알고리즘 비교 실험용 라벨 - 비어있으면(기본값) 기존과 동일한 flat 경로에 저장함'
    )
    run_ts_arg = DeclareLaunchArgument(
        'run_ts',
        default_value='',
        description="EVAL.md 실험용 공유 타임스탬프('YYYY-MM-DD_HH-MM-SS') - surface_profiling.launch.py에도 같은 값을 줘야 파일명이 통일됨. 비어있으면(기본값) 기존처럼 각자 시각을 찍음"
    )

    is_sim = LaunchConfiguration('is_sim')
    eval_run_label = LaunchConfiguration('eval_run_label')
    run_ts = LaunchConfiguration('run_ts')

    mission_executor_node = Node(
        package='dae_coverage_floor_flatness',
        executable='mission_executor',
        name='mission_executor_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'is_sim': is_sim,
            'use_sim_time': is_sim,
            'eval_run_label': eval_run_label,
            'run_ts': run_ts,
        }]
    )

    ld = LaunchDescription()
    ld.add_action(is_sim_arg)
    ld.add_action(eval_run_label_arg)
    ld.add_action(run_ts_arg)
    ld.add_action(mission_executor_node)

    return ld