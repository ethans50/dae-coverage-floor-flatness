# launch/surface_profiling.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    3D라이다 데이터를 수신 중인 기기에서 SurfaceProfiler(surface_profiler.py)를 실행함.

    전제 조건:
    - 기기가 Jetson과 같은 ROS 2 도메인에 있어야 하며,
      Jetson이 publish하는 /tf, /tf_static을 네트워크로 수신할 수 있어야 함.
    - Velodyne VLP-16 드라이버가 노트북에 연결(랜선으로 직결 등)되어 PointCloud2를
      퍼블리시하고 있어야 함(velodyne_driver 등, 이 launch 파일이 띄우지 않음).
    - Chrony 시간 동기화가 Jetson과 노트북 사이에 맞춰져 있어야 TF lookup의
      timestamp 매칭이 정확함.

    이 노드는 시작과 동시에 /surface_profiling/stop_collection_success,
    /surface_profiling/stop_collection_abort 두 서비스를 열고 대기하므로,
    (Jetson Orin Nano에서의) mission_executor.py보다 먼저 실행되어
    있어야 함.

    Terminal Command:
        ros2 launch dae_coverage_floor_flatness surface_profiling.launch.py is_sim:=true
        ros2 launch dae_coverage_floor_flatness surface_profiling.launch.py is_sim:=false

        # 알고리즘 비교 실험용 - 라벨을 주면 이번 수집물(pcd/heatmap)을
        # <workspace_root>/eval_runs/<라벨>/ 아래로 모아 저장함(안 주면 기본 동작 그대로).
        # mission_execution.launch.py에 준 라벨과 반드시 같은 값을 줘야 함:
        ros2 launch dae_coverage_floor_flatness surface_profiling.launch.py is_sim:=true eval_run_label:=algo1_centroid

        # run_ts까지 같이 주면 combined/raw pcd 파일명이 mission_execution.launch.py
        # 쪽 drive_debug/stall_report/robot_path csv 파일명과 통일됨(양쪽 launch
        # 명령에 사람이 직접 동일한 값을 입력해야 함, eval_run_label과 동일한
        # 패턴) - 이 노드가 mission_executor.py보다 먼저 시작되므로, run_ts는
        # "이 노드가 실제로 시작한 시각"이 아니라 "이번 실험 전체를 가리키는
        # 공유 식별자"로 미리 정해서 넘기는 값임. 형식은
        # 'YYYY-MM-DD_HH-MM-SS'(예: `$(date +%Y-%m-%d_%H-%M-%S)`):
        ros2 launch dae_coverage_floor_flatness surface_profiling.launch.py is_sim:=true eval_run_label:=algo1_centroid run_ts:=<timestamp>
    """

    is_sim_arg = DeclareLaunchArgument(
        'is_sim',
        default_value='false',
        description='true면 Gazebo 시뮬레이션 모드, false면 실제 로봇(Real-world) 모드로 동작'
    )
    eval_run_label_arg = DeclareLaunchArgument(
        'eval_run_label',
        default_value='',
        description='알고리즘 비교 실험용 라벨 - 비어있으면(기본값) 평소와 동일한 flat 경로에 저장함'
    )
    run_ts_arg = DeclareLaunchArgument(
        'run_ts',
        default_value='',
        description="알고리즘 비교 실험용 공유 타임스탬프('YYYY-MM-DD_HH-MM-SS') - mission_execution.launch.py에도 같은 값을 줘야 파일명이 통일됨. 비어있으면(기본값) 평소처럼 자체 시각을 찍음"
    )

    is_sim = LaunchConfiguration('is_sim')
    eval_run_label = LaunchConfiguration('eval_run_label')
    run_ts = LaunchConfiguration('run_ts')

    surface_profiler_node = Node(
        package='dae_coverage_floor_flatness',
        executable='surface_profiler',
        name='surface_profiler_node',
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
    ld.add_action(surface_profiler_node)

    return ld