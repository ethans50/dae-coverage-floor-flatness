# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
ROS_DISTRO = os.environ.get('ROS_DISTRO')

# 외부 저장소 경로의 기본값. params.yaml을 못 읽을 때만 쓰는 폴백이며,
# 다른 진입점들이 쓰는 값과 동일하게 맞춰둠.
FALLBACK_WORKSPACE_ROOT = '~/dae_floor_maps'
FALLBACK_GRID_DIR = 'maps/grid'
MAP_FILENAME = 'map_from_dae.yaml'   # map_generator.py가 쓰는 고정 파일명


def _resolve_default_map_path(pkg_share):
    """맵 기본 경로를 params.yaml에서 직접 계산함.

    경로의 단일 출처를 params.yaml로 유지하기 위함 - 여기서 경로를 따로
    적어두면 global.workspace_root를 바꿨을 때 Nav2만 옛 경로를 보게 됨.
    파일을 못 읽어도 launch가 죽지 않도록 폴백 값으로 계속 진행함.
    """
    workspace_root, grid_dir = FALLBACK_WORKSPACE_ROOT, FALLBACK_GRID_DIR
    try:
        import yaml
        with open(os.path.join(pkg_share, 'config', 'params.yaml'), 'r') as f:
            config = yaml.safe_load(f) or {}
        workspace_root = config.get('global', {}).get('workspace_root', workspace_root)
        grid_dir = config.get('environment_modeling', {}).get('output_grid_dir', grid_dir)
    except Exception as e:
        print(f'[tb3_waffle_nav2.launch] params.yaml을 읽지 못해 기본 경로로 폴백함: {e}')
    return os.path.join(os.path.expanduser(workspace_root), grid_dir, MAP_FILENAME)


def generate_launch_description():
    pkg_share = get_package_share_directory('dae_coverage_floor_flatness')

    # map
    default_map_path = _resolve_default_map_path(pkg_share)
    map_dir = LaunchConfiguration('map', default=default_map_path)

    # tutlebot3_navigation2 param (waffle)
    default_param_path = os.path.join(pkg_share, 'config', 'tb3_waffle_nav2_params.yaml')
    param_dir = LaunchConfiguration('params_file', default=default_param_path)

    # Nav2 bringup
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    # tutlebot3_navigation2 rviz
    rviz_config_dir = os.path.join(pkg_share, 'rviz', 'tb3_navigation2.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    return LaunchDescription([
        # default_value에 LaunchConfiguration 대신 실제 경로 문자열을 줌 -
        # `ros2 launch ... --show-args`에 해석된 경로가 그대로 보이게 하기 위함.
        DeclareLaunchArgument(
            'map',
            default_value=default_map_path,
            description='Full path to map file to load (default: params.yaml의 workspace_root 기준)'),

        DeclareLaunchArgument(
            'params_file',
            default_value=default_param_path,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to start RVIZ'), 

        # Nav2 핵심 Bringup 실행 (변경 없음)[cite: 4]
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': param_dir,
                'map': LaunchConfiguration('map')}.items(),
        ),

        # 내 패키지의 RVIZ 설정 파일 로드
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
            output='screen'),
    ])