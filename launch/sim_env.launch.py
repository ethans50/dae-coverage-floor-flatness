#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
# Authors: Joep Tool

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration

# params.yaml을 못 읽을 때만 쓰는 폴백. 다른 진입점들이 쓰는 값과 맞춰둠.
FALLBACK_DAE_FILE = 'INU_9_211.dae'
WORLD_XACRO = 'coverage_flatness_env.world.xacro'


def _read_dae_file(pkg_share):
    """params.yaml의 environment_modeling.dae_file을 읽음.

    파일을 못 읽어도 launch가 죽지 않도록 폴백 값으로 계속 진행함.
    """
    try:
        import yaml
        with open(os.path.join(pkg_share, 'config', 'params.yaml'), 'r') as f:
            config = yaml.safe_load(f) or {}
        return config.get('environment_modeling', {}).get('dae_file', FALLBACK_DAE_FILE)
    except Exception as e:
        print(f'[sim_env.launch] Cannot read params.yaml -> Fallback to default model: {e}')
        return FALLBACK_DAE_FILE


def _generate_world(pkg_share):
    """params.yaml의 dae_file에 맞는 world를 만들어 그 경로를 돌려줌.

    2D 맵은 dae_file로부터 생성되므로 시뮬 건물도 같은 모델이어야 함. world 파일에
    모델명을 따로 적어두면 둘이 어긋나도 에러가 나지 않아, 맵과 다른 건물 안을
    주행하게 됨. 그래서 world는 템플릿으로만 두고 모델명은 실행 시점에 채움.
    """
    import xacro

    dae_file = _read_dae_file(pkg_share)
    model_name = os.path.splitext(dae_file)[0]

    # 대응하는 Gazebo 모델이 없으면 Gazebo는 건물 없는 빈 바닥을 띄우고 그대로 돎.
    # 조용히 잘못된 시뮬이 도는 것을 막기 위해 여기서 먼저 끊음.
    models_dir = os.path.join(pkg_share, 'models')
    model_sdf = os.path.join(models_dir, model_name, 'model.sdf')
    if not os.path.exists(model_sdf):
        available = sorted(
            d for d in os.listdir(models_dir)
            if os.path.exists(os.path.join(models_dir, d, 'model.sdf'))
        ) if os.path.isdir(models_dir) else []
        raise RuntimeError(
            f"The environment_modeling.dae_file in params.yaml is '{dae_file}', "
            f"but the corresponding Gazebo model '{model_name}' was not found ({model_sdf}). "
            f"Available models: {', '.join(available) if available else 'none'}. "
            f"Please update dae_file or add model.sdf and meshes under models/{model_name}/."
        )

    doc = xacro.process_file(
        os.path.join(pkg_share, 'worlds', WORLD_XACRO),
        mappings={'model_name': model_name},
    )
    world_path = os.path.join(tempfile.gettempdir(), f'dae_coverage_{model_name}.world')
    with open(world_path, 'w') as f:
        f.write(doc.toxml())
    print(f"[sim_env.launch] Generated world with model '{model_name}': {world_path}")
    return world_path


def _launch_setup(context, *args, **kwargs):
    pkg_my_dir = get_package_share_directory('dae_coverage_floor_flatness')
    launch_file_dir = os.path.join(pkg_my_dir, 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')

    # world를 직접 준 경우에는 params.yaml을 보지 않고 그 파일을 그대로 씀.
    world = LaunchConfiguration('world').perform(context) or _generate_world(pkg_my_dir)

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'sim_robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'sim_spawn_robot.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'z_pose': z_pose
        }.items()
    )

    return [gzserver_cmd, gzclient_cmd, robot_state_publisher_cmd, spawn_turtlebot_cmd]


def generate_launch_description():
    pkg_my_dir = get_package_share_directory('dae_coverage_floor_flatness')
    turtlebot3_description_dir = get_package_share_directory('turtlebot3_description')

    # Gazebo가 3D 건물 모델(Apt 등)과 로봇 메시(Mesh)를 찾을 경로 설정
    my_models_dir = os.path.join(pkg_my_dir, 'models')
    tb3_models_dir = os.path.abspath(os.path.join(turtlebot3_description_dir, '..'))
    # SetEnvironmentVariable은 셸 확장을 하지 않으므로 "$GAZEBO_MODEL_PATH"라고 쓰면
    # 그 문자열이 그대로 들어가 기존 설정이 날아감. substitution으로 이어붙여야 함.
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            f"{my_models_dir}:{tb3_models_dir}:",
            EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
        ]
    )

    return LaunchDescription([
        gazebo_model_path,

        DeclareLaunchArgument(
            'world',
            default_value='',
            description='world 파일 경로. 비워두면(기본값) params.yaml의 '
                        'environment_modeling.dae_file에 맞춰 자동으로 만들어 씀'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument('x_pose', default_value='0.0', description='로봇 스폰 x 좌표'),
        DeclareLaunchArgument('y_pose', default_value='0.0', description='로봇 스폰 y 좌표'),
        DeclareLaunchArgument('z_pose', default_value='0.0', description='로봇 스폰 z 좌표'),

        OpaqueFunction(function=_launch_setup),
    ])
