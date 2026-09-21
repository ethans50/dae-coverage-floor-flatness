from setuptools import setup, find_packages
import glob
import os

package_name = 'dae_coverage_floor_flatness'


def package_data_tree(src_dir):
    """중첩 디렉토리를 share 아래 같은 구조로 설치하는 data_files 항목을 만듦.

    data_files는 파일 목록만 받으므로 models/처럼 하위 폴더가 있는 것은
    직접 넘길 수 없어 이렇게 펼쳐줘야 함. sim_env.launch.py가 GAZEBO_MODEL_PATH를
    share/<패키지>/models 기준으로 잡으므로 이 설치가 빠지면 Gazebo가 건물 모델을
    찾지 못함.
    """
    entries = []
    for root, _, files in os.walk(src_dir):
        if not files:
            continue
        entries.append((
            os.path.join('share', package_name, root),
            [os.path.join(root, f) for f in files],
        ))
    return entries

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
        # Config files(params.yaml 등) 관리
        # 참고로 Input/Output files(맵, 경로 파일 등)는 사용자 홈 디렉토리의 외부 저장소(~/dae_floor_maps)에서 관리하므로 패키지에는 포함하지 않음
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml')),
        # transit 구간 전용 nav2 Behavior Tree XML
        (os.path.join('share', package_name, 'behavior_trees'), glob.glob('behavior_trees/*.xml')),
        # rviz files
        (os.path.join('share', package_name, 'rviz'), glob.glob('rviz/*.rviz')),
        # urdf files
        (os.path.join('share', package_name, 'urdf'), glob.glob('urdf/*')),
        # world files
        (os.path.join('share', package_name, 'worlds'), glob.glob('worlds/*.world') + glob.glob('worlds/*.world.xacro')),
        # Gazebo 건물 모델(model.sdf + 메시). 하위 폴더가 있어 펼쳐서 등록함
        *package_data_tree('models'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ChanggonSong',
    maintainer_email='gon05158557@gmail.com',
    description='DAE 기반 실내 환경 모델링과 커버리지 주행을 이용한 3D LiDAR 바닥 평탄도 자율 측정 시스템',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_executor = mission_execution.mission_executor:main',
            'surface_profiler = surface_profiling.surface_profiler:main',
        ],
    },
)
