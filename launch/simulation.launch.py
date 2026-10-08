"""Gazebo Harmonic world + PX4 SITL (standalone gz mode) + Micro XRCE-DDS Agent + ros_gz_bridge.

ros2 launch /workspace/launch/simulation.launch.py gui:=false
"""
import os
import shutil
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROOT = Path(os.environ.get('PS05_ROOT', '/workspace'))
PX4_DIR = Path(os.environ.get('PX4_DIR', '/opt/PX4-Autopilot'))
PX4_BUILD = PX4_DIR / 'build/px4_sitl_default'


def setup(context):
    world = LaunchConfiguration('world').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    pose = LaunchConfiguration('start_pose').perform(context)
    run_dir = Path(LaunchConfiguration('px4_rootfs').perform(context))

    resource = ':'.join([str(ROOT / 'simulation/models'), str(ROOT / 'simulation/worlds'),
                         str(PX4_DIR / 'Tools/simulation/gz/models'), os.environ.get('GZ_SIM_RESOURCE_PATH', '')])
    env = {**os.environ, 'GZ_SIM_RESOURCE_PATH': resource}

    # custom PS05 airframe (GPS disabled, external-vision aiding) -> PX4 runtime airframe dir
    af_src = ROOT / 'simulation/px4/airframes'
    af_dst = PX4_BUILD / 'etc/init.d-posix/airframes'
    for f in af_src.glob('*'):
        shutil.copy(f, af_dst / f.name)
    # fresh PX4 working dir every run -> no stale parameters between trials
    shutil.rmtree(run_dir, ignore_errors=True)
    run_dir.mkdir(parents=True, exist_ok=True)

    world_file = str(ROOT / 'simulation/worlds' / f'{world}.sdf')
    gz_cmd = ['gz', 'sim', '-r', '-v', '2', world_file] if gui else \
        ['gz', 'sim', '-s', '-r', '--headless-rendering', '-v', '2', world_file]
    gz = ExecuteProcess(cmd=gz_cmd, env=env, output='screen', name='gz_sim')

    px4_env = {**env, 'PX4_GZ_STANDALONE': '1', 'PX4_SYS_AUTOSTART': '4050', 'PX4_SIM_MODEL': 'gz_x500_ps05',
               'PX4_GZ_WORLD': world, 'PX4_GZ_MODEL_POSE': pose, 'HEADLESS': '1',
               'PX4_GZ_MODELS': str(ROOT / 'simulation/models'), 'PX4_GZ_WORLDS': str(ROOT / 'simulation/worlds')}
    px4 = ExecuteProcess(cmd=[str(PX4_BUILD / 'bin/px4'), '-d', '-w', str(run_dir), str(PX4_BUILD / 'etc')],
                         cwd=str(run_dir), env=px4_env, output='screen', name='px4_sitl')
    agent = ExecuteProcess(cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'], output='log', name='xrce_agent')
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge', output='log',
                  parameters=[{'config_file': str(ROOT / 'simulation/configs/bridge.yaml'), 'use_sim_time': True,
                               'qos_overrides./tf_static.publisher.durability': 'transient_local'}])
    return [gz, agent, TimerAction(period=4.0, actions=[px4]), TimerAction(period=2.0, actions=[bridge])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='ps05_challenge'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('start_pose', default_value='5,5,0.25,0,0,0'),
        DeclareLaunchArgument('px4_rootfs', default_value='/tmp/px4_rootfs'),
        OpaqueFunction(function=setup),
    ])
