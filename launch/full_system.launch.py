"""Complete PS05 system: simulation + sensors + navigation + landing + PX4 interface
+ evaluation + scenario harness (+ optional dashboard bridge and RViz).

ros2 launch /workspace/launch/full_system.launch.py gui:=false rviz:=false dashboard:=true run_id:=demo_001
The active scenario is selected with PS05_SCENARIO_FILE (see scripts/run_mission.sh).
"""
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROOT = Path(os.environ.get('PS05_ROOT', '/workspace'))
HERE = Path(__file__).resolve().parent
P = [{'use_sim_time': True}]


def inc(name, args=None):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(HERE / name)), launch_arguments=(args or {}).items())


def generate_launch_description():
    lc = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('dashboard', default_value='true'),
        DeclareLaunchArgument('world', default_value='ps05_challenge'),
        DeclareLaunchArgument('start_pose', default_value='5,5,0.25,0,0,0'),
        DeclareLaunchArgument('run_id', default_value=os.environ.get('PS05_RUN_ID', 'manual')),
        inc('simulation.launch.py', {'gui': lc('gui'), 'world': lc('world'), 'start_pose': lc('start_pose')}),
        TimerAction(period=6.0, actions=[
            inc('sensors.launch.py'),
            Node(package='px4_interface', executable='px4_interface', parameters=P, output='log'),
            Node(package='evaluation', executable='evaluation_node', output='log',
                 parameters=[{'use_sim_time': True, 'run_id': lc('run_id')}]),
            Node(package='aeronav_simulation', executable='scenario_events', output='log',
                 parameters=[{'use_sim_time': True, 'world': lc('world')}]),
            Node(package='dashboard_bridge', executable='dashboard_bridge', parameters=P, output='log',
                 condition=IfCondition(lc('dashboard'))),
            Node(package='rviz2', executable='rviz2', arguments=['-d', str(ROOT / 'rviz/ps05.rviz')], parameters=P,
                 output='log', condition=IfCondition(lc('rviz'))),
        ]),
        TimerAction(period=9.0, actions=[inc('navigation.launch.py'), inc('landing.launch.py')]),
    ])
