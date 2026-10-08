"""3D mapping, global planning and local obstacle avoidance."""
from launch import LaunchDescription
from launch_ros.actions import Node

P = [{'use_sim_time': True}]


def generate_launch_description():
    return LaunchDescription([
        Node(package='mapping', executable='mapping_node', parameters=P, output='log'),
        Node(package='planner_3d', executable='planner_node', parameters=P, output='log'),
        Node(package='obstacle_avoidance', executable='avoidance_node', parameters=P, output='log'),
    ])
