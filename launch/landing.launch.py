"""Landing target detection, LiDAR landing validation and the mission/landing state machine."""
from launch import LaunchDescription
from launch_ros.actions import Node

P = [{'use_sim_time': True}]


def generate_launch_description():
    return LaunchDescription([
        Node(package='landing_detector', executable='detector_node', parameters=P, output='log'),
        Node(package='landing_validator', executable='validator_node', parameters=P, output='log'),
        Node(package='landing_controller', executable='mission_manager', parameters=P, output='screen'),
    ])
