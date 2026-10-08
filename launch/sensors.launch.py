"""Sensors, TF2, GPS-free localisation and fusion.

static_frames, camera_processor, lidar_processor, ground_tracker (sensor model),
localization_node, fusion_node.
"""
from launch import LaunchDescription
from launch_ros.actions import Node

P = [{'use_sim_time': True}]


def generate_launch_description():
    return LaunchDescription([
        Node(package='aeronav_description', executable='static_frames', parameters=P, output='log'),
        Node(package='camera_processor', executable='camera_processor', parameters=P, output='log'),
        Node(package='lidar_processor', executable='lidar_processor', parameters=P, output='log'),
        Node(package='aeronav_simulation', executable='ground_tracker', parameters=P, output='log'),
        Node(package='localization', executable='localization_node', parameters=P, output='log'),
        Node(package='sensor_fusion', executable='fusion_node', parameters=P, output='log'),
    ])
