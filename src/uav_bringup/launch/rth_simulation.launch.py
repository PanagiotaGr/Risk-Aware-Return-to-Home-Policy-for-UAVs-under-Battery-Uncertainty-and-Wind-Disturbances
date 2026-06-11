#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uav_sim',
            executable='uav_sim',
            name='uav_sim',
            output='screen',
        ),
        Node(
            package='uav_planner',
            executable='uav_planner',
            name='uav_planner',
            output='screen',
        ),
    ])
