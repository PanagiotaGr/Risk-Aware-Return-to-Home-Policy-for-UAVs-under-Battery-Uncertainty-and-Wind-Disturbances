from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uav_reinflearning',
            executable='uav_agent',
            name='uav_rl_agent',
            output='screen',
        )
    ])
