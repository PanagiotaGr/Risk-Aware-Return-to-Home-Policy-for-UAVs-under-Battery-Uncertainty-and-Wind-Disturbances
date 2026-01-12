from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_noise_adaptive_planner',
            executable='noise_adaptive_node',
            name='lidar_noise_adaptive_planner',
            output='screen',
            parameters=[{
                # αν ο controller_server σου έχει άλλο όνομα, το αλλάζεις εδώ
                'target_node_name': 'controller_server',
                'low_noise_threshold': 0.01,
                'high_noise_threshold': 0.05,
            }]
        )
    ])
