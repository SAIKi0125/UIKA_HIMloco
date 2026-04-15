from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rl_sar',
            executable='uika_mock_publisher',
            name='uika_mock_publisher',
            output='screen',
        ),
        Node(
            package='rl_sar',
            executable='uika',
            name='uika_rl_node',
            output='screen',
        ),
    ])
