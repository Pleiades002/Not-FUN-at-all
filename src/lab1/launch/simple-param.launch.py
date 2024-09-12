from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim_plus',
            namespace='',
            executable='turtlesim_plus_node.py',
            name='turtlesim'
        ),
        Node(
            package='lab1',
            namespace='linear',
            executable='noise_generator.py',
            name='linear_noise',
            parameters=[
                {'rate': 10.0}
            ]
        ),
        Node(
            package='lab1',
            namespace='angular',
            executable='noise_generator.py',
            name='angular_noise',
            parameters=[
                {'rate': 20.0}
            ]
        ),
        Node(
            package='lab1',
            namespace='',
            executable='velocity_mux.py',
            name='mux',
            parameters=[
                {'rate': 30.0}
            ],
            remappings=[
                ('/cmd_vel', 'turtle1/cmd_vel')
            ]
        ),
    ])
