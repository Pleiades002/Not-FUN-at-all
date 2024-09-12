from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_description = LaunchDescription()

    rate_launch_arg = DeclareLaunchArgument(
        'rate',
        default_value='5.0'
    )
    launch_description.add_action(rate_launch_arg)

    turtlesim_node = Node(
        package='turtlesim_plus',
        namespace='',
        executable='turtlesim_plus_node.py',
        name='turtlesim'
    )
    launch_description.add_action(turtlesim_node)

    package_name = 'lab1'
    executable_name = 'noise_generator.py'
    namespace = ['linear', 'angular']
    rate = [10.0, 30.0]
    for i in range(len(namespace)):
        noise_gen = Node(
            package=package_name,
            namespace=namespace[i],
            executable=executable_name,
            name=namespace[i] + '_noise',
            parameters=[
                {'rate': float(rate[i])}
            ]
        )
        launch_description.add_action(noise_gen)

    velocity_mux = Node(
        package=package_name,
        namespace='',
        executable='velocity_mux.py',
        name='mux',
        parameters=[
            {'rate': 30.0}
        ],
        remappings=[
            ('/cmd_vel', 'turtle1/cmd_vel')
        ]
    )
    launch_description.add_action(velocity_mux)

    return launch_description
