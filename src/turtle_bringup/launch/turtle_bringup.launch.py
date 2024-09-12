from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_description = LaunchDescription()
    
    package_turtle_bringup = 'turtle_bringup'
    namespace_controller = 'test_controller'
    namespace_crazy_turtle = 'test_crazy_turtle'
    
    frequency_launch_arg = DeclareLaunchArgument(
        'frequency',
        default_value='100.0'
    )
    launch_description.add_action(frequency_launch_arg)

    turtlesim_node = Node(
        package='turtlesim_plus',
        namespace='',
        executable='turtlesim_plus_node.py',
        name='turtlesim'
    )
    launch_description.add_action(turtlesim_node)

    kill_turtle1 = ExecuteProcess(
        cmd=[
            'ros2 service call ',
            '/remove_turtle ',
            'turtlesim/srv/Kill',
            "\"{name: 'turtle1'}\""
        ],
        shell=True
    )
    launch_description.add_action(kill_turtle1)

    spawn_turtle = ExecuteProcess(
        cmd=[
            'ros2 service call ',
            '/spawn_turtle ',
            'turtlesim/srv/Spawn ',
            f'"{{x: 2.5, y: 2.5, theta: 0.0, name: {namespace_controller}}}"'
        ],
        shell=True
    )
    launch_description.add_action(spawn_turtle)

    controller_node = Node(
        package=package_turtle_bringup,
        namespace=namespace_controller,
        executable='controller.py',
        name='controller',
        parameters=[
            {'frequency': LaunchConfiguration('frequency')}
        ]
    )
    launch_description.add_action(controller_node)

    crazy_turtle_node = Node(
        package=package_turtle_bringup,
        namespace=namespace_crazy_turtle,
        executable='crazy_turtle.py',
        name='crazy_turtle',
        parameters=[
            {'turtle_name': namespace_crazy_turtle},
            {'frequency': LaunchConfiguration('frequency')}
        ],
        shell=True
    )
    launch_description.add_action(crazy_turtle_node)

    crazy_pizza_node = Node(
        package=package_turtle_bringup,
        namespace='',
        executable='crazy_pizza.py',
        name='crazy_pizza'
    )
    launch_description.add_action(crazy_pizza_node)

    return launch_description