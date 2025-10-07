from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo de turtlesim
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            output='screen'
        ),

        # Tu nodo que dibuja el número (sprint1.py)
        Node(
            package='g01_prii3_turtlesim',
            executable='sprint1',
            name='draw_number',
            output='screen'
        )
    ])

