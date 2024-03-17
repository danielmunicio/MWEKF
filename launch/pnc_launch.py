from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # MPC (Control) Node (WIP)
        Node(
            package='mpc',
            executable='mpc',
            name='mpc_node_actual',
        ),
        # global path planning node
        Node(
            package='global_path',
            executable='global_path',
            name='global_path',
        )
    ])
