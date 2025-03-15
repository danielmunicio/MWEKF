from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    print("boobies")
    mpc_node = Node(
        package = "mpc",
        executable = "mpc"
    )
    global_path_node = Node(
        package = "global_path",
        executable = "global_path"
    )
    local_path_node = Node(
        package = "local_path",
        executable = "local_path"
    )
    graphslam_node = Node(
        package = "graphslam_global",
        executable = "graphslam_global"
    )
    # Add Nodes to the LaunchDescription
    ld.add_action(mpc_node)
    ld.add_action(global_path_node)
    ld.add_action(local_path_node)
    ld.add_action(graphslam_node)

    return ld
