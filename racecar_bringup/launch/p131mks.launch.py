from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    print("boobies")
    graphslam_node = Node(
        package = "graphslam_global",
        executable = "graphslam_global"
    )



    # Add Nodes to the LaunchDescription
    ld.add_action(graphslam_node)

    return ld
