from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # The YOLO node
        Node(
            package='camera_perception',
            executable='yolov8_node',
            name='yolo_node'
        ),
        # The sensor fusion node
        Node(
            package='sensor_fusion',
            executable='fusion_node',
            name='fusion_node'
        )
    ])