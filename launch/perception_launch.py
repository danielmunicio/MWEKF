from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # The YOLO node
        Node(
            package='camera_perception',
            executable='yolov8_node',
            name='yolo_node_actual'
        ),
        # The LiDAR node
        Node(
            package='lidar_perception_cpp',
            executable='lidar_node_cpp',
            name='lidar_node_actual'
        ),
        # The sensor fusion node (currently Python prototype)
        Node(
            package='sensor_fusion',
            executable='fusion_node',
            name='fusion_node_proto'
        )
    ])