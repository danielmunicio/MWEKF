from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',  # or your camera driver package
            executable='realsense2_camera_node',
            name='realsense_imu',
            namespace='camera',
            parameters=[{
                'enable_color': True,
                'enable_depth': True,
                'enable_gyro': True,
                'enable_accel': True,
                'enable_infra': False,
                'align_depth.enable': True,
                'gyro_fps': 200,
                'accel_fps': 200,
                'serial_no': '344422072170',
            }]
        ),
        Node(
            package='realsense2_camera',  # or your camera driver package
            executable='realsense2_camera_node',
            name='realsense2',
            namespace='camera',
            parameters=[{
                'enable_color': True,
                'enable_depth': True,
                'enable_infra': False,
                'align_depth.enable': True,
                'serial_no': '042222071145',
            }]
        ),
    ])
