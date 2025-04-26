import rclpy 
from rclpy.node import Node

from all_settings.all_settings import CameraOnlySettings as settings
from .realsense_camera_only import RealsenseCameraOnly 
from .depth_anything_camera_only import DepthAnythingCameraOnly

def main(args=None):
    rclpy.init(args=args)
    if settings.dual_camera: 
        # I didnt code this yet
        pass
    elif settings.logitech_camera:
        node = DepthAnythingCameraOnly()
        rclpy.spin(node)
        # I also didn't code this yet
        pass 
    elif settings.realsense_camera: 
        node = RealsenseCameraOnly()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()