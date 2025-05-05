import rclpy
from rclpy.node import Node

from all_settings.all_settings import GraphSLAMSettings as settings
from .ground_truth_publisher import Ground_Truth_Publisher
from .graphslam_global import GraphSLAM_Global
from .graphslam_frontend_mwekf import GraphSLAM_MWEKF

def main(args=None):#
    rclpy.init(args=args)
    if settings.using_mwekf:
        graphslam_node = GraphSLAM_MWEKF()
        rclpy.spin(graphslam_node)
        rclpy.shutdown()
    elif settings.bypass_SLAM == True:
        graphslam_bypass_node = Ground_Truth_Publisher()
        rclpy.spin(graphslam_bypass_node)
        rclpy.shutdown()
    else:
        graphslam_global_node = MWEKF()
        rclpy.spin(graphslam_global_node)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    