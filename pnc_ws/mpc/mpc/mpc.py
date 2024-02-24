import rclpy
from mpc_controller import KinMPCPathFollower

def main(args=None):
    rclpy.init(args=args)
        
    # node = LidarBoardDetector()
    node = KinMPCPathFollower()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

    rclpy.spin(yolo_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()