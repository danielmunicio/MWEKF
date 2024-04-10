import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64


class MPCPublisherNode(Node):
    def __init__(self):
        self.acceleration = 0 
        self.steering = 0
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(AckermannDriveStamped, '/cmd', 1)
        self.curr_throttle_sub = self.create_subscription(Float64, '/control/throttle', self.acceleration_callback, 1)
        self.curr_steer_sub = self.create_subscription(Float64, '/control/steer', self.steer_callback, 1) 

    def publish_message(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.acceleration = self.acceleration  
        msg.drive.steering_angle = self.steering_angle
        self.publisher.publish(msg)

    def steer_callback(self, msg: Float64):
        '''
        Set Steering Angle based on MPC Algorithm 
        '''
        self.steering_angle = float(msg.data)
        self.publish_message()

    def acceleration_callback(self, msg: Float64):
        '''
        Set acceleration callback based on MPC Algorithm 
        '''
        self.acceleration = float(msg.data)
        self.publish_message()

        