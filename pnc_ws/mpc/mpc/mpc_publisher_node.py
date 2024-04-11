import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu


class MPCPublisherNode(Node):
    def __init__(self):
        self.acceleration = 0 
        self.steering = 0
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(AckermannDriveStamped, '/cmd', 1)
        self.curr_throttle_sub = self.create_subscription(Float64, '/control/throttle', self.penis_callback, 1)
        self.curr_steer_sub = self.create_subscription(Float64, '/control/steer', self.boobies_callback, 1) 
        self.penis_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 1)
        print('NODE INITIALIZED BOOBIES')
        self.get_logger().debug("NODE INITIALIZED")
        print('NODE INITIALIZED BOOBIES BOOBIES')



    def publish_message(self):
        print('Publishing message')
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.acceleration = self.acceleration
        msg.drive.steering_angle = self.steering_angle
        print(msg)
        self.publisher.publish(msg)
    
    def imu_callback(self, msg: Imu): 
        print('Recieved IMU DATA BOOBIEs')

    def boobies_callback(self, msg: Float64):
        '''
        Set Steering Angle based on MPC Algorithm 
        '''
        print('Recieved Steering Message')
        self.get_logger().debug("RECIEVED STEERING CALLBACK")
        self.steering_angle = float(msg.data)
        self.publish_message()

    def penis_callback(self, msg: Float64):
        '''
        Set acceleration callback based on MPC Algorithm 
        '''
        print('Recieved Acceleration Message')
        self.get_logger().debug("ACCELERATION RECIEVED")
        self.acceleration = float(msg.data)
        self.publish_message()
