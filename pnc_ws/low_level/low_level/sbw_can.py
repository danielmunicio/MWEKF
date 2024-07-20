import can
from time import sleep, perf_counter

import rclpy
from rclpy.node import Node
import numpy as np
from all_settings.all_settings import SteeringSettings as settings
from std_msgs.msg import Float64

class SBWMotorCAN(Node):
    def __init__(self, bus):
        super().__init__("sbw_can")
        self.bus = bus

        #Subscribers
        self.cmd_sub = self.create_subscription(Float64, '/control/steer', self.steer_cb, 1)
    
    def create_data(self, angle_ticks):
        return (bytearray([0xA4, 0x00])
                +int.to_bytes(int(settings.MAX_SPEED*(np.pi/180)), length=2, byteorder='little', signed=False)
                +int.to_bytes(int(angle_ticks), length=4, byteorder='little', signed=True))

    def steer_cb(self, msg: Float64):
        angle_ticks = int(msg.data
                          *settings.MOTOR_TO_WHEELS_RATIO
                          *settings.MOTOR_TICKS_PER_RAD)
        msg = can.Message(
            arbitration_id=0x141,
            is_extended_id=False,
            dlc=8,
            data=self.create_data(msg.data),
        )
        self.bus.send(msg)
        

def main(args=None):
    rclpy.init(args=args)
    with can.Bus(**settings.CAN_SETTINGS) as bus:
        sbw_can_node = SBWMotorCAN(bus)
        rclpy.spin(sbw_can_node)
        rclpy.shutdown()

if __name__ == "__main__":
    main()
