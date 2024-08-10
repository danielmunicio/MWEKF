import can
from time import sleep, perf_counter

import rclpy
from rclpy.node import Node
import numpy as np
from all_settings.all_settings import SteeringSettings as settings
from std_msgs.msg import Float64

class SBWMotorCAN(Node):
    def __init__(self, bus: can.ThreadSafeBus):
        super().__init__("sbw_can")
        self.bus = bus

        #Subscribers
        self.cmd_sub = self.create_subscription(Float64, '/control/steer', self.steer_cb, 1)
    
    def create_data(self, angle_ticks):
        return (bytearray([0xA4, 0x00])
                +int.to_bytes(int(settings.MAX_MOTOR_SPEED*(180/np.pi)), length=2, byteorder='little', signed=False)
                +int.to_bytes(int(angle_ticks), length=4, byteorder='little', signed=True))
    def set_acc_limits(self):
        assert 100<=int(settings.MAX_MOTOR_ACC*180/np.pi)<=60000, f"acceleration must be between 100 and 60,000 deg/s/s but got {int(settings.MAX_MOTOR_ACC*180/np.pi)}"
        acc = bytearray([0xA3, 0x00, 0x00, 0x00])
        dec = bytearray([0xA3, 0x01, 0x00, 0x00])
        limit = int.to_bytes(int(settings.MAX_MOTOR_ACC*180/np.pi), length=4, byteorder='little', signed=False)
        acc_msg = can.Message(
            arbitration_id=0x141,
            is_extended_id=False,
            dlc=8,
            data=acc+limit
        )
        dec_msg = can.Message(
            arbitration_id=0x141,
            is_extended_id=False,
            dlc=8,
            data=dec+limit
        )
        sleep(0.1)
        self.bus.send(acc_msg)
        self.bus.send(dec_msg)
        print(acc+limit)
    def steer_cb(self, msg: Float64):
        assert np.abs(msg.data) < settings.MAX_MOTOR_POS, f"steering angle too big: must be less than {settings.MAX_MOTOR_POS:.3f} but got {msg.data:.3f}."
        angle_ticks = int(msg.data
                          *settings.MOTOR_TO_WHEELS_RATIO
                          *settings.MOTOR_TICKS_PER_RAD)
        print("msg.data:", msg.data)
        print("angle_ticks:", angle_ticks)
        print(self.create_data(angle_ticks))
        msg = can.Message(
            arbitration_id=0x141,
            is_extended_id=False,
            dlc=8,
            data=self.create_data(angle_ticks),
        )
        self.bus.send(msg)
        

def main(args=None):
    rclpy.init(args=args)
    with can.ThreadSafeBus(**settings.CAN_SETTINGS) as bus:
        sbw_can_node = SBWMotorCAN(bus)
        sbw_can_node.set_acc_limits()
        rclpy.spin(sbw_can_node)
        rclpy.shutdown()

if __name__ == "__main__":
    main()
