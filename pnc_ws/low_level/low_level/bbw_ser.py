import serial
from time import sleep, perf_counter

import rclpy
from rclpy.node import Node
import numpy as np
from all_settings.all_settings import BrakingSettings as settings
from std_msgs.msg import Float64

class BBWSerial(Node):
    def __init__(self, ser: serial.Serial):
        super().__init__("bbw_ser")
        self.ser = ser
        #Subscribers
        self.cmd_sub = self.create_subscription(Float64, '/control/throttle', self.brake_callback, 1)
    
    def get_brake_pressure(self, acceleration: float):
        """get pneumatic pressure to send to BBW in PSI to acheive a given acceleration
        This is not implemented yet and may/should be adapted to include more information about the car's state.

        Args:
            acceleration (float): desired acceleration in m/s. Brakes should be activated if this is *negative*!

        Returns:
            float: brake pressure in PSI
        """
        return acceleration

    def brake_callback(self, msg: Float64):
        volts = self.get_brake_pressure(msg.data)*settings.VOLTS_PER_PSI + settings.VOLTS_FOR_ZERO_PSI
        duty_cycle = int(round(np.clip(volts, 0, settings.VMAX)*(255/settings.VMAX)))
        self.ser.write((str(duty_cycle)+"\n").encode())
        # perhaps implement a serial read here to get the current measured pressure
        

def main(args=None):
    rclpy.init(args=args)
    with serial.Serial(**settings.SERIAL_SETTINGS) as ser:
        bbw_ser_node = BBWSerial(ser)
        rclpy.spin(bbw_ser_node)
        rclpy.shutdown()

if __name__ == "__main__":
    main()
