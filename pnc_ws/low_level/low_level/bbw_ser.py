import serial
from time import sleep, perf_counter

import rclpy
from rclpy.node import Node
import numpy as np
from all_settings.all_settings import BrakingSettings as settings
from std_msgs.msg import Float64

class BBWSerial(Node):
    def __init__(self, ser):
        super().__init__("bbw_ser")
        self.ser = ser
        #Subscribers
        self.cmd_sub = self.create_subscription(Float64, '/control/throttle', self.brake_callback, 1)
    
    def get_brake_pressure(acceleration):
        return 0.0 if acceleration > 0 else 99999 # idk some fancy formula here

    def brake_callback(self, msg: Float64):
        volts = self.get_brake_pressure(msg.data)/settings.PSI_PER_VOLT
        duty_cycle = int(round(np.clip(volts, 0, settings.VMAX)*(255/5)))
        self.ser.write((str(duty_cycle)+"\n").encode())
        

def main(args=None):
    rclpy.init(args=args)
    with serial.Serial(**settings.SERIAL_SETTINGS) as ser:
        bbw_ser_node = BBWSerial(ser)
        rclpy.spin(bbw_ser_node)
        rclpy.shutdown()

if __name__ == "__main__":
    main()
