import rclpy
from rclpy.node import Node
from feb_msgs.msg import State, FebPath, Map, Cones
from ackermann_msgs.msg import AckermannDriveStamped

from numpy._typing._generic_alias import NDArray
# from eoms import get_eoms
# from dynamics import to_casadi
# import all_settings as settings
import casadi as ca
import numpy as np
from time import perf_counter, sleep
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import control as ct
from typing import Callable, Optional
from matplotlib.figure import Figure
from matplotlib.axes._axes import Axes
from matplotlib.animation import FuncAnimation
import struct
import serial
# from controller import mpcController, mpcControllerForces
# from swingupsim import SwingUpController



class HardwareInterface:
    def __init__(self, _, serial: serial.Serial):
        """initialize a Simulator object.

        Args:
            model_params (settings.ModelParameters): parameters for the dynamics model - lengths, masses, moments of inertia, etc.
            dt (float): simulator base time step. Choose to be approximately the sensor/actuator update frequency (much less than controller runtime)
            delay (float): extra time to add onto controller runtime, mimicking some latency in the control stack
            integration_method (settings.IntegrationMethod): which integration method to use. `rk` is typically much faster and is fine since we don't need sensitivities.
            noise (Callable[[], np.ndarray]): function that produces a 6-vector of noise each time it is called with no arguments. This is added to the state before it is passed into the controller.
            A_MAX (float): maximum acceleration acheivable by the motor. Controller outputs will be clipped by this value.
        """
        self.ser = serial
    def zero_encoders(self):
        while True:
            res = self._write_read(0.0)
            print(res)
            if np.abs(res[4])<0.02 and np.abs(res[6])<0.02:
                break
        self.ser.write(bytes([10]))
        self.ser.flush()
        res = np.array(struct.unpack("<Lffffff", self.ser.read(7*4)))
        print(f'state at time of zero: {res[1:]}')
        for i in range(5):
            res = self._write_read(0.0)
            print(f'next state: {res[1:]}')
            sleep(0.01)

    def run(self, controller: Callable[[float, np.ndarray[(6,), float]], float]):
        """run the simulation with a given controller. Each time the controller is run, we time how long it takes, and run the simulation for that amount of time before applying the input.

        Args:
            controller (Callable[[t, x], u]): function that gives control input from current time and system state (float, np.ndarray of shape (6,))
            x0 (np.ndarray | ca.DM): initial system state. Shape (6,).
            tf (float): final time (how long, in seconds, to run the simulation for)
        """

        ret = self._write_read(0.0)
        t0 = ret[0]
        prevt = t0
        self.history = []
        self.acc_hist = []
        try:
            while True:
                print(f'\rtime taken: {ret[0]-prevt:.3f}s', end='')
                prevt = ret[0]
                ret = self._write_read(acc:=controller(ret[0]-t0, ret[1:]))
                self.history.append(ret)
                self.acc_hist.append(acc)
        except:
            return
        finally:
            self.ser.write([5])
            self.ser.flush()
    def _write_read(self, acc: float):
        """set cart acceleration to `acc` meters per second^2 and get the system state.

        Args:
            acc (float): acceleration, in m/s^2

        Returns:
            tuple: (time (us), x, xdot, theta1, theta1 dot, theta2, theta2 dot)
        """
        # print(acc)
        self.ser.write(bytes([0]) + struct.pack(">f", acc*1000))
        self.ser.flush()
        res = np.array(struct.unpack("<Lffffff", self.ser.read(7*4)))
        res[0] /= 1e6
        res[1] /= 1000.0
        res[2] /= 1000.0
        # sleep(0.001)
        # print(res)
        return res

class LinearActuator(Node):
    def __init__(self, ser: serial.Serial):
        super().__init__('LinearActuator')
        self.hw = HardwareInterface(None, ser)
        self.timer = self.create_timer(1/300, self.update_hardware)
        self.state_pub = self.create_publisher(State, '/truth/state', 1)
        self.control_pub = self.create_publisher(AckermannDriveStamped, 'cmd', 1)
        self.state = State()
        self.state.x = 0.0
        self.state.velocity = 0.0
        self.d_accel = 0.3
        self.x_max = 1.5
        self.acc = 5.0
        self.done = False
        self.history = []
        self.hw.ser.write(bytes([1]) + struct.pack(">f", 750))
        print("DONE MOVING")
        self.time = perf_counter()+5

        
    def update_hardware(self):
        t = perf_counter()-self.time
        if t<0: return
        if self.done:
            acc = 0.0
        elif self.state.x<self.d_accel:
            acc = self.acc
        elif self.state.x>(self.x_max-self.d_accel):
            acc = -self.acc
        else:
            acc = 0.0

        res = self.hw._write_read(-acc)
        if not self.done:
            print(acc, res)
        self.history.append([-res[1], -res[2], -acc])
        # print(self.state.x)
        self.state.x = -res[1]+0.75
        self.state.velocity = -res[2]
        if self.state.x>self.x_max or self.state.x<-0.001 or self.state.velocity<0.0:
            self.hw.ser.write([5])
            self.hw.ser.flush()
            self.done = True
        self.state_pub.publish(self.state)
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.acceleration = acc 
        msg.drive.steering_angle = 0.0
        self.control_pub.publish(msg)

        
    

def main(args=None):
    try:
        rclpy.init(args=args)
        teensy_node = LinearActuator(serial.Serial('/dev/serial/by-id/usb-Teensyduino_USB_Serial_15749420-if00', baudrate=115200))
        rclpy.spin(teensy_node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        teensy_node.hw.ser.write(bytes([1]) + struct.pack(">f", 750))
        hist = np.array(teensy_node.history)
        plt.plot(hist.T[0])
        plt.plot(hist.T[1])
        plt.plot(hist.T[2])
        plt.show()
    finally:
        teensy_node.hw.ser.close()