# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import board
import adafruit_bno055
import rclpy
from rclpy.node import Node
from senors_msgs.msg import Imu
from geometry_msgs.msg import Vector3




i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
sensor = adafruit_bno055.BNO055_I2C(i2c)

# If you are going to use UART uncomment these lines
# uart = board.UART()
# sensor = adafruit_bno055.BNO055_UART(uart)

last_val = 0xFFFF


def temperature():
    global last_val  # pylint: disable=global-statement
    result = sensor.temperature
    if abs(result - last_val) == 128:
        result = sensor.temperature
        if abs(result - last_val) == 128:
            return 0b00111111 & result
    last_val = result
    return result

class ImuNode(Node):
    def __init__(self):
        super().__init__("ImuNode")

        #Publisher
        self.imu_publisher = self.create_publisher(Imu, '/imu', 1)

imuNode = ImuNode()
while True:
    print("Temperature: {} degrees C".format(sensor.temperature))
    """
    print(
        "Temperature: {} degrees C".format(temperature())
    )  # Uncomment if using a Raspberry Pi
    """
    print("Accelerometer (m/s^2): {}".format(sensor.acceleration))
    print("Linear acceleration (m/s^2): {}".format(sensor.linear_acceleration))
    
    msg = Imu()
    vector = Vector3()
    cur_acc = sensor.linear_acceleration
    vector.x = cur_acc[0]
    vector.y = cur_acc[1]
    vector.z = cur_acc[2]
    msg.linear_acceleration = vector
    imuNode.imu_publisher.publish(msg)




