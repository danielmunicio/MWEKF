<<<<<<< HEAD
=======
# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
>>>>>>> e57b6131526867257156bfc7d5ca599c0f043a79

import time
import board
import adafruit_bno055
import rclpy
from rclpy.node import Node
<<<<<<< HEAD
from sensor_msgs.msg import Imu 
from geometry_msgs.msg import Vector3 
from geometry_msgs.msg import Quaternion
=======
from senors_msgs.msg import Imu
from geometry_msgs.msg import Vector3




i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
sensor = adafruit_bno055.BNO055_I2C(i2c)

# If you are going to use UART uncomment these lines
# uart = board.UART()
# sensor = adafruit_bno055.BNO055_UART(uart)

last_val = 0xFFFF

>>>>>>> e57b6131526867257156bfc7d5ca599c0f043a79

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
<<<<<<< HEAD
        
        self.imu_main()

    def imu_main(self):
        i2c = board.I2C()  # uses board.SCL and board.SDA
        # i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
        sensor = adafruit_bno055.BNO055_I2C(i2c)

        last_val = 0xFFFF

        while True:
            print("Linear acceleration (m/s^2): {}".format(sensor.linear_acceleration))
            print("Orientation: {}".format(sensor.euler))
            msg = Imu()
            vector = Vector3()
            cur_acc = sensor.linear_acceleration
            vector.x = cur_acc[0]
            vector.y = cur_acc[1]
            vector.z = cur_acc[2]
            msg.linear_acceleration = vector
            #msg.orientation.x = sensor.euler[0]
            #msg.orientation.y = sensor.euler[1]
            #msg.orientation.z = sensor.euler[2]
            orientation = Quaternion()
            orientation.x = sensor.euler[0]
            orientation.y = sensor.euler[1]
            orientation.z = sensor.euler[2]
            # orientation.w = sensor.quaternion[3]
            msg.header.stamp.sec, msg.header.stamp.nanosec = self.get_clock().now().seconds_nanoseconds()
            msg.orientation = orientation
            self.imu_publisher.publish(msg)

def main(args=None): 
    rclpy.init(args=args)
    imuNode = ImuNode()
    rclpy.spin(imuNode)
    imuNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
=======

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



    time.sleep(0.1)
>>>>>>> e57b6131526867257156bfc7d5ca599c0f043a79

