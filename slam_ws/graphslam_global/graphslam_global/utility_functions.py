import math
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Vector3
from numpy import pi 

def compute_timediff(slam, header: Header) -> float:
    """
    Function that takes in message header and computes difference in time from last state msg
    Input: Header (std_msg/Header)
    - uint32 seq
    - time stamp
    - string frame_id
    Output: timediff: float
    """
    newtime = header.stamp.sec + 1e-9 * header.stamp.nanosec
    timediff = newtime - slam.statetimestamp
    slam.statetimestamp = newtime

    return timediff
    

def quat_to_euler(quat):
    """
    Function that takes in quaternion and converts to Eulerian angles
    Input: Quat (Quaternion)
    - float x
    - float y
    - float z
    - float w
    Output: roll, pitch, yaw
    """
    x = quat.x
    y = quat.y
    z = quat.z
    w = quat.w

    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * x))
    pitch = math.asin(2.0 * (w * y - z * x))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    return yaw

def cartesian_to_polar(car_state, cone):
    p_x = cone[0] - car_state[0]
    p_y = cone[1] - car_state[1]
    r = math.sqrt(p_x**2 + p_y**2)
    angle = math.atan2(p_y, p_x)

    return r, angle

def compareAngle(self, a, b, threshold): # a<b
    mn = min(b-a, 2*pi - b + a) # (ex. in degrees): a = 15 and b = 330 are 45 degrees apart (not 315)

    return mn < threshold

def compute_delta_velocity(self, acc: Vector3, dt: float, imu_direction: str) -> float:
    """
    Function that takes in linear acceleration and dt and outputs velocity (linear)
    Input:
    - linear_acceleration: from imu message
    - dt: calculated at each time step
    Output:
    - linear_velocity
    #NOTE: we assume linear acceleration is in the car's frame. This means
            x is in the longitudinal direction (positive = forwards)
            y is in the lateral direction (positive = to the right)
            depending on how IMU is providing this data, change accordingly
    """
        # longitudinal_acc = np.linalg.norm([acc.x, acc.y])
    if imu_direction == 'x':
        longitudinal_acc = acc.x
    if imu_direction == 'y':
        longitudinal_acc = acc.y
    if imu_direction == 'z':
        longitudinal_acc = acc.z
    # lateral_acc = acc.y # May be needed in the future if  
    #                       straightline model is not accurate enough
    linear_velocity = longitudinal_acc * dt
    return linear_velocity