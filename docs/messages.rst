ROS Message Types
=========================
ROS is the structured concurrency program that we use to control all the different proccesses happening in the car. All the message types that we commonly use in our pipeline are documented here for your convenience.


.. _feb_msgs-state-label:

State Message
--------------

- **header** (*std_msgs/Header*): The standard ROS message header.
- **x** (*float64*): X-coordinate of the car.
- **y** (*float64*): Y-coordinate of the car.
- **velocity** (*float64*): Current velocity of the car.
- **heading** (*float64*): Heading or orientation of the car.
- **lap_count** (*uint8*): Current lap count of the car.

.. _feb_msgs-cone-label:

Cone Message
--------------

- **header** (*std_msgs/Header*): The standard ROS message header.
- **r** (*float64[]*): Array of radial distances.
- **theta** (*float64[]*): Array of angular positions (in radians).
- **color** (*int8[]*): Array of color codes for the detected objects.

.. _feb_msgs-map-label:

Map Message
--------------

- **header** (*std_msgs/Header*): The standard ROS message header.
- **left_cones_x** (*float64[]*): Array of X-coordinates for the left cones.
- **left_cones_y** (*float64[]*): Array of Y-coordinates for the left cones.
- **right_cones_x** (*float64[]*): Array of X-coordinates for the right cones.
- **right_cones_y** (*float64[]*): Array of Y-coordinates for the right cones.

.. _feb_msgs-path-label: 

Path Message
--------------

- **header** (*std_msgs/Header*): The standard ROS message header.
- **x** (*float64[]*): Array of X-coordinates.
- **y** (*float64[]*): Array of Y-coordinates.
- **v** (*float64[]*): Array of velocities.
- **psi** (*float64[]*): Array of heading angles (in radians).

.. _sensor_msgs-IMU:

IMU Message 
-------------

- **header** (*std_msgs/Header*): The standard ROS message header.
- **orientation** (*geometry_msgs/Quaternion*): Orientation expressed as a quaternion.
- **orientation_covariance** (*float64[9]*): Covariance matrix of orientation (3x3).
- **angular_velocity** (*geometry_msgs/Vector3*): Angular velocity in rad/sec.
- **angular_velocity_covariance** (*float64[9]*): Covariance matrix of angular velocity (3x3).
- **linear_acceleration** (*geometry_msgs/Vector3*): Linear acceleration in m/s².
- **linear_acceleration_covariance** (*float64[9]*): Covariance matrix of linear acceleration (3x3).

.. _eufs_msgs/ConeArrayWithCovariance:

Cone Array With Covariance Message 
-----------------------------------

.. _eufs_msgs/CarState: 

Car State Message (EUFS)
-------------------------


