GraphSLAM!
=========================
Welcome to our GraphSLAM package! For those not familiar, `SLAM <https://youtu.be/saVZtgPyyJQ?si=ut-W3i7aFHYHAFhw>`_ stands for Simultaneous Localization and Mapping. This means, we're both figuring out our current position, while also making a global map of our surroundings (in this case, cones). To do this, we have to subscribe to a few different topics: 

Subscribers
-----------

Topic: **/imu**

- **Type**: :ref:`sensor_msgs/Imu <sensor_msgs-IMU>`
- **Description**: Receives IMU messages about acceleration and orientation  
- **Callback Function**: ``imu_callback``  

Topic: **/fusion/cones**

- **Type**: :ref:`eufs_msgs/ConeArrayWithCovariance <eufs_msgs/ConeArrayWithCovariance>`
- **Description**: Receives all the cones spottable by perception  
- **Callback Function**: ``cones_callback``  

Topic: **/ground_truth/state**

- **Type**: :ref:`eufs_msgs/CarState <eufs_msgs/CarState>`
- **Description**: Gives car position information. Used to convert cones into polar  
- **Callback Function**: ``state_sub``  

Publishers
----------

Topic: **/slam/state**

- **Type**: :ref:`feb_msgs/State <feb_msgs-state-label>`
- **Description**: Publishes the car's current state as perceived by SLAM  

Topic: **/slam/map/local**

- **Type**: :ref:`feb_msgs/Map <feb_msgs-map-label>`
- **Description**: Publishes the current local map  

Topic: **/slam/map/global**

- **Type**: :ref:`feb_msgs/Map <feb_msgs-map-label>`
- **Description**: Publishes the Global map of cones

Information Collection and Initial Guess
--------------------------------------------------
First, we have to collect all the information from the sensors, and gather the 
initial guess for where the car is in space. 

.. automodule:: graphslam_global.graphslam_global_fast
   :members:
   :undoc-members:
   :show-inheritance:


Graph Solver
--------------------------------------
Once we have all the necessary information processed, we should solve the matrix, 
to find the most optimal guess of the car and cones positions

.. automodule:: graphslam_global.GraphSLAMFast
   :members:
   :undoc-members:
   :show-inheritance:


