GraphSLAM!
=========================
Welcome to our GraphSLAM package! For those not familiar, `SLAM <https://youtu.be/saVZtgPyyJQ?si=ut-W3i7aFHYHAFhw>`_ stands for Simultaneous Localization and Mapping. This means, we're both figuring out our current position, while also making a global map of our surroundings (in this case, cones). To do this, we have to subscribe to a few different topics: 


Subscribers
-------------

``/imu``
    :Type: ``sensor_msgs/Imu``
    :Description: Recieves IMU messages about acceleration and orientation
    :Callback Function: ``imu_callback``

``/fusion/cones``
    :Type: ``eufs_msgs/ConeArrayWithCovariance``
    :Description: Recieves all the cones spottable by perception 
    :Callback Function: ``cones_callback``
``/ground_truth/state``
    :Type: ``eufs_msgs/CarState``
    :Description: Gives car position information. Used to convert cones into polar
    :Callback Function: ``state_sub``

Once again, we return out the Cars position, and a map of the cones:

Publishers
------------

``/slam/state``
    :Type: ``feb_msgs/State``
    :Description: Publishes the cars current state as percieved by SLAM

``/slam/map/local``
    :Type: ``feb_msgs/Map``
    :Description: Publishes the current local map

``/slam/map/global``
    :Type: ``feb_msgs/Map``
    :Description: Publishes the Global map of cones


Note: We also added a few extra publishers for PointCloud messages,
so we could compare the percieved position of SLAM to the actual position in the simulator in real-time.

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


