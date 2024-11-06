cmake_minimum_required(VERSION 3.5)
project(lidar_camera_calibration)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(cv_bridge REQUIRED)

# Add include directories
include_directories(
  ${ament_INCLUDE_DIRS}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(FILES utilities/segment_bounding_box_cones.pt
  DESTINATION share/${PROJECT_NAME}/files
)

install(
  PROGRAMS
    scripts/calibrate_camera_lidar.py
    scripts/file_operations.py
    scripts/model_operations.py
    scripts/realsense_camera_only.py
    scripts/point_3d_extractor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


