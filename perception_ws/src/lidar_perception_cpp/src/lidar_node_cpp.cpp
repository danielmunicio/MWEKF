#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

using std::placeholders::_1;

class LidarNode : public rclcpp::Node
{
  public:
    LidarNode()
    : Node("lidar_node_cpp")
    {
      // SUBSCRIBERS
      pc_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/rslidar_points", 1, std::bind(&LidarNode::lidar_callback, this, _1));

      // PUBLISHERS
      z_filtered_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "perception/lidar/filtered_cloud", 1);
      
      // Global variables
      MAX_RANGE = 15.0;
      MOUNT_HEIGHT = 0.6;
      Z_THRESHOLD = 0.3;
    }

  private:
    void lidar_callback(const sensor_msgs::msg::PointCloud2 & msg) const
    {
      // Time the process
      auto start_time = std::chrono::high_resolution_clock::now();

      // Replace intensity field with range field
      sensor_msgs::msg::PointField range_field;
      range_field.name = "range";
      range_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
      range_field.count = 1;
      range_field.offset = 12; // x, y, z, r (4 bytes each)
      std::vector<sensor_msgs::msg::PointField> new_msg_fields = msg.fields;
      new_msg_fields[3] = range_field;

      // New message same as old, but with different fields
      sensor_msgs::msg::PointCloud2 filtered_cloud_msg = msg;
      filtered_cloud_msg.fields = new_msg_fields;
      filtered_cloud_msg.is_dense = false;

      // Create iterators for input and output point clouds
      sensor_msgs::PointCloud2ConstIterator<float> input_it_x(msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> input_it_y(msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> input_it_z(msg, "z");
      sensor_msgs::PointCloud2Iterator<float> output_it_r(filtered_cloud_msg, "range");
      sensor_msgs::PointCloud2Iterator<float> output_it_z(filtered_cloud_msg, "z");

      // Iterate through each point in the cloud
      float x, y, z, range;
      for (uint32_t row = 0; row < msg.height; ++row)
      {
        for (uint32_t col = 0; col < msg.width; ++col)
        {
          // Calculate range
          x = *input_it_x;
          y = *input_it_y;
          z = *input_it_z;
          range = std::sqrt(x * x + y * y + z * z);

          // Check if point is valid
          if (z < Z_THRESHOLD - MOUNT_HEIGHT && range < MAX_RANGE)
          {
            // If true, assign true range value
            *output_it_r = range;
          } 
          else 
          {
            // If false, set range and z-value (to not appear in rViz) to NaN
            *output_it_r = std::numeric_limits<float>::quiet_NaN();
            *output_it_z = *output_it_r;
          }
          
          // Move the iterators to the next point
          ++input_it_x;
          ++input_it_y;
          ++input_it_z;
          ++output_it_r;
          ++output_it_z;
        }
      }

      // Publish the filtered point cloud with fresh timestamp
      filtered_cloud_msg.header.stamp = this->now();
      z_filtered_pub->publish(filtered_cloud_msg);

      auto end_time = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

      RCLCPP_INFO(get_logger(), "Filtering time: %f s.", duration.count() * 1E-6);
    }

    // Variables declared here
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr z_filtered_pub;
    float MAX_RANGE, MOUNT_HEIGHT, Z_THRESHOLD;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarNode>());
  rclcpp::shutdown();
  return 0;
}
