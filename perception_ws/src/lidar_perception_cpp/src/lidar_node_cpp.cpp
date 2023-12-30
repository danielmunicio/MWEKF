#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

using std::placeholders::_1;

class LidarNode : public rclcpp::Node {
  public:
    LidarNode() : Node("lidar_node_cpp") {
      // SUBSCRIBERS
      pc_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/rslidar_points", 1, std::bind(&LidarNode::lidar_callback, this, _1));

      // PUBLISHERS
      z_filtered_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "perception/lidar/filtered_cloud", 1);
    }

  private:
    // GLOBAL VARIABLES
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr z_filtered_pub;
    const float MAX_RANGE = 15.0;
    const float LOWER_Z_THRESHOLD = -1.f;
    const float UPPER_Z_THRESHOLD = 0.f;
    const size_t NUM_BINS = 10;
    const size_t NUM_Z_VALS = 15;
    // Create linspace for z-values and expspace range bins
    const std::vector<float> z_vals = linspace(LOWER_Z_THRESHOLD, UPPER_Z_THRESHOLD, NUM_Z_VALS);
    const std::vector<float> r_vals = expspace(.1f, MAX_RANGE, NUM_BINS);

    /**
     * Creates `num + 1` (!!) linearly spaced points in the interval `[a, b]`
     * 
     * @param a start of the interval
     * @param b end of the interval
     * @param num number of points (minus one)
     * @return vector of linearly spaced values between `a` and `b`
     */
    std::vector<float> linspace(float a, float b, int num) const {
      std::vector<float> vec;
      float step = (b - a) / num;

      for (int i = 0; i <= num; ++i) {
        vec.push_back(a + i * step);
      }

      return vec;
    }

    /**
     * Creates `num + 1` (!!) exponentially spaced points in the interval `[a, b]`. Exponential spacing
     * entails that the points are `a, a*r, a*r^2, ..., a*r^num = b` for some appropriate `r`.
     * 
     * @param a start of the interval
     * @param b end of the interval
     * @param num number of points (minus one)
     * @return vector of exponentially spaced values between `a` and `b`
     */
    std::vector<float> expspace(float a, float b, int num) const {
      std::vector<float> vec;
      float base = std::pow(b - a + 1.f, 1.f / num);

      for (int i = 0; i <= num; ++i) {
        vec.push_back(std::pow(base, i) + a - 1.f);
      }

      return vec;
    }

    void lidar_callback(const sensor_msgs::msg::PointCloud2 & msg) const {
      // Time the process
      auto start_time = std::chrono::high_resolution_clock::now();

      // Replace intensity field with range field
      sensor_msgs::msg::PointField range_field;
      range_field.name = "range";
      range_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
      range_field.count = 1; // Only one datapoint
      range_field.offset = 12; // x (0), y (4), z (8), r (12) (4 bytes each)
      auto new_msg_fields = msg.fields;
      new_msg_fields[3] = range_field; // Replace the intensity field with range field

      // New message same as old, but with different fields
      auto filtered_cloud_msg = msg;
      filtered_cloud_msg.fields = new_msg_fields;
      filtered_cloud_msg.is_dense = false; // We will use nans for invalid points

      // Create iterators for input and output point clouds
      sensor_msgs::PointCloud2ConstIterator<float> input_it_x(msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> input_it_y(msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> input_it_z(msg, "z");
      sensor_msgs::PointCloud2Iterator<float> pre_it_r(filtered_cloud_msg, "range");
      sensor_msgs::PointCloud2Iterator<float> pre_it_z(filtered_cloud_msg, "z");

      // Iterate through each point in the cloud
      for (; input_it_x != input_it_x.end(); ++input_it_x, ++input_it_y, ++input_it_z, ++pre_it_z, ++pre_it_r) {
        float x = *input_it_x;
        float y = *input_it_y;
        float z = *input_it_z;
        // Skip if already invalid
        if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
          *pre_it_r = -1.f; // Invalid point <=> range set to -1
          continue;
        }

        // Calculate range
        float range = std::sqrt(x * x + y * y + z * z);
        // Set range to -1 and z to nan if invalid
        if (range > MAX_RANGE || z > UPPER_Z_THRESHOLD) {
          *pre_it_r = -1.f;
          *pre_it_z = std::numeric_limits<float>::quiet_NaN();
        } else { // Otherwise insert range value
          *pre_it_r = range;
        }
      }
      // Track which bin each point belongs to (bin nr. NUM_BINS reserved for invalid points)
      std::vector<std::vector<size_t>> bin_map(msg.width, std::vector<size_t>(msg.height, NUM_BINS));
      // Track the z-value of the ground in each bin
      std::vector<std::vector<float>> ground_z(msg.width, std::vector<float>(NUM_BINS));
      
      // Compute point counts for each bin
      for (uint32_t channel = 0; channel < msg.width; ++channel) { // Note: each channel has NUM_BINS # of bins
        // Point count f(z) = *number of points in bin with z-value LESS THAN z*
        std::vector<std::vector<int>> point_counts(NUM_BINS, std::vector<int>(NUM_Z_VALS + 1, 0));
        // Set iterators to correct channel
        sensor_msgs::PointCloud2Iterator<float> iter_r(filtered_cloud_msg, "range");
        sensor_msgs::PointCloud2Iterator<float> iter_z(filtered_cloud_msg, "z");
        if (channel > 0) {// Don't do += 0, really screws things up
          iter_r += channel;
          iter_z += channel;
        }
        // Iterate over points in channel
        for (uint32_t p = 0; p < msg.height; ++p) {
          float range = *iter_r;
          float z = *iter_z;
          iter_r += msg.width; // Will go out of bounds, but is never used when it does
          iter_z += msg.width;
          // Skip if point invalid
          if (std::isnan(z)) continue;
          // Determine which bin the point belongs to
          size_t bin_counter = 0;
          while (bin_counter < NUM_BINS - 1 && range > r_vals[bin_counter + 1]) {
            bin_counter++;
          }
          bin_map[channel][p] = bin_counter;
          // Increment point counts for the corresponding bin (from large to small z)
          size_t i = NUM_Z_VALS;
          auto it = point_counts[bin_counter].end();
          while (i > 0 && z < z_vals[i]) {
            ++*it; // Increment count
            --it; // Decrement iterator
            --i; // Decrement index
          }
        }
        // Compute ground z-value for each bin
        size_t ground_ind = NUM_Z_VALS;
        auto ground_it = ground_z[channel].begin();
        // Iterate over bin point counts
        for (auto count : point_counts) {
          int max_diff = std::numeric_limits<int>::min();
          // Iterate over z-values to find index where maximum change occurs
          for (size_t z_ind = 1; z_ind < NUM_Z_VALS + 1; ++z_ind) {
            int diff = count[z_ind] - count[z_ind-1];
            if (diff > max_diff) {
              max_diff = diff;
              ground_ind = z_ind;
            }
          }
          *ground_it = z_vals[ground_ind];
          ++ground_it;
        }
      }
      // Remove the ground
      sensor_msgs::PointCloud2Iterator<float> output_it_r(filtered_cloud_msg, "range");
      sensor_msgs::PointCloud2Iterator<float> output_it_z(filtered_cloud_msg, "z");
      int channel, point;
      for (int i = 0; output_it_z != output_it_z.end(); ++i, ++output_it_r, ++output_it_z) {
        float z = *output_it_z;
        if (std::isnan(z)) continue; // Skip if already invalid
        // Channel = column nr, point = row nr
        channel = i % msg.width;
        point = i / msg.width;
        auto bin = bin_map[channel][point];
       if (z < ground_z[channel][bin]) { // All z-values below ground are removed
          *output_it_r = -1.f;
          *output_it_z = std::numeric_limits<float>::quiet_NaN();
        }
      }
 
      // Publish the filtered point cloud with fresh timestamp
      filtered_cloud_msg.header.stamp = this->now();
      z_filtered_pub->publish(filtered_cloud_msg);

      // Log execution time
      auto end_time = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
      RCLCPP_INFO(get_logger(), "Filtering time: %f s.", duration.count() * 1E-6);
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarNode>());
  rclcpp::shutdown();
  return 0;
}
