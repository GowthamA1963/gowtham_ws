// reading_laser.cpp
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class LaserFilterNode : public rclcpp::Node
{
public:
  LaserFilterNode() : Node("reading_laser")
  {
    // Subscriber: listen to the original LaserScan messages on /scan
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&LaserFilterNode::scan_callback, this, _1));

    // Publisher: publish the filtered LaserScan messages on /filtered_scan
    filtered_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/filtered_scan", 10);

    RCLCPP_INFO(this->get_logger(), "LaserFilterNode has been started.");
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    auto filtered_msg = sensor_msgs::msg::LaserScan();
    filtered_msg.header = msg->header;
    
    // Set the new field of view from 0 to 120 degrees (0 rad to 2.094 rad)
    // Filter only values within this range.
    double desired_angle_min = 0.0;
    double desired_angle_max = 2.094;  // 120 degrees in radians

    // We need to calculate the start and end indices of the original scan that are within the desired range.
    int start_index = 0;
    int end_index = msg->ranges.size() - 1;
    
    // Compute start_index: first index where angle >= desired_angle_min.
    double current_angle = msg->angle_min;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      if (current_angle >= desired_angle_min) {
        start_index = i;
        break;
      }
      current_angle += msg->angle_increment;
    }

    // Compute end_index: last index where angle <= desired_angle_max.
    current_angle = msg->angle_min;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      if (current_angle > desired_angle_max) {
        end_index = i - 1;
        break;
      }
      current_angle += msg->angle_increment;
    }
    
    if (end_index < start_index) {
      RCLCPP_WARN(this->get_logger(), "No laser scan data within the desired angle range.");
      return;
    }

    // Set the filtered message parameters.
    filtered_msg.angle_min = desired_angle_min;
    filtered_msg.angle_max = desired_angle_max;
    filtered_msg.angle_increment = msg->angle_increment;
    filtered_msg.time_increment = msg->time_increment;
    filtered_msg.scan_time = msg->scan_time;
    filtered_msg.range_min = msg->range_min;
    filtered_msg.range_max = msg->range_max;

    // Copy only the relevant ranges.
    for (int i = start_index; i <= end_index; ++i) {
      filtered_msg.ranges.push_back(msg->ranges[i]);
    }
    // Optionally, copy intensities if available.
    if (!msg->intensities.empty()) {
      for (int i = start_index; i <= end_index; ++i) {
        filtered_msg.intensities.push_back(msg->intensities[i]);
      }
    }

    filtered_pub_->publish(filtered_msg);
    RCLCPP_INFO(this->get_logger(), "Published filtered scan: %lu data points", filtered_msg.ranges.size());
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr filtered_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LaserFilterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
