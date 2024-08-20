#include <functional>
#include <memory>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class week2 : public rclcpp::Node
{
public:
  week2()
  : Node("minimal_subscriber"), count_(0)
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&week2::topic_callback, this, _1));
    
    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/sensor_msgs/msg/scan_modified", 10);
  }

private:
  void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    count_++;

    // RCLCPP_INFO(this->get_logger(), "Angle_min: %f radians", msg->angle_min);
    // RCLCPP_INFO(this->get_logger(), "Angle_max: %f radians", msg->angle_max);
    // RCLCPP_INFO(this->get_logger(), "Angle_increment: %f radians", msg->angle_increment);


     if (count_ % 4 == 0) {
      // Create a new ranges array to store every 4th scan
      std::vector<float> new_ranges;
      for (size_t i = 0; i < msg->ranges.size(); i += 4) {
        new_ranges.push_back(msg->ranges[i]);
      }

      // Adjust the angle_increment based on the reduction of data
      float new_angle_increment = msg->angle_increment * 4;

      // Create a new LaserScan message to publish
      auto modified_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
      *modified_msg = *msg;  // Copy original message

      // Modify the relevant fields
      modified_msg->ranges = new_ranges;
      modified_msg->angle_increment = new_angle_increment;

      publisher_->publish(*modified_msg);
      RCLCPP_INFO(this->get_logger(), "Published a modified message with every 4th range.");
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  int count_;  // Counter to track the number of messages received
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<week2>());
  rclcpp::shutdown();
  return 0;
}
