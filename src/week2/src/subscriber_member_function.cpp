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
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&week2::topic_callback, this, _1));
    
    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/sensor_msgs/msg/scan_modified", 10);
  }

private:
  void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    

    RCLCPP_INFO(this->get_logger(), "Angle_min: %f radians", msg->angle_min);
    RCLCPP_INFO(this->get_logger(), "Angle_max: %f radians", msg->angle_max);
    RCLCPP_INFO(this->get_logger(), "Angle_increment: %f radians", msg->angle_increment);


     // Create a new LaserScan message
    
    int n = 4;
    auto msg_copy = *msg;
    sensor_msgs::msg::LaserScan new_msg;
 
    // Copy necessary fields
    new_msg.header = msg_copy.header;
    new_msg.angle_min = msg_copy.angle_min;
    new_msg.angle_max = msg_copy.angle_max;
    new_msg.angle_increment = msg_copy.angle_increment * n;  // cause we're taking 1 in 3 readings
    new_msg.time_increment = msg_copy.time_increment;
    new_msg.scan_time = msg_copy.scan_time;
    new_msg.range_min = msg_copy.range_min;
    new_msg.range_max = msg_copy.range_max;
 

    // Create a new ranges array to store every 4th scan
    std::vector<float> new_ranges;
    for (size_t i = 0; i < msg->ranges.size(); i += n) {
      new_ranges.push_back(msg->ranges[i]);
    }

  
    new_msg.ranges = new_ranges;

  
    publisher_->publish(new_msg);

    RCLCPP_INFO(this->get_logger(), "Published a modified message with every 4th range.");
  }
  

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<week2>());
  rclcpp::shutdown();
  return 0;
}
