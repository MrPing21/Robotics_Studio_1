#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

class MapServer : public rclcpp::Node {
public:
    double map_x = -7;
    double map_y = -7;
    double res = 0.01;

    MapServer() : Node("map_server") {
        occupancy_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("occ_map", 10);
        raw_map_pub_ = this->create_publisher<sensor_msgs::msg::Image>("raw_map", 10);
        map_publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&MapServer::publish_map, this));
        load_map();
    }

private:
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_map_pub_;
    rclcpp::TimerBase::SharedPtr map_publish_timer_;
    cv::Mat map_image_;
    nav_msgs::msg::OccupancyGrid occupancy_map_;

    void load_map() {
        std::string package_name = "sprint_3";
        std::string package_share_directory = ament_index_cpp::get_package_share_directory(package_name);
        std::string map_path = package_share_directory + "/data/map.pgm";
        map_image_ = cv::imread(map_path, cv::IMREAD_GRAYSCALE);
        if (map_image_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load map image from %s", map_path.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Successfully loaded map from %s", map_path.c_str());
            create_occupancy_map();
        }
    }

    void create_occupancy_map() {
        occupancy_map_.header.frame_id = "map";
        occupancy_map_.info.resolution = res;
        occupancy_map_.info.width = map_image_.cols;
        occupancy_map_.info.height = map_image_.rows;
        occupancy_map_.info.origin.position.x = map_x;
        occupancy_map_.info.origin.position.y = map_y;
        occupancy_map_.info.origin.position.z = 0.0;
        occupancy_map_.info.origin.orientation.w = 1.0;
        occupancy_map_.data.resize(map_image_.total());
        for (int y = 0; y < map_image_.rows; ++y) {
            for (int x = 0; x < map_image_.cols; ++x) {
                int index = y * map_image_.cols + x;
                uchar pixel = map_image_.at<uchar>(map_image_.rows - 1 - y, x);
                if (pixel == 0) {
                    occupancy_map_.data[index] = 100;
                } else if (pixel == 255) {
                    occupancy_map_.data[index] = 0;
                } else {
                    occupancy_map_.data[index] = -1;
                }
            }
        }
    }

    void publish_map() {
        if (!map_image_.empty()) {
            occupancy_map_.header.stamp = this->now();
            occupancy_map_pub_->publish(occupancy_map_);

            sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", map_image_).toImageMsg();
            msg->header.stamp = this->now();
            msg->header.frame_id = "map";
            raw_map_pub_->publish(*msg);
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}