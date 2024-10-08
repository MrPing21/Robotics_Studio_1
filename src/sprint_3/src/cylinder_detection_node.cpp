#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>
#include <cmath>
 
/**
* @brief Node for detecting cylindrical objects using LaserScan data.
*/
class CylinderDetectionNode : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Cylinder Detection Node object.
     */
    CylinderDetectionNode()
    : Node("cylinder_detection_node")
    {
        // Subscriber to LaserScan data
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&CylinderDetectionNode::laser_callback, this, std::placeholders::_1));
    }
 
private:
    struct Point {
        float x; ///< X coordinate of the point
        float y; ///< Y coordinate of the point
    };
 
    /**
     * @brief Callback function to process LaserScan messages.
     * 
     * @param msg The received LaserScan message.
     */
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Vector to store points
        std::vector<Point> points;
        int num_points = msg->ranges.size();
        // Loop through laser scan data and convert to Cartesian coordinates
        for (int i = 0; i < num_points; ++i)
        {
            float angle = msg->angle_min + i * msg->angle_increment;
            float distance = msg->ranges[i];
 
            // Ignore invalid points (infinity or NaN)
            if (std::isinf(distance) || std::isnan(distance)) continue;
 
            // Convert polar coordinates to Cartesian
            Point point;
            point.x = distance * cos(angle);  // x-coordinate
            point.y = distance * sin(angle);  // y-coordinate
 
            points.push_back(point);
        }
 
        // Cluster the points to find groups that might represent an object
        auto clusters = cluster_points(points);
 
        // Check each cluster for cylindrical shape
        for (const auto& cluster : clusters)
        {
            if (is_cylindrical(cluster))
            {
                // Calculate and log the midpoint of the cylinder
                Point midpoint = calculate_midpoint(cluster);
                RCLCPP_INFO(this->get_logger(), "Cylinder detected! Midpoint (x = %.2f, y = %.2f)", midpoint.x, midpoint.y);
            }
            // else
            // {
            //     RCLCPP_WARN(this->get_logger(), "No cylinder detected.");
            // }
        }
    }
 
    /**
     * @brief Helper function to cluster points based on proximity.
     * 
     * @param points The points to cluster.
     * @return std::vector<std::vector<Point>> A vector of clusters, where each cluster is a vector of points.
     */
    std::vector<std::vector<Point>> cluster_points(const std::vector<Point>& points)
    {
        std::vector<std::vector<Point>> clusters;
        std::vector<Point> current_cluster;
 
        float distance_threshold = 0.05;  ///< Adjust this based on the scale of your laser scan data
 
        for (size_t i = 1; i < points.size(); ++i)
        {
            // Calculate distance between consecutive points
            float distance = std::sqrt(std::pow(points[i].x - points[i - 1].x, 2) +
                                       std::pow(points[i].y - points[i - 1].y, 2));
 
            if (distance < distance_threshold)
            {
                current_cluster.push_back(points[i]);
            }
            else
            {
                if (!current_cluster.empty())
                {
                    clusters.push_back(current_cluster);
                    current_cluster.clear();
                }
            }
        }
 
        if (!current_cluster.empty())
        {
            clusters.push_back(current_cluster);
        }
 
        return clusters;
    }
 
    /**
     * @brief Helper function to check if a cluster forms a cylindrical shape (a circular arc).
     * 
     * @param cluster The cluster of points to check.
     * @return true If the cluster forms a cylindrical shape.
     * @return false If the cluster does not form a cylindrical shape.
     */
    bool is_cylindrical(const std::vector<Point>& cluster)
    {
        if (cluster.size() < 3) return false;  // Too few points to form a cylinder
 
        // Calculate the centroid of the cluster
        Point centroid{0.0, 0.0};
        for (const auto& point : cluster)
        {
            centroid.x += point.x;
            centroid.y += point.y;
        }
        centroid.x /= cluster.size();
        centroid.y /= cluster.size();
 
        // Calculate the average radius
        float avg_radius = 0.0;
        for (const auto& point : cluster)
        {
            float distance = std::sqrt(std::pow(point.x - centroid.x, 2) + std::pow(point.y - centroid.y, 2));
            avg_radius += distance;
        }
        avg_radius /= cluster.size();
 
        // Check the variance of the distances from the centroid
        float variance = 0.0;
        for (const auto& point : cluster)
        {
            float distance = std::sqrt(std::pow(point.x - centroid.x, 2) + std::pow(point.y - centroid.y, 2));
            variance += std::pow(distance - avg_radius, 2);
        }
        variance /= cluster.size();
 
        // Set a threshold for the variance to ensure the points are roughly circular
        const float variance_threshold = 0.02;
 
        // Updated variance threshold and cylinder size
        const float min_cylinder_radius = 0.01; ///< Minimum cylinder radius (10 cm)
        const float max_cylinder_radius = 0.15; ///< Maximum cylinder radius (15 cm)

 
        return (avg_radius >= min_cylinder_radius && avg_radius <= max_cylinder_radius) && (variance < variance_threshold);
    }
 
    /**
     * @brief Helper function to calculate the midpoint of a cluster.
     * 
     * @param cluster The cluster of points.
     * @return Point The midpoint of the cluster.
     */
    Point calculate_midpoint(const std::vector<Point>& cluster)
    {
        Point midpoint{0.0, 0.0};
        for (const auto& point : cluster)
        {
            midpoint.x += point.x;
            midpoint.y += point.y;
        }
 
        midpoint.x /= cluster.size();
        midpoint.y /= cluster.size();
 
        return midpoint;
    }
 
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_; ///< Subscriber for LaserScan messages
};
 
/**
* @brief The main function for the Cylinder Detection Node.
* 
* @param argc The number of command-line arguments.
* @param argv The command-line arguments.
* @return int Exit status.
*/
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CylinderDetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}