#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>
#include <cmath>

class CylinderDetectionNode : public rclcpp::Node
{
public:
    CylinderDetectionNode()
    : Node("cylinder_detection_node")
    {
        // Subscriber to LaserScan data
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&CylinderDetectionNode::laser_callback, this, std::placeholders::_1));
    }

private:
    struct Point {
        float x;
        float y;
    };

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

    // Helper function to cluster points based on proximity
    std::vector<std::vector<Point>> cluster_points(const std::vector<Point>& points)
    {
        std::vector<std::vector<Point>> clusters;
        std::vector<Point> current_cluster;

        float distance_threshold = 0.1;  // Adjust this based on the scale of your laser scan data

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

    // Helper function to check if a cluster forms a cylindrical shape (a circular arc)
    
    bool is_cylindrical(const std::vector<Point>& cluster)
    {
        if (cluster.size() < 5) return false;  // Too few points to form a cylinder

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
        const float variance_threshold = 0.01;  // Adjust this threshold based on your data

        // For a 30 cm diameter cylinder, radius should be around 0.15 meters
        const float cylinder_radius = 0.15;
        const float tolerance = 0.05;  // Allow for some deviation

        return (std::fabs(avg_radius - cylinder_radius) < tolerance) && (variance < variance_threshold);
    }

    // Helper function to calculate the midpoint of a cluster
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

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CylinderDetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
