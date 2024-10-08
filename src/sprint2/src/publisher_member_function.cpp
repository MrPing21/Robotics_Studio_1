#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/opencv.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>

class SimpleLocalizerNode : public rclcpp::Node
{
public:
    SimpleLocalizerNode() : Node("simple_localizer_node"), relative_orientation_(0.0)
    {
        // Subscribers
        subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&SimpleLocalizerNode::mapCallback, this, std::placeholders::_1));

        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&SimpleLocalizerNode::scanCallback, this, std::placeholders::_1));

        cv::namedWindow(WINDOW1, cv::WINDOW_AUTOSIZE);
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&SimpleLocalizerNode::odomCallback, this, std::placeholders::_1));
        pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10, std::bind(&SimpleLocalizerNode::poseCallback, this, std::placeholders::_1));

        // Publishers
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "Simple Localizer Node started.");
    }

private:
    // Callback for /initialpose to set the initial pose of the robot
    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Initial Pose Set.");
        initial_pose_ = *msg;
    }

    // Callback for the laser scan
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Purpose: This function is a callback that gets triggered whenever a new LaserScan message is received from the /scan topic.
        cv::Mat img = laserScanToMat(msg);
        // Functionality:

        //     Convert LaserScan to Image: The laserScanToMat function is called to convert the LaserScan data into an image (a cv::Mat).
        //     Handle First Image: If it's the first time this function is being called, it captures and displays the first image.
        if (!first_image_captured_)
        {
            first_image_ = img.clone();
            first_image_captured_ = true;
            // Display the first image
            cv::imshow("First Image", first_image_);
            cv::waitKey(1);
        }
        //     Handle Second Image: If it's the second time this function is called, it captures and displays the second image, then calculates the change in orientation (yaw).
        else if (!second_image_captured_)
        {
            second_image_ = img.clone();
            second_image_captured_ = true;
            // Display the first image
            cv::imshow("Second Image", second_image_);
            cv::waitKey(1);
        }
        //     Update and Rotate: For subsequent calls, it updates the images, calculates the yaw change, and logs the relative orientation.
        else
        {
            first_image_ = second_image_.clone();
            second_image_ = img.clone();
            first_image_captured_ = true;
            // Display the first image
            cv::imshow("Second Image", second_image_);
            cv::waitKey(1);

            calculateYawChange();
            relative_orientation_ = relative_orientation_ + angle_difference_;
            RCLCPP_INFO(this->get_logger(), "Relative orientation %f", relative_orientaion_);
        }
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr mapMsg)
    {
        std::cout << "mapCallback" << std::endl;

        occupancyGridToImage(mapMsg);

        cv::Mat tmp_col_img = m_MapColImage.clone();

        cv::rotate(tmp_col_img, tmp_col_img, cv::ROTATE_90_COUNTERCLOCKWISE);

        cv::imshow(WINDOW1, tmp_col_img);
        cv::waitKey(1);
    }

    void occupancyGridToImage(const nav_msgs::msg::OccupancyGrid::SharedPtr grid)
    {
        int grid_data;
        unsigned int row, col, val;

        m_temp_img = cv::Mat::zeros(grid->info.height, grid->info.width, CV_8UC1);

        std::cout << "DataParse started for map: " << grid->header.stamp.sec << " Dim: " << grid->info.height << "x" << grid->info.width << std::endl;

        for (row = 0; row < grid->info.height; row++)
        {
            for (col = 0; col < grid->info.width; col++)
            {
                grid_data = grid->data[row * grid->info.width + col];
                if (grid_data != -1)
                {
                    val = 255 - (255 * grid_data) / 100;
                    val = (val == 0) ? 255 : 0;
                    m_temp_img.at<uchar>(grid->info.height - row - 1, col) = val;
                }
                else
                {
                    m_temp_img.at<uchar>(grid->info.height - row - 1, col) = 0;
                }
            }
        }

        map_scale_ = grid->info.resolution;
        origin_x = grid->info.origin.position.x;
        origin_y = grid->info.origin.position.y;
        size_x = grid->info.width;
        size_y = grid->info.height;

        cv::Mat kernel = (cv::Mat_<uchar>(3, 3) << 0, 0, 0,
                          0, 1, 0,
                          0, 0, 0);
        cv::erode(m_temp_img, m_MapBinImage, kernel);

        m_MapColImage.create(m_MapBinImage.size(), CV_8UC3);
        cv::cvtColor(m_MapBinImage, m_MapColImage, cv::COLOR_GRAY2BGR);

        std::cout << "Occupancy grid map converted to a binary image\n";

        // // Display the image to verify
        // cv::imshow("Occupancy Grid", m_MapColImage);
        // cv::waitKey(1);

        cv::Mat edge_image;
        double lower_threshold = 50; // You can adjust these thresholds for your needs
        double upper_threshold = 150;

        cv::Canny(m_MapBinImage, edge_image, lower_threshold, upper_threshold);

        // Store the edge image as map_image_ for further processing
        map_image_ = edge_image;

        // Show the edge image for debugging purposes
        // cv::imshow("Map Edges", map_image_);
        // cv::waitKey(1);
    }

    // Callback for odometry
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Update odometry here
        odometry_ = *msg;
    }

    // Convert LaserScan to OpenCV image
    cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        int size = 500; // Image size
        float max_range = scan->range_max;
        cv::Mat image = cv::Mat::zeros(size, size, CV_8UC1);

        for (size_t i = 0; i < scan->ranges.size(); ++i)
        {
            float range = scan->ranges[i];
            if (range >= scan->range_min && range <= max_range)
            {
                float angle = scan->angle_min + i * scan->angle_increment;
                int x = static_cast<int>((range * cos(angle) / max_range) * (size / 2)) + size / 2;
                int y = static_cast<int>((range * sin(angle) / max_range) * (size / 2)) + size / 2;

                if (x >= 0 && x < size && y >= 0 && y < size)
                {
                    image.at<uchar>(y, x) = 255;
                }
            }
        }

        // Apply Canny edge detection on the laser scan image
        cv::Mat edge_image;
        double lower_threshold = 50;
        double upper_threshold = 150;
        cv::Canny(image, edge_image, lower_threshold, upper_threshold);

        // Show the laser scan edge image for debugging
        cv::imshow("Laser Scan Edges", edge_image);
        cv::waitKey(1);

        return edge_image;
    }

    // Estimate the yaw change by comparing map edges (Image B) and scan image (Image C)
    void calculateYawChange()
    {
        // Detect and match features between the first and second images
        std::vector<cv::Point2f> srcPoints, dstPoints;
        detectAndMatchFeatures(first_image_, second_image_, srcPoints, dstPoints);

        if (srcPoints.size() < 3 || dstPoints.size() < 3)
        {
            RCLCPP_ERROR(this->get_logger(), "Not enough points for affine transformation.");
            return;
        }

        try
        {
            cv::Mat transform_matrix = cv::estimateAffinePartial2D(srcPoints, dstPoints);
            if (transform_matrix.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "Transformation matrix estimation failed.");
            }
            else
            {
                // Extract the rotation angle from the transformation matrix
                angle_difference_ = atan2(transform_matrix.at<double>(1, 0), transform_matrix.at<double>(0, 0));
                angle_difference_ = angle_difference_ * 180.0 / CV_PI;
                RCLCPP_INFO(this->get_logger(), "Estimated yaw angle change: %f degrees", angle_difference_);
            }
        }
        catch (const cv::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error in estimateAffinePartial2D: %s", e.what());
        }
    }

    void detectAndMatchFeatures(const cv::Mat &img1, const cv::Mat &img2, std::vector<cv::Point2f> &srcPoints, std::vector<cv::Point2f> &dstPoints)
    {
        
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;

        orb->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
        orb->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

       
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<cv::DMatch> matches;
        matcher.match(descriptors1, descriptors2, matches);

        
        
        std::sort(matches.begin(), matches.end(), [](const cv::DMatch &a, const cv::DMatch &b)
                  { return a.distance < b.distance; });

        size_t numGoodMatches = static_cast<size_t>(matches.size() * 0.15);

        for (const auto &match : matches)
        {
            srcPoints.push_back(keypoints1[match.queryIdx].pt);
            dstPoints.push_back(keypoints2[match.trainIdx].pt);
        }
    }

    // Propagate the robot's pose using odometry
    void propagateOdometry()
    {
        // Simple integration of odometry to propagate pose
        double delta_x = odometry_.twist.twist.linear.x;
        double delta_y = odometry_.twist.twist.linear.y;

        initial_pose_.pose.pose.position.x += delta_x;
        initial_pose_.pose.pose.position.y += delta_y;

        RCLCPP_INFO(this->get_logger(), "New Pose -> x: %f, y: %f", initial_pose_.pose.pose.position.x, initial_pose_.pose.pose.position.y);
    }

    // ROS2 communication components
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;

    // State variables
    cv::Mat map_image_;
    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_;
    nav_msgs::msg::Odometry odometry_;
    double relative_orientation_;
    cv::Mat first_image_, second_image_;
    bool first_image_captured_ = false;
    bool second_image_captured_ = false;

    double angle_difference_;
    double relative_orientaion_ = 0.0;
    cv::Mat m_temp_img;
    cv::Mat m_MapBinImage;
    cv::Mat m_MapColImage;
    double map_scale_;
    double origin_x;
    double origin_y;
    unsigned int size_x;
    unsigned int size_y;
    const std::string WINDOW1 = "Map Image";
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleLocalizerNode>());
    rclcpp::shutdown();
    return 0;
}
