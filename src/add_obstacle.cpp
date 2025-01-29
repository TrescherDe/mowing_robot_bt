#include "add_obstacle.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

//TODO:
// if obstacle is too close -> publishing #475: nav2_msgs.msg.SpeedLimit(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=''), percentage=False, speed_limit=0.05)
AddObstacle::AddObstacle(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr &node)
    : BT::SyncActionNode(name, config), nh_(node), obstacle_timeout_(60.0),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(nh_->get_clock())),
      tf_listener_(*tf_buffer_)
{
    obstacle_publisher_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("/eduard/fred/detected_obstacles", 1);
    bbox_sub_ = nh_->create_subscription<vision_msgs::msg::Detection2DArray>("/detected_creature/bboxes", 1, std::bind(&AddObstacle::bboxCallback, this, std::placeholders::_1));
   
    // Load Homography matrix (example, adjust with the real values)
    homography_matrix_ = cv::Mat::eye(3, 3, CV_64F);
    homography_matrix_.at<double>(0, 0) = 1.2;
    homography_matrix_.at<double>(0, 1) = 0.0;
    homography_matrix_.at<double>(0, 2) = -200;
    homography_matrix_.at<double>(1, 0) = 0.0;
    homography_matrix_.at<double>(1, 1) = 1.5;
    homography_matrix_.at<double>(1, 2) = -100;
    homography_matrix_.at<double>(2, 0) = 0.0;
    homography_matrix_.at<double>(2, 1) = 0.0;
    homography_matrix_.at<double>(2, 2) = 1.0;

    RCLCPP_INFO(nh_->get_logger(), "AddObstacle node initialized.");
}

void AddObstacle::bboxCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
    RCLCPP_INFO(nh_->get_logger(), "Received %ld bounding boxes", msg->detections.size());
    added_creature_as_obstacle_ = addObstacle(msg);
}

bool AddObstacle::addObstacle(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
    RCLCPP_INFO(nh_->get_logger(), "AddObstacle: Adding obstacle dynamically in front of the robot...");

    geometry_msgs::msg::Transform robot_transform = getRobotTransform();

    for (const auto& detection : msg->detections)
    {
        double x_center = detection.bbox.center.position.x;
        double y_center = detection.bbox.center.position.y;

        if(m_debug)
        {
            RCLCPP_INFO(nh_->get_logger(), "BBox: Center (%f, %f), Size (%f x %f)",
                    detection.bbox.center.position.x, detection.bbox.center.position.y,
                    detection.bbox.size_x, detection.bbox.size_y);
        
            if (!detection.results.empty())
            {
                RCLCPP_INFO(nh_->get_logger(), "Class ID: %s, Confidence: %.2f",
                            detection.results[0].hypothesis.class_id.c_str(),
                            detection.results[0].hypothesis.score);
            }
        }
        
        // Transform bounding box center to world point   ,
        geometry_msgs::msg::Point world_point = calculatePointWithIPM(x_center, y_center, homography_matrix_);

        // Transform the Point to the Map
        geometry_msgs::msg::Point map_point = transformPointToMapFrame(world_point, robot_transform);

        // Store the obstacles
        dynamic_obstacles_.emplace_back(map_point, nh_->get_clock()->now());

        if(m_debug)
        {
            RCLCPP_INFO(nh_->get_logger(), "Robot position at map coordinates: x=%f, y=%f, z=%f", robot_transform.translation.x, robot_transform.translation.y, robot_transform.translation.z);
            RCLCPP_INFO(nh_->get_logger(), "Added obstacle at map coordinates: x=%f, y=%f, z=%f", map_point.x, map_point.y, map_point.z);
        }
    }    

    publishObstacles();
    added_creature_as_obstacle_ = true;
    return true;
}

// Function to calculate the world point using IPM
geometry_msgs::msg::Point AddObstacle::calculatePointWithIPM(const double& x_center, const double& y_center, const cv::Mat &homography_matrix)
{
    // Apply the homography transformation
    cv::Mat point_image = (cv::Mat_<double>(3, 1) << x_center, y_center, 1.0);
    cv::Mat point_world = homography_matrix * point_image;

    // Normalize the resulting point
    double x_world = point_world.at<double>(0, 0) / point_world.at<double>(2, 0);
    double y_world = point_world.at<double>(1, 0) / point_world.at<double>(2, 0);

    // Create and return the Point message
    geometry_msgs::msg::Point world_point;
    world_point.x = x_world;
    world_point.y = y_world;
    world_point.z = 0.0; // Assuming the ground plane

    return world_point;
}

geometry_msgs::msg::Point AddObstacle::transformPointToMapFrame(const geometry_msgs::msg::Point &world_point, const geometry_msgs::msg::Transform &robot_transform) 
{
    // Extract robot position and orientation
    double robot_x = robot_transform.translation.x;
    double robot_y = robot_transform.translation.y;
    double robot_z = robot_transform.translation.z;

    // Extract yaw from quaternion
    tf2::Quaternion q(
        robot_transform.rotation.x,
        robot_transform.rotation.y,
        robot_transform.rotation.z,
        robot_transform.rotation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Transform the world point to the map frame
    geometry_msgs::msg::Point map_point;
    map_point.x = robot_x + (std::cos(yaw) * world_point.x - std::sin(yaw) * world_point.y);
    map_point.y = robot_y + (std::sin(yaw) * world_point.x + std::cos(yaw) * world_point.y);
    map_point.z = robot_z + world_point.z; // Assuming Z remains the same

    return map_point;
}

geometry_msgs::msg::Transform AddObstacle::getRobotTransform() 
{
    geometry_msgs::msg::Transform transform;
    try
    {        
        geometry_msgs::msg::TransformStamped transform_stamped = 
            tf_buffer_->lookupTransform("eduard/fred/map", "eduard/fred/base_link", rclcpp::Time(0), std::chrono::seconds(2));

        // Extract the transform
        transform = transform_stamped.transform;
    } 
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(nh_->get_logger(), "Failed to get transform: %s", ex.what());
    }

    return transform;
}

void AddObstacle::removeStaleObstacles()
{
    rclcpp::Time now = nh_->get_clock()->now();
    dynamic_obstacles_.erase(
        std::remove_if(dynamic_obstacles_.begin(), dynamic_obstacles_.end(),
                       [&now, this](const auto &obstacle) {
                           return (now - obstacle.second).seconds() > obstacle_timeout_;
                       }),
        dynamic_obstacles_.end());
}

void AddObstacle::publishObstacles()
{
    removeStaleObstacles();

    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.frame_id = "eduard/fred/map";
    cloud_msg.header.stamp = nh_->get_clock()->now();
    cloud_msg.height = 1;
    cloud_msg.width = dynamic_obstacles_.size();
    cloud_msg.is_dense = true;
    cloud_msg.is_bigendian = false;
    cloud_msg.point_step = 12;  // 3 fields (x, y, z) * 4 bytes each
    cloud_msg.row_step = cloud_msg.point_step * dynamic_obstacles_.size();
    cloud_msg.fields.resize(3);

    cloud_msg.fields[0].name = "x";
    cloud_msg.fields[0].offset = 0;
    cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[0].count = 1;

    cloud_msg.fields[1].name = "y";
    cloud_msg.fields[1].offset = 4;
    cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[1].count = 1;

    cloud_msg.fields[2].name = "z";
    cloud_msg.fields[2].offset = 8;
    cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[2].count = 1;

    // Prepare the PointCloud2 data buffer
    std::vector<uint8_t> data(cloud_msg.point_step * dynamic_obstacles_.size());
    for (size_t i = 0; i < dynamic_obstacles_.size(); ++i) {
        const auto &map_point = dynamic_obstacles_[i].first;  // geometry_msgs::msg::Point

        *reinterpret_cast<float *>(&data[i * 12 + 0]) = static_cast<float>(map_point.x);
        *reinterpret_cast<float *>(&data[i * 12 + 4]) = static_cast<float>(map_point.y);
        *reinterpret_cast<float *>(&data[i * 12 + 8]) = static_cast<float>(map_point.z);
    }

    cloud_msg.data = data;

    // Publish the PointCloud2 message
    obstacle_publisher_->publish(cloud_msg);
}

BT::NodeStatus AddObstacle::tick()
{
    if (added_creature_as_obstacle_)
    {
        RCLCPP_INFO(nh_->get_logger(), "Creature was added as an obstacle.");
        return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(nh_->get_logger(), "No creature found to add as an obstacle.");
    return BT::NodeStatus::FAILURE;
}