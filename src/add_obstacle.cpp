#include "add_obstacle.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

//TODO:
// if obstacle is too close -> publishing #475: nav2_msgs.msg.SpeedLimit(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=''), percentage=False, speed_limit=0.05)
AddObstacle::AddObstacle(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr &node)
    : BT::SyncActionNode(name, config), nh_(node), obstacle_timeout_(60.0),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(nh_->get_clock())),
      tf_listener_(*tf_buffer_)
{
    // Initialize the service client
    set_collision_free_client_ = nh_->create_client<std_srvs::srv::SetBool>("set_collision_free_path");
    obstacle_publisher_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("/eduard/fred/detected_obstacles", 1);

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

    RCLCPP_INFO(nh_->get_logger(), "AddObstacle: Service client for 'set_collision_free_path' initialized.");
}


void AddObstacle::addObstacle(const geometry_msgs::msg::Point &map_point)
{
    // Store the obstacle as a map point
    dynamic_obstacles_.emplace_back(map_point, nh_->get_clock()->now());
}

// Function to calculate the world point using IPM
geometry_msgs::msg::Point AddObstacle::calculatePointWithIPM(int x_min, int y_min, int x_max, int y_max, const cv::Mat &homography_matrix)
{
    // Calculate the center of the bounding box
    double x_center = (x_min + x_max) / 2.0;
    double y_center = (y_min + y_max) / 2.0;

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
    RCLCPP_INFO(nh_->get_logger(), "AddObstacle: Adding obstacle dynamically in front of the robot...");

    geometry_msgs::msg::Transform robot_transform = getRobotTransform();

    //float x, y, z = 0.0;  // Default z to 0.0
    int x_min = 100, y_min = 200, x_max = 300, y_max = 400;

    // Transform bounding box center to world point
    geometry_msgs::msg::Point world_point = calculatePointWithIPM(x_min, y_min, x_max, y_max, homography_matrix_);
    // Transform the Point to the Map
    geometry_msgs::msg::Point map_point = transformPointToMapFrame(world_point, robot_transform);
    map_point.x = 0.5;
    map_point.y = 0.5;
    map_point.z = 0.57;
    addObstacle(map_point);
   
    RCLCPP_INFO(nh_->get_logger(), "Robot position at map coordinates: x=%f, y=%f, z=%f", robot_transform.translation.x, robot_transform.translation.y, robot_transform.translation.z);
    RCLCPP_INFO(nh_->get_logger(), "Added obstacle at map coordinates: x=%f, y=%f, z=%f", map_point.x, map_point.y, map_point.z);

    publishObstacles();

    return BT::NodeStatus::SUCCESS;

    if (!set_collision_free_client_->wait_for_service(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(nh_->get_logger(), "Service 'set_collision_free_path' is not available.");
        return BT::NodeStatus::FAILURE;
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = false;

    auto future = set_collision_free_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(nh_, future) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(nh_->get_logger(), "Failed to call service 'set_collision_free_path'.");
        return BT::NodeStatus::FAILURE;
    }

    auto response = future.get();
    if (!response->success)
    {
        RCLCPP_ERROR(nh_->get_logger(), "Service call to 'set_collision_free_path' failed: %s", response->message.c_str());
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(nh_->get_logger(), "Service call to 'set_collision_free_path' succeeded: %s", response->message.c_str());

    return BT::NodeStatus::SUCCESS;
}
