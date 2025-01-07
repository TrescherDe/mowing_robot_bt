#include "add_obstacle.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

AddObstacle::AddObstacle(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr &node)
    : BT::SyncActionNode(name, config), nh_(node), obstacle_timeout_(5.0),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(nh_->get_clock())),
      tf_listener_(*tf_buffer_)
{
    // Initialize the service client
    set_collision_free_client_ = nh_->create_client<std_srvs::srv::SetBool>("set_collision_free_path");
    obstacle_publisher_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("/dynamic_obstacles", 1);

    RCLCPP_INFO(nh_->get_logger(), "AddObstacle: Service client for 'set_collision_free_path' initialized.");
}


void AddObstacle::addObstacle(float x, float y, float z)
{
    dynamic_obstacles_.emplace_back(std::array<float, 3>{x, y, z}, nh_->get_clock()->now());
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
    cloud_msg.header.frame_id = "map";
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

    std::vector<uint8_t> data(cloud_msg.point_step * dynamic_obstacles_.size());
    for (size_t i = 0; i < dynamic_obstacles_.size(); ++i) {
        *reinterpret_cast<float *>(&data[i * 12 + 0]) = dynamic_obstacles_[i].first[0];
        *reinterpret_cast<float *>(&data[i * 12 + 4]) = dynamic_obstacles_[i].first[1];
        *reinterpret_cast<float *>(&data[i * 12 + 8]) = dynamic_obstacles_[i].first[2];
    }
    cloud_msg.data = data;

    obstacle_publisher_->publish(cloud_msg);
}

BT::NodeStatus AddObstacle::tick()
{
    RCLCPP_INFO(nh_->get_logger(), "AddObstacle: Adding obstacle dynamically in front of the robot...");

    geometry_msgs::msg::TransformStamped transform;
    try
    {
        transform = tf_buffer_->lookupTransform("map", "base_link", rclcpp::Time(0), std::chrono::seconds(2));
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(nh_->get_logger(), "Failed to get transform: %s", ex.what());
        return BT::NodeStatus::FAILURE;
    }

    
    float x, y, z = 0.0;  // Default z to 0.0
    // need logic to use these coordinates
    if (!getInput("x", x) || !getInput("y", y)) {
        RCLCPP_ERROR(nh_->get_logger(), "Missing required inputs [x, y]");
        return BT::NodeStatus::FAILURE;
    }

    float robot_x = transform.transform.translation.x;
    float robot_y = transform.transform.translation.y;

    tf2::Quaternion q(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    float offset = 0.5; // Distance in front of the robot
    float obstacle_x = robot_x + offset * std::cos(yaw);
    float obstacle_y = robot_y + offset * std::sin(yaw);
    float obstacle_z = 0.0; // Assume ground level

    addObstacle(obstacle_x, obstacle_y, obstacle_z);
    publishObstacles();

    RCLCPP_INFO(nh_->get_logger(), "Added obstacle at (%.2f, %.2f, %.2f) and published all dynamic obstacles.", obstacle_x, obstacle_y, obstacle_z);

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
