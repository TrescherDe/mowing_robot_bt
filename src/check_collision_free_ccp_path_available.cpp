#include "check_collision_free_ccp_path_available.hpp"

CheckCollisionFreeCcpPathAvailable::CheckCollisionFreeCcpPathAvailable(
    const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr &node)
    : BT::ConditionNode(name, config), nh_(node)
{
    // Initialize the client to call the get service
    get_collision_free_client_ = nh_->create_client<std_srvs::srv::Trigger>("get_collision_free_path");
}

BT::PortsList CheckCollisionFreeCcpPathAvailable::providedPorts()
{
    return {};  // No ports required
}

BT::NodeStatus CheckCollisionFreeCcpPathAvailable::tick()
{
    if (!get_collision_free_client_->wait_for_service(std::chrono::seconds(2)))
    {
        RCLCPP_ERROR(nh_->get_logger(), "Service not available: get_collision_free_path");
        return BT::NodeStatus::FAILURE;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto response_future = get_collision_free_client_->async_send_request(request);

    // Wait for the service response
    if (rclcpp::spin_until_future_complete(nh_, response_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(nh_->get_logger(), "Failed to call service: get_collision_free_path");
        return BT::NodeStatus::FAILURE;
    }

    auto response = response_future.get();
    RCLCPP_INFO(nh_->get_logger(), "Service response: %s", response->message.c_str());

    return response->message == "Path is collision-free" ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
