#include "update_ccp_map.hpp"

UpdateCcpMap::UpdateCcpMap(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr &node)
    : BT::SyncActionNode(name, config), nh_(node)
{
    // Initialize the service client
    set_collision_free_client_ = nh_->create_client<std_srvs::srv::SetBool>("set_collision_free_path");
    RCLCPP_INFO(nh_->get_logger(), "UpdateCcpMap: Service client for 'set_collision_free_path' initialized.");
}

BT::PortsList UpdateCcpMap::providedPorts()
{
    return {};
}

BT::NodeStatus UpdateCcpMap::tick()
{
    RCLCPP_INFO(nh_->get_logger(), "UpdateCcpMap: Updating the Complete Coverage Path (CCP) map...");

    // Simulate logic to update the CCP map with a detected obstacle
    // Only Call the services here and have the logic somewhere else
    // TODO: Add actual logic to modify the global map and set the variable accordingly
    updateMap();

    // Check if the service is available
    if (!set_collision_free_client_->wait_for_service(std::chrono::seconds(2)))
    {
        RCLCPP_ERROR(nh_->get_logger(), "Service 'set_collision_free_path' is not available.");
        return BT::NodeStatus::FAILURE;
    }

    // Create the service request
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = false;

    // Call the service
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

// Dummy function for updating the map
void UpdateCcpMap::updateMap()
{
    RCLCPP_INFO(nh_->get_logger(), "Simulating map update.");
    // TODO: Replace this with actual path planning logic
}
