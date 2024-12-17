#include "update_ccp_map.hpp"

UpdateCcpMap::UpdateCcpMap(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr &node)
    : BT::SyncActionNode(name, config), nh_(node)
{
    RCLCPP_INFO(nh_->get_logger(), "UpdateCcpMap action node initialized.");
}

BT::PortsList UpdateCcpMap::providedPorts()
{
    return {BT::OutputPort<bool>("write_collisionFreePathAvailable")};
}

BT::NodeStatus UpdateCcpMap::tick()
{
    RCLCPP_INFO(nh_->get_logger(), "UpdateCcpMap: Updating the Complete Coverage Path (CCP) map...");

    // Simulate logic to update the CCP map with a detected obstacle
    // Only Call the services here and have the logic somewhere else
    // TODO: Add actual logic to modify the global map and set the variable accordingly
    updateMap();

    // For this example, simulate failure to create a collision-free path
    RCLCPP_INFO(nh_->get_logger(), "Setting collisionFreeCcpPathAvailable to false.");
    setOutput("write_collisionFreePathAvailable", false);

    return BT::NodeStatus::SUCCESS;
}

// Dummy function for updating the map
void UpdateCcpMap::updateMap()
{
    RCLCPP_INFO(nh_->get_logger(), "Simulating map update.");
    // TODO: Replace this with actual path planning logic
}
