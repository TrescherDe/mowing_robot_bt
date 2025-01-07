#include "complete_coverage_path.hpp"

CompleteCoveragePath::CompleteCoveragePath(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr &node)
    : BT::SyncActionNode(name, config), nh_(node)
{    
    set_collision_free_service_ = nh_->create_service<std_srvs::srv::SetBool>("set_collision_free_path",
        std::bind(&CompleteCoveragePath::handleSetCollisionFreePath, this, std::placeholders::_1, std::placeholders::_2));

    get_collision_free_service_ = nh_->create_service<std_srvs::srv::Trigger>("get_collision_free_path",
        std::bind(&CompleteCoveragePath::handleGetCollisionFreePath, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(nh_->get_logger(), "CompleteCoveragePath action node initialized.");
}

BT::PortsList CompleteCoveragePath::providedPorts()
{
    return {};
}

BT::NodeStatus CompleteCoveragePath::tick()
{
    RCLCPP_INFO(nh_->get_logger(), "Generating complete coverage path...");
    generateCoveragePath();
    return BT::NodeStatus::SUCCESS;
}

// Dummy function for path generation
void CompleteCoveragePath::generateCoveragePath()
{
    RCLCPP_INFO(nh_->get_logger(), "Generating the complete coverage path here...");
    collisionFreeCcpPathAvailable_ = true;
    // TODO: Replace this with actual path planning logic
}

void CompleteCoveragePath::handleSetCollisionFreePath(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    collisionFreeCcpPathAvailable_ = request->data;
    response->success = true;
    response->message = collisionFreeCcpPathAvailable_ ? "Path is collision-free" : "Path is not collision-free";
    RCLCPP_INFO(nh_->get_logger(), "Collision-free path set to: %s",
                collisionFreeCcpPathAvailable_ ? "true" : "false");
}

void CompleteCoveragePath::handleGetCollisionFreePath(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    response->success = true;
    response->message = collisionFreeCcpPathAvailable_ ? "Path is collision-free" : "Path is not collision-free";
    RCLCPP_INFO(nh_->get_logger(), "Collision-free path status: %s",
                collisionFreeCcpPathAvailable_ ? "true" : "false");
}
