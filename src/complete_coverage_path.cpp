#include "complete_coverage_path.hpp"

CompleteCoveragePath::CompleteCoveragePath(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr &node)
    : BT::SyncActionNode(name, config), nh_(node)
{
    RCLCPP_INFO(nh_->get_logger(), "CompleteCoveragePath action node initialized.");
}

BT::PortsList CompleteCoveragePath::providedPorts()
{
    return {BT::OutputPort<bool>("collisionFreePathAvailable")};
}

BT::NodeStatus CompleteCoveragePath::tick()
{
    RCLCPP_INFO(nh_->get_logger(), "Starting CompleteCoveragePath action...");

    // Call the dummy function for path generation
    generateCoveragePath();

    // Set the blackboard variable ?
    //config().blackboard->set("collisionFreeCppPathAvailable", true);
    //RCLCPP_INFO(nh_->get_logger(), "Blackboard: collisionFreeCppPathAvailable set to true");

    // Return SUCCESS
    return BT::NodeStatus::SUCCESS;
}

// Dummy function for path generation
void CompleteCoveragePath::generateCoveragePath()
{
    RCLCPP_INFO(nh_->get_logger(), "Generating the complete coverage path here...");
    // TODO: Replace this with actual path planning logic
}
