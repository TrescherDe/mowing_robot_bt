#include "complete_coverage_path.hpp"

CompleteCoveragePath::CompleteCoveragePath(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr &node)
    : BT::SyncActionNode(name, config), nh_(node)
{
    RCLCPP_INFO(nh_->get_logger(), "CompleteCoveragePath action node initialized.");
}

BT::PortsList CompleteCoveragePath::providedPorts()
{
    return
    {
        BT::InputPort<bool>("read_collisionFreePathAvailable"),
        BT::OutputPort<bool>("write_collisionFreePathAvailable")
    };
}

BT::NodeStatus CompleteCoveragePath::tick()
{
    bool collisionFreePathAvailable = false;

    if (!getInput("read_collisionFreePathAvailable", collisionFreePathAvailable))
    {
        RCLCPP_WARN(rclcpp::get_logger("CompleteCoveragePath"), "Input 'read_collisionFreePathAvailable' was not yet set. Defaulting to false.");
    }

    RCLCPP_INFO(rclcpp::get_logger("CompleteCoveragePath"), 
                "Input value: %s", collisionFreePathAvailable ? "true" : "false");

    if(collisionFreePathAvailable)
    {
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        RCLCPP_INFO(nh_->get_logger(), "Starting CompleteCoveragePath action...");

        // update this variable in the generateCoveragePath, e.g. success&
        collisionFreePathAvailable = true;

        // Call the dummy function for path generation
        generateCoveragePath();

        RCLCPP_INFO(nh_->get_logger(), "Setting collisionFreeCcpPathAvailable to true.");
        setOutput("write_collisionFreePathAvailable", collisionFreePathAvailable);

        return BT::NodeStatus::SUCCESS;
    }
}

// Dummy function for path generation
void CompleteCoveragePath::generateCoveragePath()
{
    RCLCPP_INFO(nh_->get_logger(), "Generating the complete coverage path here...");
    // TODO: Replace this with actual path planning logic
}
