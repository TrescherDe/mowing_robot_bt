#ifndef COMPLETE_COVERAGE_PATH_HPP
#define COMPLETE_COVERAGE_PATH_HPP

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"

class CompleteCoveragePath : public BT::SyncActionNode
{
public:
    CompleteCoveragePath(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr &node);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr nh_;

    void generateCoveragePath();  // Dummy function for path generation
};

#endif // COMPLETE_COVERAGE_PATH_HPP
