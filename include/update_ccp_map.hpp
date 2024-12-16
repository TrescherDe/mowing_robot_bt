#ifndef UPDATE_CCP_MAP_HPP
#define UPDATE_CCP_MAP_HPP

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"

class UpdateCcpMap : public BT::SyncActionNode
{
public:
    UpdateCcpMap(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr &node);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    void updateMap();  // Dummy function for updating the map

    rclcpp::Node::SharedPtr nh_;
};

#endif // UPDATE_CCP_MAP_HPP
