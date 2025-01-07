#ifndef UPDATE_CCP_MAP_HPP
#define UPDATE_CCP_MAP_HPP

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"

#include "std_srvs/srv/set_bool.hpp"

class UpdateCcpMap : public BT::SyncActionNode
{
public:
    UpdateCcpMap(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr &node);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    void updateMap();  // Dummy function for updating the map

    rclcpp::Node::SharedPtr nh_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr set_collision_free_client_; // Service client
};

#endif // UPDATE_CCP_MAP_HPP
