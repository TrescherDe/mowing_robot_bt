#ifndef CHECK_COLLISION_FREE_CCP_PATH_AVAILABLE_HPP
#define CHECK_COLLISION_FREE_CCP_PATH_AVAILABLE_HPP

#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

class CheckCollisionFreeCcpPathAvailable : public BT::ConditionNode
{
public:
    CheckCollisionFreeCcpPathAvailable(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr &node);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr nh_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr get_collision_free_client_;
};

#endif // CHECK_COLLISION_FREE_CCP_PATH_AVAILABLE_HPP
