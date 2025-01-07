#ifndef COMPLETE_COVERAGE_PATH_HPP
#define COMPLETE_COVERAGE_PATH_HPP

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"

#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

class CompleteCoveragePath : public BT::SyncActionNode
{
public:
    CompleteCoveragePath(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr &node);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

    // Set service callback
    void handleSetCollisionFreePath(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    // Get service callback
    void handleGetCollisionFreePath(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
private:
    rclcpp::Node::SharedPtr nh_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_collision_free_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_collision_free_service_;

    void generateCoveragePath();  // Dummy function for path generation

    bool collisionFreeCcpPathAvailable_ = false;
};

#endif // COMPLETE_COVERAGE_PATH_HPP
