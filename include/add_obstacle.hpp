#ifndef ADD_OBSTACLE_HPP
#define ADD_OBSTACLE_HPP

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <vector>
#include <utility>
#include <array>
#include <chrono>

class AddObstacle : public BT::SyncActionNode
{
public:
    AddObstacle(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr &node);
    void addObstacle(float x, float y, float z);
    void removeStaleObstacles();
    void publishObstacles();

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<float>("x"),
            BT::InputPort<float>("y"),
            BT::InputPort<float>("z")
        };
    }

    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr nh_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_publisher_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr set_collision_free_client_; // Service client

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    std::vector<std::pair<std::array<float, 3>, rclcpp::Time>> dynamic_obstacles_;  // Buffer for obstacles with timestamps
    double obstacle_timeout_;  // Timeout duration for stale obstacles (in seconds)
    
};

#endif // ADD_OBSTACLE_HPP
