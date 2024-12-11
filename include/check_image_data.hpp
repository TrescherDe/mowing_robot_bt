#ifndef CHECK_IMAGE_DATA_HPP
#define CHECK_IMAGE_DATA_HPP

#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class CheckImageData : public BT::ConditionNode
{
public:
    CheckImageData(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr &node);


    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    bool processImage(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Node::SharedPtr nh_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    bool object_detected_ = false;
};

#endif // CHECK_IMAGE_DATA_HPP
