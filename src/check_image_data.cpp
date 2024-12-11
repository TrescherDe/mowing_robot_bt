#include "check_image_data.hpp"

CheckImageData::CheckImageData(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr &node)
    : BT::ConditionNode(name, config), nh_(node)
{
    // Subscribe to image data
    image_sub_ = nh_->create_subscription<sensor_msgs::msg::Image>(
        "/thermal_image", 10,
        std::bind(&CheckImageData::imageCallback, this, std::placeholders::_1));
}


BT::PortsList CheckImageData::providedPorts()
{
    return {}; // No input or output ports in this simple example
}

BT::NodeStatus CheckImageData::tick()
{
    if (object_detected_)
    {
        RCLCPP_INFO(nh_->get_logger(), "Object detected in image.");
        return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(nh_->get_logger(), "No object detected.");
    return BT::NodeStatus::FAILURE;
}

void CheckImageData::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    RCLCPP_INFO(nh_->get_logger(), "Image callback triggered. Height: %d, Width: %d",
                msg->height, msg->width);
    // Simulate object detection (replace with actual detection logic)
    object_detected_ = processImage(msg);
}

bool CheckImageData::processImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // Replace this with actual image processing logic (e.g., ML inference)
    return true; // Simulated detection condition
}
