#ifndef CHECK_IMAGE_DATA_HPP
#define CHECK_IMAGE_DATA_HPP

#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include <pybind11/embed.h>  // pybind11 for Python-C++ integration
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include "sensor_msgs/msg/image.hpp"

namespace py = pybind11;

class CheckImageData : public BT::ConditionNode
{
public:
    CheckImageData(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr &node);
    ~CheckImageData();

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    bool processImage(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Node::SharedPtr nh_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    py::object detect_objects;  // Python function for detection
    bool object_detected_ = false;

    float threshold = 0.05;
    float nms_threshold = 0.5;
};

#endif // CHECK_IMAGE_DATA_HPP
