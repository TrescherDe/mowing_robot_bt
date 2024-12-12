#ifndef CHECK_IMAGE_DATA_HPP
#define CHECK_IMAGE_DATA_HPP

#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include "sensor_msgs/msg/image.hpp"

// pybind11 for Python-C++ integration
#include <pybind11/embed.h>  
#include <pybind11/pybind11.h> 

#pragma GCC visibility push(default)

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

    // Add visibility attribute to detect_objects
    pybind11::object detect_objects; // Member visibility matches the class

    bool object_detected_ = false;
    int compound_coef = 0;
    float threshold = 0.8;
    float nms_threshold = 0.05;
};

#pragma GCC visibility pop

#endif // CHECK_IMAGE_DATA_HPP
