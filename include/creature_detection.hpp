#ifndef CREATURE_DETECTION_HPP
#define CREATURE_DETECTION_HPP

#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/msg/camera_info.hpp>    

#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

class CreatureDetection : public BT::ConditionNode
{
public:
    CreatureDetection(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr &node);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
    void processImage(const sensor_msgs::msg::Image::SharedPtr msg);
    void publishCameraInfo(const std_msgs::msg::Header& header);

    rclcpp::Node::SharedPtr nh_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr marked_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr bbox_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_raw_pub_;
    
    bool creature_detected_ = false;
    bool nn_ready_ = false;
    bool m_debug = false;
    bool is_image_processed_ = true;

    cv::Mat thermal_image_;
    int compound_coef = 0;
    float threshold = 0.8;
    float nms_threshold = 0.05;
};

#endif // CREATURE_DETECTION_HPP
