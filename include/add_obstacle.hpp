#ifndef ADD_OBSTACLE_HPP
#define ADD_OBSTACLE_HPP

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <vector>
#include <utility>
#include <array>
#include <chrono>

// Inverse Perspective Mapping
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>

class AddObstacle : public BT::SyncActionNode
{
public:
    AddObstacle(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr &node);

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override;

private:
    geometry_msgs::msg::Point calculatePointWithIPM(const double& x_center, const double& y_center, const cv::Mat &homography_matrix);
    geometry_msgs::msg::Point transformPointToMapFrame(const geometry_msgs::msg::Point &world_point, const geometry_msgs::msg::Transform &robot_transform);
    geometry_msgs::msg::Transform getRobotTransform();
    void bboxCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
    bool addObstacle(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
    void removeStaleObstacles();
    void publishObstacles();

    rclcpp::Node::SharedPtr nh_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_publisher_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr bbox_sub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    std::vector<std::pair<geometry_msgs::msg::Point, rclcpp::Time>> dynamic_obstacles_;
    double obstacle_timeout_;  // Timeout duration for stale obstacles (in seconds)
    cv::Mat homography_matrix_; // Homography matrix of the camera
    bool added_creature_as_obstacle_ = false;
    bool m_debug = false;
};

#endif // ADD_OBSTACLE_HPP
