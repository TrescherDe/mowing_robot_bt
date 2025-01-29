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

// Inverse Perspective Mapping
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/point.hpp>

class AddObstacle : public BT::SyncActionNode
{
public:
    AddObstacle(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr &node);
    geometry_msgs::msg::Point calculatePointWithIPM(int x_min, int y_min, int x_max, int y_max, const cv::Mat &homography_matrix);
    geometry_msgs::msg::Point transformPointToMapFrame(const geometry_msgs::msg::Point &world_point, const geometry_msgs::msg::Transform &robot_transform);
    geometry_msgs::msg::Transform getRobotTransform();
    void addObstacle(const geometry_msgs::msg::Point &map_point);
    void removeStaleObstacles();
    void publishObstacles();

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr nh_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_publisher_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr set_collision_free_client_; // Service client

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    std::vector<std::pair<geometry_msgs::msg::Point, rclcpp::Time>> dynamic_obstacles_;
    double obstacle_timeout_;  // Timeout duration for stale obstacles (in seconds)
    cv::Mat homography_matrix_; // Homography matrix of the camera    
};

#endif // ADD_OBSTACLE_HPP
