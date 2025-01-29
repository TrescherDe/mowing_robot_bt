#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "creature_detection.hpp"
#include "add_obstacle.hpp"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include <chrono>
#include <thread>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Use shared pointers for ROS2 nodes
    auto main_node = std::make_shared<rclcpp::Node>("task_planner");
    auto vision_node = std::make_shared<rclcpp::Node>("vision_node");
    auto navigation_node = std::make_shared<rclcpp::Node>("navigation_node");

    // Create a factory and register the custom node
    BT::BehaviorTreeFactory factory;

    factory.registerBuilder<CreatureDetection>("creatureDetected",
    [&vision_node](const std::string &name, const BT::NodeConfiguration &config) 
    {
        return std::make_unique<CreatureDetection>(name, config, vision_node);
    });

    factory.registerBuilder<AddObstacle>("addObstacle",
    [&navigation_node](const std::string &name, const BT::NodeConfiguration &config) 
    {
        return std::make_unique<AddObstacle>(name, config, navigation_node);
    });

    // Load the Behavior Tree
    std::string xml_file = "/workspaces/eduartrobotik_ros2_jazzy/ros_ws/src/mowing_robot_bt/trees/mowing_robot_bt.xml";
    
    RCLCPP_INFO(main_node->get_logger(), "Loading Behavior Tree from: %s", xml_file.c_str());
    auto tree = factory.createTreeFromFile(xml_file);

    // Add a ZMQ publisher to enable Groot visualization
    BT::PublisherZMQ zmq_publisher(tree);

    RCLCPP_INFO(main_node->get_logger(), "Starting Behavior Tree execution...");

    // Main execution loop
    while (rclcpp::ok())
    {
        // Tick the root node of the tree
        BT::NodeStatus status = tree.tickRoot();

        // Log the status
        RCLCPP_INFO(main_node->get_logger(), "Behavior Tree status: %s",
                    (status == BT::NodeStatus::SUCCESS ? "SUCCESS" :
                    (status == BT::NodeStatus::FAILURE ? "FAILURE" : "RUNNING")));

        // Process callbacks for all nodes
        rclcpp::spin_some(vision_node);
        rclcpp::spin_some(navigation_node);
        rclcpp::spin_some(main_node);

        // Sleep to simulate periodic ticking
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    rclcpp::shutdown();
    return 0;
}
