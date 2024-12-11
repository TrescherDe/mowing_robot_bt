#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "check_image_data.hpp"
#include <chrono>
#include <thread>

#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create a shared ROS node
    auto shared_node = rclcpp::Node::make_shared("task_planner");

    // Create a factory and register the custom node
    BT::BehaviorTreeFactory factory;

    // Manually register the CheckImageData node with a custom builder
    factory.registerBuilder<CheckImageData>("CheckImageData",
        [&shared_node](const std::string &name, const BT::NodeConfiguration &config) {
            return std::make_unique<CheckImageData>(name, config, shared_node);
        });

    // Dynamically find the XML file
    std::string xml_file = "/workspaces/ros2_jazzy/ros_ws/src/mowing_robot_bt/trees/mowing_robot_bt.xml";

    RCLCPP_INFO(rclcpp::get_logger("task_planner"), "Loading Behavior Tree from: %s", xml_file.c_str());

    // Load the Behavior Tree
    auto tree = factory.createTreeFromFile(xml_file);

    // Add a ZMQ publisher to enable Groot visualization
    BT::PublisherZMQ zmq_publisher(tree);

    RCLCPP_INFO(rclcpp::get_logger("task_planner"), "Starting Behavior Tree execution...");

    // Main execution loop
    while (rclcpp::ok())
    {
        // Tick the root node of the tree
        BT::NodeStatus status = tree.tickRoot();

        // Optional: Log the status
        RCLCPP_INFO(shared_node->get_logger(), "Behavior Tree status: %s",
                    (status == BT::NodeStatus::SUCCESS ? "SUCCESS" :
                    (status == BT::NodeStatus::FAILURE ? "FAILURE" : "RUNNING")));

        // Allow ROS to process any callbacks
        rclcpp::spin_some(shared_node);

        // Sleep to simulate periodic ticking
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    rclcpp::shutdown();
    return 0;
}
