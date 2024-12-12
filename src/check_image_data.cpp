#include "check_image_data.hpp"

CheckImageData::CheckImageData(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr &node)
    : BT::ConditionNode(name, config), nh_(node)
{
    
    // Initialize Python interpreter
    py::initialize_interpreter();

    py::module::import("sys").attr("path").attr("append")("/workspaces/ros2_jazzy/01_repos/PMBW_Object_Detection_In_Thermal_Images");

    try 
    {
        py::module detection_wrapper = py::module::import("detection_wrapper");
        detect_objects = detection_wrapper.attr("load_and_detect");
    } 
    catch (const py::error_already_set &e) 
    {
        RCLCPP_ERROR(nh_->get_logger(), "Error initializing Python detection module: %s", e.what());
        throw;
    }
}


CheckImageData::~CheckImageData()
{
    // Finalize Python interpreter
    py::finalize_interpreter();
}

BT::PortsList CheckImageData::providedPorts()
{
    return {};  // No ports in this example
}

BT::NodeStatus CheckImageData::tick()
{
    if (object_detected_) {
        RCLCPP_INFO(nh_->get_logger(), "Object detected in image.");
        return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(nh_->get_logger(), "No object detected.");
    return BT::NodeStatus::FAILURE;
}

void CheckImageData::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    object_detected_ = processImage(msg);
}

bool CheckImageData::processImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv::Mat cv_image;
    try {
        // Convert ROS image to OpenCV format
        cv_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(nh_->get_logger(), "cv_bridge exception: %s", e.what());
        return false;
    }

    try {
        // Call the Python detection function
        py::object result = detect_objects(cv_image, threshold, nms_threshold);

        // Check if any objects were detected
        auto detections = result.cast<std::vector<py::dict>>();
        return !detections.empty();
    } catch (const py::error_already_set &e) {
        RCLCPP_ERROR(nh_->get_logger(), "Error during Python detection: %s", e.what());
        return false;
    }
}
