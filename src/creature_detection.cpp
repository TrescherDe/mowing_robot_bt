#include "creature_detection.hpp"

// For simulating the hedgehog detection
#include <filesystem> 
namespace fs = std::filesystem;

CreatureDetection::CreatureDetection(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr &node)
    : BT::ConditionNode(name, config), nh_(node)
{

    image_sub_ = nh_->create_subscription<sensor_msgs::msg::Image>("/thermal_image", 1,std::bind(&CreatureDetection::imageCallback, this, std::placeholders::_1));
    marked_image_pub_ = nh_->create_publisher<sensor_msgs::msg::Image>("/creature_detection/marked_image", 1);
    camera_info_pub_ = nh_->create_publisher<sensor_msgs::msg::CameraInfo>("/camera_info", 1);
    bbox_pub_ = nh_->create_publisher<vision_msgs::msg::Detection2DArray>("/creature_detection/bboxes", 1);
    image_raw_pub_ = nh_->create_publisher<sensor_msgs::msg::Image>("/creature_detection/image_raw", 1);
}

void CreatureDetection::publishCameraInfo(const std_msgs::msg::Header &header)
{
    sensor_msgs::msg::CameraInfo camera_info_msg;
    camera_info_msg.header = header; // Use the same header as the image

    camera_info_msg.width = 640;  // Match the image width
    camera_info_msg.height = 480; // Match the image height

    camera_info_msg.k = {1.0, 0.0, camera_info_msg.width / 2.0,
                         0.0, 1.0, camera_info_msg.height / 2.0,
                         0.0, 0.0, 1.0};
    camera_info_msg.p = {1.0, 0.0, camera_info_msg.width / 2.0, 0.0,
                         0.0, 1.0, camera_info_msg.height / 2.0, 0.0,
                         0.0, 0.0, 1.0, 0.0};

    camera_info_pub_->publish(camera_info_msg);
}

BT::PortsList CreatureDetection::providedPorts()
{
    return {};
}

BT::NodeStatus CreatureDetection::tick()
{
    if (object_detected_)
    {
        RCLCPP_INFO(nh_->get_logger(), "Object detected in image.");
        return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(nh_->get_logger(), "No object detected.");
    return BT::NodeStatus::FAILURE;
}

void CreatureDetection::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    object_detected_ = processImage(msg);
}

bool CreatureDetection::processImage(const sensor_msgs::msg::Image::SharedPtr msg)
{  
    cv::Mat cv_image;
    bool hedgehog_detected = false;

    bool simulation = true;
    static bool paths_loaded = false;
    static std::vector<std::string> test_image_paths; 
    static size_t current_image_index = 0;    

    if (!nn_ready_)
    {
        std::string test_image_path = "/workspaces/ros2_jazzy/ros_ws/src/mowing_robot_bt/test_image/thermal_image_2024-11-26_13-28-07_000460.tiff";
        cv_image = cv::imread(test_image_path, cv::IMREAD_UNCHANGED);  //Load as MONO16

        if (cv_image.empty())
        {
            RCLCPP_ERROR(nh_->get_logger(), "Failed to load test image from path: %s", test_image_path.c_str());
            return false;
        }
    }
    else if (simulation)
    {
        // Load image paths only once
        if (!paths_loaded)
        {
            RCLCPP_INFO(nh_->get_logger(), "Simulating hedgehog image data");
            std::string test_images_folder = "/workspaces/ros2_jazzy/ros_ws/src/mowing_robot_bt/test_video";
            for (const auto &entry : fs::directory_iterator(test_images_folder))
            {
                if (entry.is_regular_file())
                {
                    test_image_paths.push_back(entry.path().string());
                }
            }
            if (test_image_paths.empty())
            {
                RCLCPP_ERROR(nh_->get_logger(), "No test images found in folder: %s", test_images_folder.c_str());
                return false;
            }
            paths_loaded = true;
        }

        // Use the current image from the test_image_paths
        std::string test_image_path = test_image_paths[current_image_index];
        cv_image = cv::imread(test_image_path, cv::IMREAD_UNCHANGED);  //Load as MONO16
        if (cv_image.empty())
        {
            RCLCPP_ERROR(nh_->get_logger(), "Failed to load test video images from path: %s", test_image_path.c_str());
            return false;
        }
        current_image_index = (current_image_index + 1) % test_image_paths.size();
    }
    else
    {
        try 
        {
            // Convert ROS image to OpenCV format (Keep as MONO16)
            cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16)->image;
        }
        catch (cv_bridge::Exception &e) 
        {
            RCLCPP_ERROR(nh_->get_logger(), "cv_bridge exception: %s", e.what());
            return false;
        }
    }
    
    if(m_debug)
    {
        RCLCPP_INFO(nh_->get_logger(), "Trying to detect objects");
    }
    
    nn_ready_ = true;        
    sensor_msgs::msg::Image::SharedPtr image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "16UC1", cv_image).toImageMsg();
    image_raw_pub_->publish(*image_msg);
    return hedgehog_detected;
}
