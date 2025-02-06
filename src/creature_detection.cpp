#include "creature_detection.hpp"

// For simulating the hedgehog detection
#include <filesystem> 
namespace fs = std::filesystem;

CreatureDetection::CreatureDetection(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr &node)
    : BT::ConditionNode(name, config), nh_(node)
{
    image_sub_ = nh_->create_subscription<sensor_msgs::msg::Image>("/thermal_image", 1, std::bind(&CreatureDetection::imageCallback, this, std::placeholders::_1));
    marked_image_pub_ = nh_->create_publisher<sensor_msgs::msg::Image>("/creature_detection/marked_image", 1);
    camera_info_pub_ = nh_->create_publisher<sensor_msgs::msg::CameraInfo>("/camera_info", 1);
    bbox_sub_ = nh_->create_subscription<vision_msgs::msg::Detection2DArray>("/creature_detection/bboxes", 1, std::bind(&CreatureDetection::detectionCallback, this, std::placeholders::_1));
    image_raw_pub_ = nh_->create_publisher<sensor_msgs::msg::Image>("/creature_detection/image_raw", 1);
}

void CreatureDetection::publishCameraInfo(const std_msgs::msg::Header &header)
{
    sensor_msgs::msg::CameraInfo camera_info_msg;
    camera_info_msg.header = header; // Use the same header as the image

    camera_info_msg.width = 640;  // Match the image width
    camera_info_msg.height = 480; // Match the image height

    // Camera Intrinsic Matrix (K)
    camera_info_msg.k = {
        568.834076, 0.0, 320.907981,  // fx, 0, cx
        0.0, 568.518058, 238.348171,  // 0, fy, cy
        0.0, 0.0, 1.0                 // 0, 0, 1
    };

    // Distortion Coefficients (D)
    camera_info_msg.distortion_model = "plumb_bob";
    camera_info_msg.d = {-0.420116, 0.238702, 0.001551, -0.002084, 0.0};

    // Rectification Matrix (R)
    camera_info_msg.r = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    };

    // Projection Matrix (P)
    camera_info_msg.p = {
        491.301453, 0.0, 318.747986, 0.0,
        0.0, 523.376709, 238.217768, 0.0,
        0.0, 0.0, 1.0, 0.0
    };
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
    processImage(msg);
}

void CreatureDetection::detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
    if(m_debug)
    {
        RCLCPP_INFO(nh_->get_logger(), "Processing the received bounding boxes.");
    }
    
    if (thermal_image_.empty())  // Ensure image is loaded
    {
        RCLCPP_WARN(nh_->get_logger(), "No image available to process.");
        return;
    }

    cv::Mat marked_image = thermal_image_.clone();  // Clone to avoid modifying original
    is_image_processed_ = true;

    // Iterate over detections
    for (const auto &detection : msg->detections)
    {
        // Extract bounding box info
        int x = static_cast<int>(detection.bbox.center.position.x - detection.bbox.size_x / 2);
        int y = static_cast<int>(detection.bbox.center.position.y - detection.bbox.size_y / 2);
        int width = static_cast<int>(detection.bbox.size_x);
        int height = static_cast<int>(detection.bbox.size_y);

        // Ensure coordinates are within bounds
        x = std::max(0, x);
        y = std::max(0, y);
        width = std::min(marked_image.cols - x, width);
        height = std::min(marked_image.rows - y, height);

        // Draw bounding box (white for thermal image)
        cv::rectangle(marked_image, cv::Rect(x, y, width, height), cv::Scalar(255, 255, 255), 2);

        if(detection.results[0].hypothesis.class_id == "hedgehog")
        {
            nn_ready_ = true;
            object_detected_ = true;
        }

        // Put label (class_id) above the box
        if (!detection.results.empty())
        {
            std::string label = detection.results[0].hypothesis.class_id + " " +
                                std::to_string(detection.results[0].hypothesis.score);
            cv::putText(marked_image, label, cv::Point(x, y - 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        }
    }

    // Convert back to ROS2 message and publish
    sensor_msgs::msg::Image::SharedPtr image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "16UC1", marked_image).toImageMsg();

    image_msg->header.stamp = nh_->get_clock()->now();
    image_msg->header.frame_id = "thermal_camera_frame";

    marked_image_pub_->publish(*image_msg);
    RCLCPP_INFO(nh_->get_logger(), "Published marked image.");
}


void CreatureDetection::processImage(const sensor_msgs::msg::Image::SharedPtr msg)
{  
    cv::Mat cv_image;
    bool simulation = true;
    static bool paths_loaded = false;
    static std::vector<std::string> test_image_paths;
    static size_t current_image_index = 0;

    if (!nn_ready_)
    {
        std::string test_image_path = "/workspaces/ros_jazzy/ros_ws/src/mowing_robot_bt/test_image/thermal_image_2024-11-26_13-28-07_000460.tiff";
        cv_image = cv::imread(test_image_path, cv::IMREAD_UNCHANGED);  //Load as MONO16

        if (cv_image.empty())
        {
            RCLCPP_ERROR(nh_->get_logger(), "Failed to load test image from path: %s", test_image_path.c_str());
            return;
        }
    }
    else if (simulation)
    {
        if (!paths_loaded)
        {
            RCLCPP_INFO(nh_->get_logger(), "Simulating hedgehog image data");
            std::string test_images_folder = "/workspaces/ros_jazzy/ros_ws/src/mowing_robot_bt/test_video";
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
                return;
            }
            paths_loaded = true;
        }

        // Use the current image from the test_image_paths
        std::string test_image_path = test_image_paths[current_image_index];
        cv_image = cv::imread(test_image_path, cv::IMREAD_UNCHANGED);  //Load as MONO16
        if (cv_image.empty())
        {
            RCLCPP_ERROR(nh_->get_logger(), "Failed to load test video images from path: %s", test_image_path.c_str());
            return;
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
            return;
        }
    }

    if(is_image_processed_)
    {
        is_image_processed_ = false;
        thermal_image_ = cv_image;
        std_msgs::msg::Header header = msg->header;
        header.frame_id = "thermal_camera_frame"; // match RViz Fixed Frame
        sensor_msgs::msg::Image::SharedPtr image_msg = cv_bridge::CvImage(header, "16UC1", cv_image).toImageMsg();
        image_raw_pub_->publish(*image_msg);
        publishCameraInfo(header);

        if(m_debug)
        {
            RCLCPP_INFO(nh_->get_logger(), "Trying to detect objects");
        }
    }    
}
