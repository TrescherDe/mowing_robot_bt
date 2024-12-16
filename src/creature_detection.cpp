#include "creature_detection.hpp"

#include <pybind11/numpy.h>

// For simulating the hedgehog detection
#include <filesystem> 
namespace fs = std::filesystem;

py::array_t<uint8_t> matToNumpy(const cv::Mat& mat)
{
    // Ensure the input image is continuous
    if (!mat.isContinuous())
    {
        throw std::runtime_error("cv::Mat is not continuous, cannot convert to NumPy array.");
    }

    // Define the shape of the NumPy array
    std::vector<ssize_t> shape = {static_cast<ssize_t>(mat.rows), static_cast<ssize_t>(mat.cols), static_cast<ssize_t>(mat.channels())};

    // Define the strides for the NumPy array, explicitly cast to ssize_t
    std::vector<ssize_t> strides = {static_cast<ssize_t>(mat.step[0]), static_cast<ssize_t>(mat.step[1]), 1};

    // Create a NumPy array from the cv::Mat
    return py::array_t<uint8_t>(
        shape,                       // Shape of the array
        strides,                     // Strides for each dimension
        mat.data                     // Pointer to the data
    );
}

CreatureDetection::CreatureDetection(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr &node)
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
    image_sub_ = nh_->create_subscription<sensor_msgs::msg::Image>("/thermal_image", 1,std::bind(&CreatureDetection::imageCallback, this, std::placeholders::_1));
    marked_image_pub_ = nh_->create_publisher<sensor_msgs::msg::Image>("/marked_image", 1);
    camera_info_pub_ = nh_->create_publisher<sensor_msgs::msg::CameraInfo>("/camera_info", 1);
}

void CreatureDetection::publishCameraInfo(const std_msgs::msg::Header &header)
{
    sensor_msgs::msg::CameraInfo camera_info_msg;
    camera_info_msg.header = header; // Use the same header as the image

    camera_info_msg.width = 640;  // Match your image width
    camera_info_msg.height = 480; // Match your image height

    camera_info_msg.k = {1.0, 0.0, camera_info_msg.width / 2.0,
                         0.0, 1.0, camera_info_msg.height / 2.0,
                         0.0, 0.0, 1.0};
    camera_info_msg.p = {1.0, 0.0, camera_info_msg.width / 2.0, 0.0,
                         0.0, 1.0, camera_info_msg.height / 2.0, 0.0,
                         0.0, 0.0, 1.0, 0.0};

    camera_info_pub_->publish(camera_info_msg);
}

CreatureDetection::~CreatureDetection()
{
    // Finalize Python interpreter
    py::finalize_interpreter();
}

BT::PortsList CreatureDetection::providedPorts()
{
    return {};  // No ports in this example
}

BT::NodeStatus CreatureDetection::tick()
{
    if (object_detected_) {
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
        std::string test_image_path = "/workspaces/ros2_jazzy/ros_ws/src/mowing_robot_bt/test_image/thermal_image_2024-11-26_13-28-07_000460.jpg";
        cv_image = cv::imread(test_image_path, cv::IMREAD_COLOR);

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
            std::string test_images_folder = "/workspaces/ros2_jazzy/ros_ws/src/mowing_robot_bt/test_video/";
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
        cv_image = cv::imread(test_image_path, cv::IMREAD_COLOR);
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
            // Convert ROS image to OpenCV format (16-bit single channel)
            cv::Mat thermal_image_16 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16)->image;

            // Normalize and convert to 8-bit
            cv::Mat thermal_image_8;
            double min_val, max_val;
            cv::minMaxLoc(thermal_image_16, &min_val, &max_val);
            thermal_image_16.convertTo(thermal_image_8, CV_8U, 255.0 / (max_val - min_val), -min_val * 255.0 / (max_val - min_val));
            cv_image = thermal_image_8;
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

    try
    {
        // Convert cv::Mat to NumPy array
        py::array_t<uint8_t> np_image = matToNumpy(cv_image);

        // Call the Python detection function
        py::object result = detect_objects(np_image, threshold, nms_threshold);

        // Cast Python list to py::list
        py::list detection_list = result.cast<py::list>();
        if(m_debug)
        {            
            RCLCPP_INFO(nh_->get_logger(), "Detections found: %ld", detection_list.size());
        }

        for (auto item : detection_list)
        {
            py::dict detection = item.cast<py::dict>();

            if (detection["rois"].cast<py::array_t<float>>().size() == 0) {
                RCLCPP_WARN(nh_->get_logger(), "Empty detection result.");
                continue;
            }

            // Convert "rois" from numpy.ndarray to pybind11::array
            py::array_t<float> rois_array = detection["rois"].cast<py::array_t<float>>();
            auto rois_info = rois_array.request(); // Get buffer info

            // Validate the shape of the ROIs array
            if (rois_info.shape.size() != 2 || rois_info.shape[1] != 4) {
                RCLCPP_ERROR(nh_->get_logger(), "ROIs shape invalid: [%ld, %ld]", rois_info.shape[0], rois_info.shape[1]);
                return false;
            }

            // Iterate over each bounding box
            float* rois_ptr = static_cast<float*>(rois_info.ptr);
            for (size_t i = 0; i < rois_info.shape[0]; ++i) {
                // Extract each bounding box
                std::vector<float> bbox(rois_ptr + i * 4, rois_ptr + (i + 1) * 4);

                // Ensure bounding box coordinates are valid
                float x_min = std::min(bbox[0], bbox[2]);
                float x_max = std::max(bbox[0], bbox[2]);
                float y_min = std::min(bbox[1], bbox[3]);
                float y_max = std::max(bbox[1], bbox[3]);

                // Convert "scores" from numpy.ndarray to float
                py::array_t<float> scores_array = detection["scores"].cast<py::array_t<float>>();
                auto scores_info = scores_array.request();
                float* scores_ptr = static_cast<float*>(scores_info.ptr);
                float score = scores_ptr[i];

                // Convert "class_ids" from numpy.ndarray to int
                py::array_t<int> class_ids_array = detection["class_ids"].cast<py::array_t<int>>();
                auto class_ids_info = class_ids_array.request();
                int* class_ids_ptr = static_cast<int*>(class_ids_info.ptr);
                int class_id = class_ids_ptr[i];

                // Draw the bounding box and add labels
                cv::rectangle(cv_image, 
                            cv::Point(x_min, y_min), 
                            cv::Point(x_max, y_max), 
                            cv::Scalar(0, 255, 0), 2);
                
                std::string label;
                if(class_id == 0)
                {
                    label = "Hedgehog: " + std::to_string(score);
                    hedgehog_detected = true;
                }
                else
                {
                    label = "Class " + std::to_string(class_id) + ": " + std::to_string(score);
                }                
                cv::putText(cv_image, label, cv::Point(x_min, y_min - 10), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);

                if(m_debug)
                {
                    RCLCPP_INFO(nh_->get_logger(),
                            "Detected object %ld: Class ID=%d, Score=%.2f, BBox=[%.2f, %.2f, %.2f, %.2f]",
                            i, class_id, score, x_min, y_min, x_max, y_max);
                }
                
                if (class_id == 0 && !nn_ready_)
                {
                    RCLCPP_INFO(nh_->get_logger(), "NN Ready");
                    nn_ready_ = true;
                }
            }
        }

        std_msgs::msg::Header header = msg->header;
        header.frame_id = "map"; // match RViz Fixed Frame
        // Convert the modified OpenCV image back to a ROS message
        sensor_msgs::msg::Image::SharedPtr marked_msg = cv_bridge::CvImage(
            header, 
            "bgr8", 
            cv_image
        ).toImageMsg();

        // Publish the marked image        
        marked_image_pub_->publish(*marked_msg);
        // Add the CameraInfo
        publishCameraInfo(header);

        return hedgehog_detected;
    }
    catch (const py::error_already_set &e)
    {
        RCLCPP_ERROR(nh_->get_logger(), "Error during Python detection: %s", e.what());
        return false;
    }

}
