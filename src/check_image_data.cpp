#include "check_image_data.hpp"

#include <pybind11/numpy.h>

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
    image_sub_ = nh_->create_subscription<sensor_msgs::msg::Image>("/thermal_image", 1,std::bind(&CheckImageData::imageCallback, this, std::placeholders::_1));

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
    bool simulation = false;
    cv::Mat cv_image;

    if (simulation)
    {
        // Load the test image
        std::string test_image_path = "/workspaces/ros2_jazzy/ros_ws/src/mowing_robot_bt/test_image/thermal_image_2024-11-26_13-28-07_000460.jpg";
        cv_image = cv::imread(test_image_path, cv::IMREAD_COLOR);

        if (cv_image.empty())
        {
            RCLCPP_ERROR(nh_->get_logger(), "Failed to load test image from path: %s", test_image_path.c_str());
            return false;
        }
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
            cv::minMaxLoc(thermal_image_16, &min_val, &max_val); // Find the range of the 16-bit image
            thermal_image_16.convertTo(thermal_image_8, CV_8U, 255.0 / (max_val - min_val), -min_val * 255.0 / (max_val - min_val));
            cv_image = thermal_image_8;
        }
        catch (cv_bridge::Exception &e) 
        {
            RCLCPP_ERROR(nh_->get_logger(), "cv_bridge exception: %s", e.what());
            return false;
        }
    }
    
    RCLCPP_INFO(nh_->get_logger(), "Trying to detect objects");

    try
    {
        // Convert cv::Mat to NumPy array
        py::array_t<uint8_t> np_image = matToNumpy(cv_image);

        // Call the Python detection function
        py::object result = detect_objects(np_image, threshold, nms_threshold);

        // Cast Python list to py::list
        py::list detection_list = result.cast<py::list>();
        RCLCPP_INFO(nh_->get_logger(), "Detections found: %ld", detection_list.size());

        for (auto item : detection_list)
        {
            py::dict detection = item.cast<py::dict>();

            // Check if 'rois', 'scores', or 'class_ids' are empty
            if (detection["rois"].cast<py::array_t<float>>().size() == 0) {
                RCLCPP_WARN(nh_->get_logger(), "Empty detection result.");
                continue;
            }

            // Convert "rois" from numpy.ndarray to std::vector<float>
            py::array_t<float> rois_array = detection["rois"].cast<py::array_t<float>>();
            auto rois_info = rois_array.request(); // Get buffer info
            float* rois_ptr = static_cast<float*>(rois_info.ptr); // Pointer to the data
            std::vector<float> bbox(rois_ptr, rois_ptr + rois_info.shape[0]); // Construct vector

            // Convert "scores" from numpy.ndarray to float
            py::array_t<float> scores_array = detection["scores"].cast<py::array_t<float>>();
            float score = *static_cast<float*>(scores_array.request().ptr);

            // Convert "class_ids" from numpy.ndarray to int
            py::array_t<int> class_ids_array = detection["class_ids"].cast<py::array_t<int>>();
            int class_id = *static_cast<int*>(class_ids_array.request().ptr);

            RCLCPP_INFO(nh_->get_logger(),
                        "Detected object: Class ID=%d, Score=%.2f, BBox=[%.2f, %.2f, %.2f, %.2f]",
                        class_id, score, bbox[0], bbox[1], bbox[2], bbox[3]);
        }

        return !detection_list.empty();
    }
    catch (const py::error_already_set &e)
    {
        RCLCPP_ERROR(nh_->get_logger(), "Error during Python detection: %s", e.what());
        return false;
    }

}
