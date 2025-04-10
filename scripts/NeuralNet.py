#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from cv_bridge import CvBridge

from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose, BoundingBox2D, Point2D, Pose2D
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from hailo_platform import (HEF, VDevice, HailoStreamInterface, InferVStreams, ConfigureParams,
InputVStreamParams, OutputVStreamParams, InputVStreams, OutputVStreams,FormatType)

from pathlib import Path
from ament_index_python.packages import get_package_share_directory

# Get path to the 'mowing_robot_bt' package share directory
package_share_path = Path(get_package_share_directory('mowing_robot_bt'))

class NeuralNet(Node):
    def __init__(self):
        super().__init__('neural_net')
        self.image_sub_ = self.create_subscription(Image, '/creature_detection/image_raw', self.image_callback, 1)
        self.bbox_pub_ = self.create_publisher(Detection2DArray, '/creature_detection/bboxes', 1)
        self.bridge = CvBridge()
        self.m_debug = False

    def image_callback(self, msg):
        """ Callback function for processing incoming images """
        self.get_logger().info("Received Image for Processing.")

        # Convert ROS Image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Ensure the image is 16-bit unsigned (uint16)
        if cv_image.dtype != np.uint16:
            self.get_logger().error("Unexpected image format! Expected uint16 (MONO16), got:", cv_image.dtype)
            return

        # Convert MONO16 to Float32
        thermal_image_float = cv_image.astype(np.float32)

        # Convert grayscale to RGB
        image_rgb = cv2.cvtColor(thermal_image_float, cv2.COLOR_GRAY2RGB)

        # Apply letterbox padding
        image_padded, scale, pad_w, pad_h = self.letterbox_image(image_rgb)

        if self.m_debug:
            self.get_logger().info(f"Image padded: Scale={scale}, Pad W={pad_w}, Pad H={pad_h}")

        # Add batch dimension
        if len(image_padded.shape) == 3:
            image_padded = np.expand_dims(image_padded, axis=0) # Shape (1, 640, 640, 3)

        if self.m_debug:
            self.get_logger().info(f"Input shape after batch dimension: {image_padded.shape}")

        target = VDevice()

        hef_path = package_share_path / 'model' / 'hedgehog_yolov8n.hef'

        hef = HEF(str(hef_path))

        # Configure network groups
        configure_params = ConfigureParams.create_from_hef(hef=hef, interface=HailoStreamInterface.PCIe)
        network_groups = target.configure(hef, configure_params)
        network_group = network_groups[0]
        network_group_params = network_group.create_params()

        # Create input and output virtual streams params
        input_vstreams_params = InputVStreamParams.make(network_group, format_type=FormatType.FLOAT32)
        output_vstreams_params = OutputVStreamParams.make(network_group, format_type=FormatType.FLOAT32)        

        input_vstream_info = hef.get_input_vstream_infos()[0]
        output_vstream_info = hef.get_output_vstream_infos()[0]

        if self.m_debug:
            self.get_logger().info(f"Expected input shape: {input_vstream_info.shape}")        

        # Infer
        with InferVStreams(network_group, input_vstreams_params, output_vstreams_params) as infer_pipeline:
            input_data = {input_vstream_info.name: image_padded}
            with network_group.activate(network_group_params):
                infer_results = infer_pipeline.infer(input_data)

        # Process and Display Results
        output_data = infer_results[output_vstream_info.name]
        output_data = np.array(output_data)

        if self.m_debug:
            print("Inference Output Shape:", output_data.shape)
            print("Inference Results:", output_data)

        self.publish_detections(output_data)

        
    def publish_detections(self, output_data):

        detection_array = Detection2DArray()
        detection_array.header = Header()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = "thermal_camera_frame"   

        if output_data.shape[2] == 0 or output_data.shape[1] == 0:
            self.get_logger().warn("No detections found. Publishing empty message.")
            self.bbox_pub_.publish(detection_array)
            return

        # Iterate over detected objects
        for i in range(output_data.shape[2]):
            try:
                ymin, xmin, ymax, xmax, confidence = output_data[0, 0, i, :]

                if confidence < 0.5:
                    continue

                # Convert normalized values to image scale
                xmin *= 640
                ymin *= 640
                xmax *= 640
                ymax *= 640

                # Remove the letterbox padding
                ymin -= 80
                ymax -= 80

                # Create Detection2D message
                # bounding box
                detection = Detection2D()
                detection.header = Header()
                detection.header.stamp = self.get_clock().now().to_msg()
                detection.header.frame_id = "thermal_camera_frame"

                position_center = Point2D()
                position_center.x = float((xmin + xmax) / 2.0)
                position_center.y = float((ymin + ymax) / 2.0)

                bbox_center = Pose2D()
                bbox_center.position = position_center
                bbox_center.theta = 0.0

                bbox = BoundingBox2D()
                bbox.center = bbox_center
                bbox.size_x = float(xmax - xmin)
                bbox.size_y = float(ymax - ymin)

                detection.bbox = bbox 

                # classification result
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = "hedgehog" # only one class exists
                hypothesis.hypothesis.score = float(confidence)

                detection.results.append(hypothesis)

                if self.m_debug:
                    print("Detection Before Append:")
                    print(f" - Position_center X: {detection.bbox.center.position.x} ")
                    print(f" - Position_center Y: {detection.bbox.center.position.y} ")
                    print(f" - Size X: {detection.bbox.size_x} ({type(detection.bbox.size_x)})")
                    print(f" - Size Y: {detection.bbox.size_y} ({type(detection.bbox.size_y)})")
                    print(f" - Results: {detection.results}")

                detection_array.detections.append(detection)

            except IndexError:
                self.get_logger().error(f"IndexError: output_data[{i}] is out of bounds.")
                continue

        self.bbox_pub_.publish(detection_array)
        if detection_array.detections:
            self.get_logger().info(f"Published {len(detection_array.detections)} valid detections.")
        else:
            self.get_logger().warn("No valid detections after filtering. Published empty message.")



    def letterbox_image(self, image, model_w=640, model_h=640):
        """ Resize and pad image while maintaining aspect ratio """
        orig_h, orig_w = image.shape[:2]

        # Compute the scale factor to fit within model size
        scale = min(model_w / orig_w, model_h / orig_h)

        # Compute new size
        new_w = int(orig_w * scale)
        new_h = int(orig_h * scale)

        # Resize while keeping aspect ratio
        resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        # Compute padding
        pad_w = (model_w - new_w) // 2
        pad_h = (model_h - new_h) // 2

        # Add padding
        padded_image = cv2.copyMakeBorder(resized, pad_h, pad_h, pad_w, pad_w, cv2.BORDER_CONSTANT, value=(0, 0, 0))

        return padded_image, scale, pad_w, pad_h

def main(args=None):
    rclpy.init(args=args)
    node = NeuralNet()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
