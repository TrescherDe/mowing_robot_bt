#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

from multiprocessing import Process
from hailo_platform import (HEF, VDevice, HailoStreamInterface, InferVStreams, ConfigureParams,
InputVStreamParams, OutputVStreamParams, InputVStreams, OutputVStreams,FormatType)


class NeuralNet(Node):
    def __init__(self):
        super().__init__('neural_net')

        self.subscription = self.create_subscription(Image, '/creature_detection/image_raw', self.image_callback, 1)
        self.image_pub = self.create_publisher(Image, '/creature_detection/marked_image', 1)

        self.bridge = CvBridge()

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

        # Print letterbox debug info
        self.get_logger().info(f"Image padded: Scale={scale}, Pad W={pad_w}, Pad H={pad_h}")

        self.get_logger().info("preprocessed the image!")


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

    def debug_print_values(self, image, title):
        """ Print sample pixel values for debugging """
        print(f"\n{title} - Sample Values (Top-left 5x5):")
        for i in range(min(5, image.shape[0])):
            for j in range(min(5, image.shape[1])):
                print(f"{image[i, j]:.2f}", end=" ")
            print()

def main(args=None):
    rclpy.init(args=args)
    node = NeuralNet()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
