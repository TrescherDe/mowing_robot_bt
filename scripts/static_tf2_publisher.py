#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations

import numpy as np
class StaticTFPublisher(Node):
    def __init__(self):
        super().__init__('static_tf2_publisher')

        # TF Broadcasters
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        
        # TF Buffer & Listener to check for existing transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
        self.timer = self.create_timer(0.5, self.check_and_publish_transforms)

    def check_and_publish_transforms(self):
        """ Check if base_link -> map exists. If not, publish base_link at z=0.04 """
        try:
            # Check for transform availability
            transform = self.tf_buffer.lookup_transform("eduard/fred/map", "eduard/fred/base_link", rclpy.time.Time())
            self.get_logger().info("Transform exists: /eduard/fred/map -> /eduard/fred/base_link")

        except Exception as e:
            self.get_logger().warn("No transform found: /eduard/fred/map -> /eduard/fred/base_link. Publishing default.")
            base_link_transform = TransformStamped()
            base_link_transform.header.stamp = self.get_clock().now().to_msg()
            base_link_transform.header.frame_id = "eduard/fred/map"
            base_link_transform.child_frame_id = "eduard/fred/base_link"
            base_link_transform.transform.translation.x = 0.0
            base_link_transform.transform.translation.y = 0.0
            base_link_transform.transform.translation.z = 0.04

            base_link_transform.transform.rotation.x = 0.0
            base_link_transform.transform.rotation.y = 0.0
            base_link_transform.transform.rotation.z = 0.0
            base_link_transform.transform.rotation.w = 1.0 

            self.tf_broadcaster.sendTransform(base_link_transform)

        # Always publish static transform for the thermal camera
        self.publish_camera_transform()

    def publish_camera_transform(self):
        """ Publishes the transform from /eduard/fred/base_link to /thermal_camera_frame """
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "eduard/fred/base_link"
        transform.child_frame_id = "thermal_camera_frame"

        # Empirically found values: tilt of the camera ~ -0.01, x = -0.43 (due to zoom offsett i guess!)

        transform.transform.translation.x = 0.05 # normally it would be 5cm offset to base_link
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.17  # Camera height

        # Rotation: 90° Z then 90° X
        q = tf_transformations.quaternion_from_euler(-1.5708, 0, -1.5708)  # (π/2, 0, π/2)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().info("Published transform: base_link -> thermal_camera_frame")

def main():
    rclpy.init()
    node = StaticTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
