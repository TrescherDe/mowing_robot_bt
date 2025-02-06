#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class StaticTFPublisher(Node):
    def __init__(self):
        super().__init__('static_tf2_publisher')
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.timer = self.create_timer(1.0, self.publish_transform)

    def publish_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "thermal_camera_frame"

        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.1  # Camera height

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().info("Published static transform: map -> thermal_camera_frame")

def main():
    rclpy.init()
    node = StaticTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
