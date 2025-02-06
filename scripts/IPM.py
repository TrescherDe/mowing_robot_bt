#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import struct
from ipm_library.exceptions import NoIntersectionError
from ipm_library.ipm import IPM

import tf2_ros as tf2
from sensor_msgs.msg import CameraInfo, PointCloud2, PointField
from shape_msgs.msg import Plane
from vision_msgs.msg import Point2D, Detection2DArray

class IPMExample(Node):
    def __init__(self):
        super().__init__('IPM')

        self.pointcloud_pub = self.create_publisher(PointCloud2, '/eduard/fred/detected_obstacles', 1)
        self.m_debug = False

        self.tf_buffer = tf2.Buffer()
        self.tf_listener = tf2.TransformListener(self.tf_buffer, self, spin_thread=True)

        self.camera_info = None
        self.ipm = None

        self.create_subscription(CameraInfo, '/camera_info', self.camera_info_callback, 1)
        self.create_subscription(Detection2DArray, '/creature_detection/bboxes', self.bbox_callback, 1)

        # Define the ground plane (normal is in z-direction)
        self.plane = Plane()
        self.plane.coef = [0.0, 0.0, 1.0, -1.0] # adjust to the relative ground -> test images were taken ~ 1m above the ground

    def camera_info_callback(self, msg: CameraInfo):
        """Updates camera parameters when receiving /camera_info."""
        self.camera_info = msg

        self.ipm = IPM(self.tf_buffer, self.camera_info, distortion=True)
        self.get_logger().info("Received CameraInfo and initialized IPM.")

    def bbox_callback(self, msg: Detection2DArray):
        """Processes bounding boxes and projects them to world coordinates."""
        if self.ipm is None:
            self.get_logger().warn("No CameraInfo received yet. Cannot perform IPM.")
            return
        
        if len(msg.detections) == 0 or (len(msg.detections) == 1 and msg.detections[0].results == []):
            self.get_logger().warn("Received an empty Detection2DArray, skipping IPM projection.")
            return

        points = []

        for detection in msg.detections:
            bbox = detection.bbox
            x_center = bbox.center.position.x
            y_center = bbox.center.position.y                

            if self.m_debug:
                if self.camera_info:
                    self.get_logger().info(f"Creature detected at image coordinate X={x_center}, Y={y_center}")
                    self.get_logger().info(f"Camera Info received: frame_id={self.camera_info.header.frame_id}, width={self.camera_info.width}, height={self.camera_info.height}")
                    self.get_logger().info(f"Intrinsic Matrix (K): {self.camera_info.k}")
                else:
                    self.get_logger().warn("No CameraInfo available!")

            time = msg.header.stamp  

            try:
                self.tf_buffer.can_transform('map', 'thermal_camera_frame', time, rclpy.duration.Duration(seconds=2.0))

                projected_point = self.ipm.map_point(
                    self.plane,
                    Point2D(x=x_center, y=y_center),
                    time,
                    plane_frame_id="thermal_camera_frame",
                    output_frame_id="map"
                )

                self.get_logger().info(f"Projected Creature at X={projected_point.point.x:.2f}, Y={projected_point.point.y:.2f}")

                # Store as a tuple (X, Y, Z)
                points.append((projected_point.point.x, projected_point.point.y, projected_point.point.z))

            except NoIntersectionError:
                self.get_logger().warn("No valid intersection found for IPM.")

        self.publish_pointcloud(points, msg.header)

    def publish_pointcloud(self, points, header):
        """Converts a list of (x, y, z) points into a PointCloud2 message."""
        if not points:
            self.get_logger().warn("No valid projected points, skipping PointCloud2 publishing.")
            return

        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.header.frame_id = "map"
        cloud_msg.height = 1  # Unordered point cloud
        cloud_msg.width = len(points)
        cloud_msg.is_dense = True
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 12  # 3 fields (x, y, z) * 4 bytes each
        cloud_msg.row_step = cloud_msg.point_step * len(points)

        # Define PointField format
        cloud_msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        data = []
        for x, y, z in points:
            data.append(struct.pack("fff", x, y, z))

        cloud_msg.data = b"".join(data)

        self.pointcloud_pub.publish(cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IPMExample()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
