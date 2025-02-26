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

from sensor_msgs_py.point_cloud2 import create_cloud_xyz32

import numpy as np
class IPMExample(Node):
    def __init__(self):
        super().__init__('IPM')

        self.pointcloud_pub = self.create_publisher(PointCloud2, '/eduard/fred/detected_creatures', 1)
        self.m_debug = False

        self.tf_buffer = tf2.Buffer()
        self.tf_listener = tf2.TransformListener(self.tf_buffer, self, spin_thread=True)

        self.camera_info = None
        self.ipm = None

        self.create_subscription(CameraInfo, '/camera_info', self.camera_info_callback, 1)
        self.create_subscription(Detection2DArray, '/creature_detection/bboxes', self.bbox_callback, 1)

        # Define the ground plane (normal is in z-direction)
        self.plane = Plane()
        self.plane.coef = [0.0, 0.0, -1.0, 0.0] # is defined in map frame

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
        
        project_bboxes_instead_of_plane=True
        if(project_bboxes_instead_of_plane):
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
                try:
                    projected_point = self.ipm.map_point(
                        self.plane,
                        Point2D(x=x_center, y=y_center),
                        rclpy.time.Time(),  # Use the latest available time
                        plane_frame_id="eduard/fred/map",
                        output_frame_id="eduard/fred/map"
                    )
                    self.get_logger().info(f"Creature detected at image coordinate X={x_center}, Y={y_center}")
                    self.get_logger().info(f"Projected Creature at X={projected_point.point.x:.2f}, Y={projected_point.point.y:.2f}, Z={projected_point.point.z:.2f}")
                    # Store as a tuple (X, Y, Z)
                    points.append((projected_point.point.x, projected_point.point.y, projected_point.point.z))

                except NoIntersectionError:
                    self.get_logger().warn("No valid intersection found for IPM.")
            self.compute_mae(points)
            self.publish_pointcloud(points, msg.header)
        else:
            # debug camera projection plane
            points = np.meshgrid(np.arange(0, self.camera_info.width, 10), np.arange(0, self.camera_info.height, 10))
            points = np.stack(points, axis=-1).reshape(-1, 2)
            current_time = self.get_clock().now().to_msg()

            header, mapped_points = self.ipm.map_points(
                self.plane,
                points,
                current_time,
                plane_frame_id='eduard/fred/map',
                output_frame_id='eduard/fred/map'
            )

            # Convert the NumPy array into a point cloud message so we can publish it for visualization
            point_cloud = create_cloud_xyz32(header, mapped_points)
            self.pointcloud_pub.publish(point_cloud)

    def publish_pointcloud(self, points, header):
        """Converts a list of (x, y, z) points into a PointCloud2 message."""
        if not points:
            self.get_logger().warn("No valid projected points, skipping PointCloud2 publishing.")
            return

        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.header.frame_id = "eduard/fred/map"
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

    def compute_mae(self, projected_points):
        """Computes and prints the Mean Absolute Error (MAE) for projected points."""
        # Define the expected distances in meters
        expected_distances = np.arange(0.4, 3.6, 0.2)  # 0.4, 0.6, 0.8, ..., 3.4
        projected_x_values = np.array([p[0] for p in projected_points])

        # Ensure we have the same number of expected and predicted values
        min_length = min(len(expected_distances), len(projected_x_values))
        expected_values = expected_distances[:min_length]
        projected_x_values = projected_x_values[:min_length]

        errors = np.abs(expected_values - projected_x_values)

        # Split into two categories: distances < 2.0m and distances >= 2.0m
        mask_below_2m = expected_values < 2.0
        mask_above_2m = expected_values >= 2.0

        # Compute MAE for each range
        mae_below_2m = np.mean(errors[mask_below_2m]) if np.any(mask_below_2m) else 0.0
        mae_above_2m = np.mean(errors[mask_above_2m]) if np.any(mask_above_2m) else 0.0
        mae_overall = np.mean(errors)
        
        for i, (expected, predicted, error) in enumerate(zip(expected_values, projected_x_values, errors)):
            self.get_logger().info(f"Point {i}: Expected={expected:.2f}m, Predicted={predicted:.2f}m, Error={error:.2f}m")

        self.get_logger().info(f"MAE (< 2m): {mae_below_2m:.2f}m | MAE (≥ 2m): {mae_above_2m:.2f}m | Overall MAE: {mae_overall:.2f}m")


def main(args=None):
    rclpy.init(args=args)
    node = IPMExample()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
