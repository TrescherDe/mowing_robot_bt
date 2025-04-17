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

from geometry_msgs.msg import PointStamped
from tf2_ros import TransformException

import numpy as np
import math

class IPMExample(Node):
    def __init__(self):
        super().__init__('IPM')

        self.pointcloud_pub = self.create_publisher(PointCloud2, '/eduard/fred/detected_creatures', 1)
        self.m_debug = False

        self.tf_buffer = tf2.Buffer()
        self.tf_listener = tf2.TransformListener(self.tf_buffer, self, spin_thread=True)

        self.camera_info = None
        self.ipm = None

        self.create_subscription(CameraInfo, '/camera_info', self.cameraInfoCallback, 1)
        self.create_subscription(Detection2DArray, '/creature_detection/bboxes', self.bboxCallback, 1)

        # Define the ground plane (normal is in z-direction)
        self.plane = Plane()
        self.plane.coef = [0.0, 0.0, -1.0, -0.04] # is defined from base link -> the map is 4cm below base_link

    def cameraInfoCallback(self, msg: CameraInfo):
        """Updates camera parameters when receiving /camera_info."""
        self.camera_info = msg

        self.ipm = IPM(self.tf_buffer, self.camera_info, distortion=True)
        self.get_logger().info("Received CameraInfo and initialized IPM.")

    def bboxCallback(self, msg: Detection2DArray):
        """Processes bounding boxes and projects them to world coordinates."""
        if self.ipm is None:
            self.get_logger().warn("No CameraInfo received yet. Cannot perform IPM.")
            return
        
        if len(msg.detections) == 0 or (len(msg.detections) == 1 and msg.detections[0].results == []):
            self.get_logger().warn("Received an empty Detection2DArray, skipping IPM projection.")
            return

        ipm_calibration = False
        project_bboxes_instead_of_plane=True

        if(project_bboxes_instead_of_plane):
            points = []
            for detection in msg.detections:
                bbox = detection.bbox
                x_center = bbox.center.position.x
                y_center = bbox.center.position.y                

                if ipm_calibration:
                    # Adjust y position to 1/3 of the height from the bottom
                    bbox_height = bbox.size_y
                    y_adjusted = y_center + (bbox_height / 6)
                    y_center = y_adjusted

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
                        plane_frame_id="eduard/fred/base_link",  # Project in base_link because its parallel to the camera
                        output_frame_id="eduard/fred/base_link"
                    )

                    self.get_logger().info(f"Creature detected at image coordinate X={x_center}, Y={y_center}")
                    self.get_logger().info(f"Projected Creature at X={projected_point.point.x:.2f}, Y={projected_point.point.y:.2f}, Z={projected_point.point.z:.2f}")
                    # Store as a tuple (X, Y, Z)
                    points.append((projected_point.point.x - 0.1763, projected_point.point.y, 0.0)) # MBE correction for detections below 2m

                except NoIntersectionError:
                    self.get_logger().warn("No valid intersection found for IPM.")

            if ipm_calibration:
                # Transform projected points to thermal camera frame for comparison
                transformed_points = self.transformPointsToFrame(points)
                self.computeMae(transformed_points)

            self.publishPointcloud(points, msg.header, radius=0.15, num_circle_points=16)

        else:
            # debug camera projection plane
            points = np.meshgrid(np.arange(0, self.camera_info.width, 10), np.arange(0, self.camera_info.height, 10))
            points = np.stack(points, axis=-1).reshape(-1, 2)
            current_time = self.get_clock().now().to_msg()

            header, mapped_points = self.ipm.map_points(
                self.plane,
                points,
                current_time,
                plane_frame_id='eduard/fred/base_link',
                output_frame_id='eduard/fred/base_link'
            )

            # Convert the NumPy array into a point cloud message so we can publish it for visualization
            point_cloud = create_cloud_xyz32(header, mapped_points)
            self.pointcloud_pub.publish(point_cloud)

    def publishPointcloud(self, points, header, radius=0.15, num_circle_points=16):
        """Converts a list of (x, y, z) points into a PointCloud2 message with circular area points."""
        if not points:
            self.get_logger().warn("No valid projected points, skipping PointCloud2 publishing.")
            return

        # Transform points from base_link to map frame
        transformed_points = self.transformPointsToFrame(points, "eduard/fred/base_link", "eduard/fred/map")
        if not transformed_points:
            self.get_logger().warn("No valid transformed points, skipping PointCloud2 publishing.")
            return

        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.header.frame_id = "eduard/fred/map"
        cloud_msg.height = 1  # Unordered point cloud

        cloud_points = []

        for x, y, z in transformed_points:
            # Add the center point (already transformed)
            cloud_points.append((x, y, z))

            # Add circular points around the center
            for i in range(num_circle_points):
                angle = 2 * math.pi * i / num_circle_points
                dx = radius * math.cos(angle)
                dy = radius * math.sin(angle)
                cloud_points.append((x + dx, y + dy, z))

        cloud_msg.width = len(cloud_points)
        cloud_msg.is_dense = True
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 12  # 3 fields (x, y, z) * 4 bytes each
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width

        # Define PointField format
        cloud_msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        data = []
        for px, py, pz in cloud_points:
            data.append(struct.pack("fff", px, py, pz))

        cloud_msg.data = b"".join(data)

        self.pointcloud_pub.publish(cloud_msg)

    def transformPointsToFrame(self, points, frame_from="eduard/fred/base_link", frame_to="eduard/fred/map"):
        """Transforms a list of points from 'frame_from' to 'frame_to' using TF2, ensuring time validity."""
        transformed_points = []

        for p in points:
            point_stamped = PointStamped()
            point_stamped.header.frame_id = frame_from
            point_stamped.header.stamp = rclpy.time.Time().to_msg()  # Use latest available time
            point_stamped.point.x = p[0]
            point_stamped.point.y = p[1]
            point_stamped.point.z = p[2]

            try:
                transformed_point = self.tf_buffer.transform(point_stamped, frame_to, timeout=rclpy.duration.Duration(seconds=1.0))
                transformed_points.append((transformed_point.point.x, transformed_point.point.y, transformed_point.point.z))
            except TransformException as e:
                self.get_logger().warn(f"TF transform failed for point ({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f}): {e}")
                continue

        return transformed_points

    def computeMae(self, projected_points):
        """Computes and prints the Mean Absolute Error (MAE) for projected points."""
        # Define the expected distances in meters (already in thermal camera frame)
        expected_distances = np.arange(0.55, 3.75, 0.2)  # 0.55, 0.75, 0.95, ..., 3.55

        # Extract the **Z values** (since in thermal camera frame, forward direction is Z)
        projected_z_values = np.array([p[2] for p in projected_points])

        # Ensure we have the same number of expected and predicted values
        min_length = min(len(expected_distances), len(projected_z_values))
        expected_values = expected_distances[:min_length]
        projected_z_values = projected_z_values[:min_length]

        # Compute absolute errors
        errors = np.abs(expected_values - projected_z_values)

        # Split into two categories: distances < 2.0m and distances ≥ 2.0m
        mask_below_2m = expected_values < 2.0
        mask_above_2m = expected_values >= 2.0

        # Compute MAE for each range
        mae_below_2m = np.mean(errors[mask_below_2m]) if np.any(mask_below_2m) else 0.0
        mae_above_2m = np.mean(errors[mask_above_2m]) if np.any(mask_above_2m) else 0.0
        mae_overall = np.mean(errors)

        # Log results
        for i, (expected, predicted, error) in enumerate(zip(expected_values, projected_z_values, errors)):
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
