#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose, BoundingBox2D, Point2D, Pose2D

class BBoxPublisher(Node):
    def __init__(self):
        super().__init__('bbox_publisher')
        self.publisher_ = self.create_publisher(Detection2DArray, '/creature_detection/bboxes', 1)
        self.timer = self.create_timer(1.0, self.publish_bboxes)
        
        self.label_path = "/workspaces/ros_jazzy/ros_ws/src/mowing_robot_bt/IPM_calibration/labels"
        self.image_width = 640
        self.image_height = 480
        self.class_id = "hedgehog"
        self.m_debug = True

    def get_sorted_label_files(self):
        """ Get all label files sorted numerically based on timestamp suffix. """
        files = [f for f in os.listdir(self.label_path) if f.endswith(".txt")]
        files.sort(key=lambda x: int(x.split("_")[-1].split(".")[0]))
        return files

    def yolo_to_bbox(self, class_id, x_center, y_center, width, height):
        """ Convert YOLO format (normalized) to absolute pixel values. """
        xmin = (x_center - width / 2) * self.image_width
        ymin = (y_center - height / 2) * self.image_height
        xmax = (x_center + width / 2) * self.image_width
        ymax = (y_center + height / 2) * self.image_height
        return xmin, ymin, xmax, ymax

    def publish_bboxes(self):
        """ Reads YOLO labels, converts to ROS2 messages, and publishes them. """
        files = self.get_sorted_label_files()
        if not files:
            self.get_logger().warn("No label files found.")
            return

        detection_array = Detection2DArray()
        detection_array.header = Header()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = "thermal_camera_frame"

        for file in files:
            with open(os.path.join(self.label_path, file), 'r') as f:
                for line in f:
                    parts = line.strip().split()
                    if len(parts) != 5:
                        continue
                    
                    class_idx, x_center, y_center, width, height = map(float, parts)
                    xmin, ymin, xmax, ymax = self.yolo_to_bbox(class_idx, x_center, y_center, width, height)

                    detection = Detection2D()
                    detection.header = detection_array.header

                    position_center = Point2D()
                    position_center.x = (xmin + xmax) / 2.0
                    position_center.y = (ymin + ymax) / 2.0

                    bbox_center = Pose2D()
                    bbox_center.position = position_center
                    bbox_center.theta = 0.0

                    bbox = BoundingBox2D()
                    bbox.center = bbox_center
                    bbox.size_x = xmax - xmin
                    bbox.size_y = ymax - ymin

                    detection.bbox = bbox 

                    # Classification result
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = self.class_id
                    hypothesis.hypothesis.score = 1.0  # Dummy value

                    detection.results.append(hypothesis)
                    detection_array.detections.append(detection)

                    if self.m_debug:
                        self.get_logger().info(f"Published bbox from {file}: Center=({position_center.x:.1f}, {position_center.y:.1f}), Size=({bbox.size_x:.1f}, {bbox.size_y:.1f})")

        self.publisher_.publish(detection_array)

def main(args=None):
    rclpy.init(args=args)
    node = BBoxPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
