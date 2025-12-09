#!/usr/bin/env python3
import os
os.environ["PYTORCH_NO_CUDA_MEMORY_CACHING"] = "1"
os.environ["CUDA_VISIBLE_DEVICES"] = ""  # force CPU to avoid TLS memory issues

import torch
torch.set_num_threads(1)

import rclpy
from rclpy.node import Node

import cv2
import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped

from ultralytics import YOLO


class PingPongNode(Node):
    def __init__(self):
        super().__init__("pingpong_node")
        self.bridge = CvBridge()

        # Load YOLOv8n pretrained on COCO, force CPU
        try:
            self.model = YOLO("yolov8n.pt")
            self.model.to("cpu")
            self.get_logger().info("YOLOv8n model loaded on CPU.")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            raise e

        # Camera intrinsics
        self.K = None

        # Image buffers
        self.rgb_image = None
        self.depth_image = None

        # Subscribers
        self.create_subscription(Image, "/camera/color/image_raw", self.rgb_cb, 10)
        self.create_subscription(Image, "/camera/aligned_depth_to_color/image_raw", self.depth_cb, 10)
        self.create_subscription(CameraInfo, "/camera/color/camera_info", self.caminfo_cb, 10)

        # Publisher
        self.xyz_pub = self.create_publisher(PointStamped, "/vision/target_xyz", 10)

    # -----------------------------
    # Callbacks
    # -----------------------------
    def rgb_cb(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.try_process()

    def depth_cb(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        self.try_process()

    def caminfo_cb(self, msg):
        self.K = msg.k
        # self.get_logger().info("Camera intrinsics received.")

    # -----------------------------
    # Processing
    # -----------------------------
    def try_process(self):
        if self.rgb_image is None or self.depth_image is None or self.K is None:
            # self.get_logger().info("DOES NOT ENTER TRY PROCESS")
            return

        try:
            results = self.model.predict(
                source=self.rgb_image,
                conf=0.5,
                device="cpu"
            )
        except Exception as e:
            self.get_logger().error(f"YOLO prediction failed: {e}")
            return

        if len(results) == 0 or results[0].boxes is None:
            return

        boxes = results[0].boxes
        for i, cls in enumerate(boxes.cls):
            if int(cls) == 32:  # sports ball
                xyxy = boxes.xyxy[i].cpu().numpy()
                cx = int((xyxy[0] + xyxy[2]) / 2)
                cy = int((xyxy[1] + xyxy[3]) / 2)

                # ---- bounds check vs depth image ----
                h, w = self.depth_image.shape
                if cx < 0 or cx >= w or cy < 0 or cy >= h:
                    self.get_logger().warn(
                        f"Skipping ball: depth index ({cx},{cy}) out of bounds for depth size ({w},{h})"
                    )
                    continue

                depth_raw = float(self.depth_image[cy, cx])
                if depth_raw == 0 or np.isnan(depth_raw):
                    self.get_logger().info(
                        f"Skipping ball: invalid depth at ({cx},{cy}), value={depth_raw}"
                    )
                    continue

                depth = depth_raw * 0.001  # keep if depth topic is in mm

                X, Y, Z = self.pixel_to_3d(cx, cy, depth)
                pt_msg = PointStamped()
                pt_msg.header.frame_id = "camera_link"
                pt_msg.point.x = X
                pt_msg.point.y = Y
                pt_msg.point.z = Z
                self.xyz_pub.publish(pt_msg)
                self.get_logger().info(
                    f"Ping pong ball detected at ({X:.2f}, {Y:.2f}, {Z:.2f})"
                )
                break


    # -----------------------------
    # Utility
    # -----------------------------
    def pixel_to_3d(self, u, v, depth):
        fx, fy, cx, cy = self.K[0], self.K[4], self.K[2], self.K[5]
        X = (u - cx) * depth / fx
        Y = (v - cy) * depth / fy
        Z = depth
        return X, Y, Z


def main(args=None):
    rclpy.init(args=args)
    node = PingPongNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

