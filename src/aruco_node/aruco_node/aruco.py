#!/usr/bin/env python3

import cv2
import cv2.aruco as aruco
import numpy as np

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped


class ArucoNode(Node):
    def __init__(self):
        super().__init__("aruco_node")

        self.bridge = CvBridge()

        # Camera intrinsics
        self.K = None   # fx,0,cx,0,fy,cy,0,0,1

        # Image buffers
        self.rgb = None
        self.depth = None

        # Set the dictionary and target marker ID
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()
        self.target_id = 0   # We printed marker ID = 0

        # Subscribers
        self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self.rgb_callback,
            10
        )

        self.create_subscription(
            Image,
            "/camera/aligned_depth_to_color/image_raw",
            self.depth_callback,
            10
        )

        self.create_subscription(
            CameraInfo,
            "/camera/color/camera_info",
            self.caminfo_callback,
            10
        )

        # Publisher
        self.home_pub = self.create_publisher(
            PointStamped,
            "/vision/home_xyz",
            10
        )

        self.get_logger().info("ArucoNode is running. Looking for DICT_4X4_50, ID = 0")

    # ---------------------------
    # Camera Info callback
    # ---------------------------
    def caminfo_callback(self, msg):
        self.K = msg.k

    # ---------------------------
    # RGB callback
    # ---------------------------
    def rgb_callback(self, msg):
        self.rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.try_process()

    # ---------------------------
    # Depth callback
    # ---------------------------
    def depth_callback(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        self.try_process()

    # ---------------------------
    # Main processing
    # ---------------------------
    def try_process(self):
        if self.rgb is None or self.depth is None or self.K is None:
            return

        gray = cv2.cvtColor(self.rgb, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_params
        )

        if ids is None:
            return

        ids = ids.flatten()

        # Find our specific marker ID = 0
        for i, marker_id in enumerate(ids):
            if marker_id == self.target_id:
                pts = corners[i][0]   # 4x2 corner array

                # Compute marker center (avg of opposite corners)
                cx = int((pts[0][0] + pts[2][0]) / 2)
                cy = int((pts[0][1] + pts[2][1]) / 2)

                # Check depth image bounds
                h, w = self.depth.shape
                if not (0 <= cx < w and 0 <= cy < h):
                    return

                depth_raw = float(self.depth[cy, cx])
                if depth_raw <= 0 or np.isnan(depth_raw):
                    return

                depth_m = depth_raw * 0.001   # mm → m

                X, Y, Z = self.pixel_to_3d(cx, cy, depth_m)

                # Publish result
                msg = PointStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "camera_link"
                msg.point.x = X
                msg.point.y = Y
                msg.point.z = Z

                self.home_pub.publish(msg)

                self.get_logger().info(
                    f"Aruco ID {marker_id} at X={X:.2f}, Y={Y:.2f}, Z={Z:.2f}"
                )
                return

    # ---------------------------
    # Pixel → 3D conversion
    # ---------------------------
    def pixel_to_3d(self, u, v, depth):
        fx, fy, cx, cy = self.K[0], self.K[4], self.K[2], self.K[5]
        X = (u - cx) * depth / fx
        Y = (v - cy) * depth / fy
        Z = depth
        return X, Y, Z


def main(args=None):
    rclpy.init(args=args)
    node = ArucoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()



