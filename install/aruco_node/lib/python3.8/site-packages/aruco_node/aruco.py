#!/usr/bin/env python3

import cv2
import cv2.aruco as aruco
import numpy as np

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped

# SYNC: Import message_filters for time synchronization
import message_filters

class ArucoNode(Node):
    def __init__(self):
        super().__init__("aruco_node")

        self.bridge = CvBridge()
        self.K = None 
        
        # ---------------------------------------------------------
        # OPENCV FIX: Setup ArucoDetector for OpenCV 4.7+
        # ---------------------------------------------------------
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()
        
        # Try-catch to handle different OpenCV versions automatically
        try:
            # New OpenCV (4.7+)
            self.detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.use_new_api = True
        except AttributeError:
            # Old OpenCV (<4.7)
            self.use_new_api = False

        self.target_id = 0

        # 1. Subscriber for Camera Info
        self.create_subscription(
            CameraInfo,
            "/camera/color/camera_info",
            self.caminfo_callback,
            10
        )

        # 2. Setup Subscribers using message_filters
        rgb_sub = message_filters.Subscriber(self, Image, "/camera/color/image_raw")
        depth_sub = message_filters.Subscriber(self, Image, "/camera/aligned_depth_to_color/image_raw")

        # 3. Synchronize the topics
        # ApproximateTimeSynchronizer ensures we get matching RGB and Depth frames
        self.ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.sync_callback)

        self.home_pub = self.create_publisher(PointStamped, "/vision/home_xyz", 10)

        self.get_logger().info(f"ArucoNode running. OpenCV Version: {cv2.__version__}")

    def caminfo_callback(self, msg):
        self.K = msg.k

    def sync_callback(self, rgb_msg, depth_msg):
        if self.K is None:
            return

        try:
            cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        self.process_frame(cv_rgb, cv_depth, rgb_msg.header)

    def process_frame(self, rgb, depth_img, header):
        gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)

        # ---------------------------------------------------------
        # OPENCV FIX: Use the correct detection method
        # ---------------------------------------------------------
        if self.use_new_api:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = aruco.detectMarkers(
                gray, 
                self.aruco_dict, 
                parameters=self.aruco_params
            )

        if ids is None:
            return

        ids = ids.flatten()

        for i, marker_id in enumerate(ids):
            if marker_id == self.target_id:
                pts = corners[i][0]
                
                # Calculate center
                cx = int((pts[0][0] + pts[2][0]) / 2)
                cy = int((pts[0][1] + pts[2][1]) / 2)

                # Bounds check
                h, w = depth_img.shape
                if not (0 <= cx < w and 0 <= cy < h):
                    continue

                depth_raw = depth_img[cy, cx]
                
                if depth_raw == 0 or np.isnan(depth_raw):
                    continue

                # Conversion: mm to meters
                depth_m = float(depth_raw) * 0.001 

                X, Y, Z = self.pixel_to_3d(cx, cy, depth_m)

                # Publish
                point_msg = PointStamped()
                point_msg.header = header 
                point_msg.point.x = X
                point_msg.point.y = Y
                point_msg.point.z = Z

                self.home_pub.publish(point_msg)
                
                # Log occasionally (every 50th frame or similar) to avoid spam, 
                # or just log once per detection:
                # self.get_logger().info(f"Detected ID {marker_id}: {X:.3f}, {Y:.3f}, {Z:.3f}")

    def pixel_to_3d(self, u, v, depth):
        fx, fy, cx, cy = self.K[0], self.K[4], self.K[2], self.K[5]
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth
        return x, y, z

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