import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('arm_aruco')

        # --- CONFIGURATION ---
        self.MARKER_SIZE = 0.02  # 3cm (Measure your tag!)
        self.TARGET_ID = 0       # Your tag ID
        self.ARUCO_DICT = cv2.aruco.DICT_4X4_50

        # Setup Aruco
        self.dictionary = cv2.aruco.getPredefinedDictionary(self.ARUCO_DICT)
        self.parameters = cv2.aruco.DetectorParameters()
        
        # Bridge
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

        # Pubs/Subs
        self.sub_info = self.create_subscription(
            CameraInfo, 
            '/camera/color/camera_info', 
            self.info_cb, 
            qos_profile_sensor_data
        )

        self.sub_img = self.create_subscription(
            Image, 
            '/camera/color/image_raw', 
            self.image_cb, 
            qos_profile_sensor_data
        )

        # Output Topic
        self.pub_target = self.create_publisher(PoseStamped, '/aruco_target', 10)

        self.get_logger().info("ArUco Detector Ready. Waiting for Camera Info...")

    def info_cb(self, msg):
        if self.camera_matrix is None:
            k = np.array(msg.k).reshape((3, 3))
            d = np.array(msg.d)
            self.camera_matrix = k
            self.dist_coeffs = d
            self.get_logger().info("Camera Intrinsics Received!")

    def image_cb(self, msg):
        if self.camera_matrix is None: return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # NOTE: If you are still getting 'detectMarkers' error, 
            # the issue is still your OpenCV install, not this code.
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray, self.dictionary, parameters=self.parameters)

            if ids is not None and self.TARGET_ID in ids:
                index = np.where(ids == self.TARGET_ID)[0][0]
                
                # Estimate Pose (tvecs is the translation/position vector)
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.MARKER_SIZE, self.camera_matrix, self.dist_coeffs)
                
                # Extract the X, Y, Z from the translation vector (in meters)
                tvec = tvecs[index][0] 
                cam_x = tvec[0]
                cam_y = tvec[1]
                cam_z = tvec[2]

                # --- NEW DEBUGGING OUTPUT ---
                self.get_logger().info(f"Target Found: X={cam_x:.3f} | Y={cam_y:.3f} | Z={cam_z:.3f} (m)")
                # ----------------------------

                # Publish PoseStamped message
                pose_msg = PoseStamped()
                pose_msg.header = msg.header
                pose_msg.pose.position.x = cam_x
                pose_msg.pose.position.y = cam_y
                pose_msg.pose.position.z = cam_z
                pose_msg.pose.orientation.w = 1.0 

                self.pub_target.publish(pose_msg)
                
        except Exception as e:
            # Added a specific check for a common error structure
            if 'detectMarkers' in str(e):
                 self.get_logger().warn("CV Error: OpenCV Aruco module missing. Re-install using 'pip install opencv-contrib-python'.")
            else:
                 self.get_logger().warn(f"CV Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ArucoDetector())
    rclpy.shutdown()