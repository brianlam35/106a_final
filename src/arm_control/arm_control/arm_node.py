import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Float32MultiArray, String
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data
import numpy as np

from arm_control.ik_solver import IKSolver

class ArmManager(Node):
    def __init__(self):
        super().__init__('arm_manager')
        self.ik = IKSolver()
        
        # --- OFFSETS (Physical distance from Arm Base to Camera Lens) ---
        self.cam_x_offset = 0.23  # Forward/Back   0.38-0.12 = base to cam offset - calibration offset
        self.cam_y_offset = 0.04  # Left/Right
        self.cam_z_offset = 0.0   # Up/Down

        # --- TUNING: SCALING FACTORS ---
        self.x_scale = 1.9   
        self.y_scale = 0.667  # Your calibrated value
        self.z_scale = 1.0   

        # --- TUNING: BIAS ---
        self.y_bias = -0.06   #tweak this

        # --- ARUCO SETTINGS ---
        # Radius of the object (Water Bottle)
        # We add this to reach the CENTER of the bottle, not just the surface tag.
        self.BOTTLE_RADIUS = 0.02 # 2cm

        # State Variables
        self.current_joints = []
        self.target_joints = []
        self.state = "WAITING_FOR_START" # <--- NEW INITIAL STATE
        self.start_time = 0

        # --- SUBSCRIBERS ---
        
        # 1. The Trigger (Prerequisite)
        self.sub_start = self.create_subscription(
            String, '/arm_start', self.start_cb, 10)

        # 2. Vision (ArUco Pose)
        self.sub_vision_pose = self.create_subscription(
            PoseStamped, '/aruco_target', self.vision_cb_pose, qos_profile_sensor_data)

        # 3. Joint State
        self.sub_state = self.create_subscription(
            JointState, '/arm_joint_states', self.state_cb, 10)
            
        # --- PUBLISHER ---
        self.pub_joints = self.create_publisher(Float32MultiArray, '/arm_joint_commands', 10)
        
        # Control Loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Arm Manager Ready. Waiting for '/arm_start' signal...")

    def start_cb(self, msg):
        # Check if we are waiting AND if the message is correct
        if self.state == "WAITING_FOR_START" and msg.data == 'start':
            self.get_logger().info("Received START signal! Activating Arm...")
            self.state = "IDLE" # Now ready to process vision

    def state_cb(self, msg):
        self.current_joints = list(msg.position)

    def vision_cb_pose(self, msg):
        # X=Right, Y=Down, Z=Forward (Standard Camera Frame)
        self.process_target(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def process_target(self, cam_x, cam_y, cam_z):
        # Only process if we have received the start signal
        if self.state != "IDLE": return 
        
        # --- COORDINATE MAPPING ---
        
        # 1. Forward Reach (tx)
        # Cam Z (Depth) + Offset + Reach INSIDE the bottle (Radius)
        tx = (cam_z * self.x_scale) + self.cam_x_offset + self.BOTTLE_RADIUS
        
        # 2. Left/Right (ty)
        # -Cam X (Right is neg) + Offset + Bias
        ty = (-cam_x * self.y_scale) + self.cam_y_offset + self.y_bias
        
        # 3. Height (tz)
        # Cam Y (Down) + Offset. 
        # Since the tag is on the SIDE of the cap, this Z is the exact cap height.
        # We might subtract 5mm to ensure we don't hit the top lip.
        tz = (cam_y * self.z_scale) + self.cam_z_offset - 0.1
        
        self.get_logger().info(f"DEBUG: InputZ={cam_z:.2f} | Radius Added | Final Target: ({tx:.2f}, {ty:.2f}, {tz:.2f})")
        
        try:
            self.approach_angles = list(self.ik.compute_ik(tx, ty, tz))
            self.lift_angles     = list(self.ik.compute_ik(tx, ty, tz + 0.30))

            self.get_logger().info("Target Calculated. Starting Approach...")
            self.state = "START_APPROACH"
            
        except Exception as e:
            self.get_logger().warn(f"IK Failed: {e}")

    def send_cmd(self, angles, gripper):
        cmd = Float32MultiArray()
        cmd.data = angles + [gripper]
        self.pub_joints.publish(cmd)
        self.target_joints = angles 

    def is_at_target(self, threshold=0.15):
        if not self.current_joints or not self.target_joints: return False
        error = sum([abs(c - t) for c, t in zip(self.current_joints[:6], self.target_joints[:6])])
        return error < threshold

    def control_loop(self):
        if self.state == "WAITING_FOR_START":
            pass # Do nothing, wait for topic

        elif self.state == "IDLE":
            pass # Active, waiting for ArUco tag

        elif self.state == "START_APPROACH":
            self.send_cmd(self.approach_angles, 1.5) 
            self.state = "APPROACHING"
            
        elif self.state == "APPROACHING":
            if self.is_at_target():
                self.state = "START_GRIP"
        
        elif self.state == "START_GRIP":
            self.send_cmd(self.approach_angles, -1.5) 
            self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
            self.state = "GRIPPING"
            
        elif self.state == "GRIPPING":
            if (self.get_clock().now().seconds_nanoseconds()[0] - self.start_time) > 3:
                self.state = "START_LIFT"
        
        elif self.state == "START_LIFT":
            self.send_cmd(self.lift_angles, -1.5)
            self.state = "LIFTING"

        elif self.state == "LIFTING":
            now = self.get_clock().now().seconds_nanoseconds()[0]
            is_timed_out = (now - self.start_time) > 5.0 

            if self.is_at_target() or is_timed_out:
                self.get_logger().info("Lift Complete! Disabling further action.")
                self.state = "TASK_COMPLETE"

        elif self.state == "TASK_COMPLETE":
            pass # Arm is done. Ignore all vision data and start signals.

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ArmManager())
    rclpy.shutdown()