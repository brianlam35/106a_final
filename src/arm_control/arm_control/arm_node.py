import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data
import numpy as np

from arm_control.ik_solver import IKSolver

class ArmManager(Node):
    def __init__(self):
        super().__init__('arm_manager')
        self.ik = IKSolver()
        
        # --- OFFSETS ---
        self.cam_x_offset = 0.2
        self.cam_y_offset = 0.0
        self.cam_z_offset = 0.1

        # --- HARDWARE CALIBRATION SHIM ---
        # Positive = Shifts Target LEFT relative to the robot
        # Negative = Shifts Target RIGHT relative to the robot
        self.y_bias = -0.04  # <--- ADJUST THIS TO CALIBRATE

        # State Variables
        self.current_joints = []
        self.target_joints = []
        self.state = "IDLE" 
        self.start_time = 0

        # --- SUBSCRIBERS ---
        # 1. Vision (We support both PointStamped and PoseStamped)
        self.sub_vision_pt = self.create_subscription(
            PointStamped, '/vision/target_pose', self.vision_cb_point, 10)
            
        self.sub_vision_pose = self.create_subscription(
            PoseStamped, '/vision/target_pose', self.vision_cb_pose, qos_profile_sensor_data)

        # 2. Joint State
        self.sub_state = self.create_subscription(
            JointState, '/arm_joint_states', self.state_cb, 10)
            
        # --- PUBLISHER ---
        self.pub_joints = self.create_publisher(Float32MultiArray, '/arm_joint_commands', 10)
        
        # Control Loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Arm Manager Ready (Immediate Mode). Waiting for vision...")

    def state_cb(self, msg):
        self.current_joints = list(msg.position)

    # Handler for PointStamped (Matches your manual test)
    def vision_cb_point(self, msg):
        # DIRECT MAPPING (Matches your original script)
        # X -> Forward
        # Y -> Left/Right
        # Z -> Up/Down
        self.process_target(msg.point.x, msg.point.y, msg.point.z)

    # Handler for PoseStamped (Matches your original script logic)
    def vision_cb_pose(self, msg):
        self.process_target(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def process_target(self, msg_x, msg_y, msg_z):
        if self.state != "IDLE": return # Ignore if busy
        
        # --- COORDINATE MAPPING (Reverted to your original) ---
        tx = msg_x + self.cam_x_offset
        ty = msg_y + self.cam_y_offset + self.y_bias # <--- BIAS APPLIED HERE
        tz = msg_z + self.cam_z_offset
        
        # --- DEBUG PRINT ---
        self.get_logger().info(f"DEBUG: Bias={self.y_bias}. InputX={msg_x:.2f} -> TargetX={tx:.2f}, TargetY={ty:.3f}")
        
        try:
            self.approach_angles = list(self.ik.compute_ik(tx, ty, tz))
            self.lift_angles     = list(self.ik.compute_ik(tx, ty, tz + 0.15))

            # self.get_logger().info("Target Calculated. Starting Approach...")
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
        if self.state == "IDLE":
            pass

        elif self.state == "START_APPROACH":
            self.send_cmd(self.approach_angles, 1.5) # Open
            self.state = "APPROACHING"
            
        elif self.state == "APPROACHING":
            if self.is_at_target():
                # self.get_logger().info("Reached Target. Gripping...")
                self.state = "START_GRIP"
        
        elif self.state == "START_GRIP":
            self.send_cmd(self.approach_angles, -1.5) # Close
            self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
            self.state = "GRIPPING"
            
        elif self.state == "GRIPPING":
            if (self.get_clock().now().seconds_nanoseconds()[0] - self.start_time) > 3:
                # self.get_logger().info("Grip Complete. Lifting...")
                self.state = "START_LIFT"
        
        elif self.state == "START_LIFT":
            self.send_cmd(self.lift_angles, -1.5) # Keep Closed
            self.state = "LIFTING"

        elif self.state == "LIFTING":
            if self.is_at_target():
                self.get_logger().info("Lift Complete! Resetting.")
                self.state = "IDLE"

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ArmManager())
    rclpy.shutdown()