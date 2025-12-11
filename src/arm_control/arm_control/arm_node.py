import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
import numpy as np

from arm_control.ik_solver import IKSolver

class ArmManager(Node):
    def __init__(self):
        super().__init__('arm_manager')
        self.ik = IKSolver()
        
        # Offsets
        self.cam_x_offset, self.cam_y_offset, self.cam_z_offset = 0.2, 0.0, 0.1

        # --- NEW: HARDWARE CALIBRATION SHIM ---
        # If the arm consistently misses to the Left, put a negative number here.
        # If it misses to the Right, put a positive number.
        # Start with 2cm (0.02) and tune it.
        self.y_bias = 0.2  # Example: Adjust this until it hits dead center!

        # State Variables
        self.current_joints = []
        self.target_joints = []
        self.state = "IDLE" # IDLE, MOVING, GRIPPING, LIFTING, DONE
        self.start_time = 0

        # Communication
        self.sub_vision = self.create_subscription(PointStamped, '/vision/target_pose', self.vision_cb, 10)
        self.sub_state = self.create_subscription(JointState, '/arm_joint_states', self.state_cb, 10)
        self.pub_joints = self.create_publisher(Float32MultiArray, '/arm_joint_commands', 10)
        
        # 10Hz Control Loop (Checks "Are we there yet?")
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Arm Manager Ready (Closed Loop).")

    def state_cb(self, msg):
        self.current_joints = list(msg.position) # Update current angles (radians)

    def vision_cb(self, msg):
        if self.state != "IDLE": return # Ignore if busy
        
        # Calculate Approach Target
        tx = msg.point.x + self.cam_x_offset
        ty = msg.point.y + self.cam_y_offset
        tz = msg.point.z + self.cam_z_offset
        
        self.approach_angles = list(self.ik.compute_ik(tx, ty, tz))
        
        # Calculate Lift Target (Same XY, Higher Z)
        self.lift_angles = list(self.ik.compute_ik(tx, ty, tz + 0.1))

        self.get_logger().info("Target Received. Starting Approach...")
        self.state = "START_APPROACH"

    def send_cmd(self, angles, gripper):
        cmd = Float32MultiArray()
        cmd.data = angles + [gripper]
        self.pub_joints.publish(cmd)
        self.target_joints = angles # Store for error checking

    def is_at_target(self, threshold=0.1):
        if not self.current_joints or not self.target_joints: return False
        
        # Check error for first 6 joints (ignore gripper for now)
        error = 0.0
        for i in range(6):
            error += abs(self.current_joints[i] - self.target_joints[i])
        return error < threshold

    def control_loop(self):
        # State Machine Logic
        if self.state == "IDLE":
            pass

        elif self.state == "START_APPROACH":
            self.send_cmd(self.approach_angles, 1.5) # Open
            self.state = "APPROACHING"
            
        elif self.state == "APPROACHING":
            if self.is_at_target():
                self.get_logger().info("Reached Target. Gripping...")
                self.state = "START_GRIP"
        
        elif self.state == "START_GRIP":
            self.send_cmd(self.approach_angles, -1.5) # Close
            self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
            self.state = "GRIPPING"
            
        elif self.state == "GRIPPING":
            # Gripper doesn't give good position feedback (stall), so we use time here
            if (self.get_clock().now().seconds_nanoseconds()[0] - self.start_time) > 3:
                self.get_logger().info("Grip Complete. Lifting...")
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