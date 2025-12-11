import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32MultiArray
import numpy as np

# Import your helper class
from arm_control.ik_solver import IKSolver

class ArmManager(Node):
    def __init__(self):
        super().__init__('arm_manager')

        # 1. Initialize IK Solver 
        self.get_logger().info("Loading IK Solver...")
        self.ik = IKSolver()

        # 2. MANUAL CALIBRATION (Camera -> Arm)      
        # EXAMPLE: If Camera is 20cm IN FRONT of arm, x_offset = 0.2
        # EXAMPLE: If Camera is 10cm ABOVE arm, z_offset = 0.1
        self.cam_x_offset = 0.367  
        self.cam_y_offset = 0.034
        self.cam_z_offset = 0.0

        # 3. Setup Pub/Sub
        # Listen for the bottle position from the vision node
        self.sub_vision = self.create_subscription(
            PointStamped, 
            '/vision/target_pose',  # <--- UPDATED TOPIC NAME
            self.vision_callback, 
            10
        )
        
        # Publish commands to the arm driver
        self.pub_joints = self.create_publisher(
            Float32MultiArray, 
            '/arm_joint_commands', 
            10
        )

        self.get_logger().info("Arm Manager Ready! Waiting for /vision/target_pose...")

    def vision_callback(self, msg):
        try:
            # --- A. MANUAL TRANSFORM (Camera Frame -> Arm Frame) ---
            # We assume the camera and arm are aligned (x=forward, z=up).
            # If the camera is tilted, this math gets harder (requires rotation matrix).
            # For now, we just add the linear offset.
            
            # P_arm = P_camera + Offset
            target_x = msg.point.x + self.cam_x_offset
            target_y = msg.point.y + self.cam_y_offset
            target_z = msg.point.z + self.cam_z_offset
            
            self.get_logger().info(f"Vision Target: ({msg.point.x:.2f}, {msg.point.y:.2f}, {msg.point.z:.2f})")
            self.get_logger().info(f"Arm Target:    ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")

            # --- B. COMPUTE IK ---
            joint_angles = self.ik.compute_ik(target_x, target_y, target_z)
            
            # --- C. PUBLISH COMMAND ---
            cmd = Float32MultiArray()
            
            # 1.5 = Open Gripper (Ready to grab)
            # -1.5 = Close Gripper
            gripper_cmd = 1.5 
            
            # Combine [6 angles] + [1 gripper]
            cmd.data = list(joint_angles) + [gripper_cmd]
            
            self.pub_joints.publish(cmd)
            self.get_logger().info("Move Command Sent!")

        except Exception as e:
            self.get_logger().error(f"Logic Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ArmManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()