#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
from sensor_msgs.msg import JointState


class ArmGripperRelease(Node):
    def __init__(self):
        super().__init__('arm_gripper_release')

        self.current_joints = None
        self.released = False

        # Joint states (optional)
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_cb,
            10
        )

        # Tag reached event
        self.create_subscription(
            Bool,
            '/vision/tag_reached',
            self.tag_reached_cb,
            10
        )

        # Arm command publisher
        self.pub_arm = self.create_publisher(
            Float32MultiArray,
            '/arm_joint_commands',
            10
        )

        self.get_logger().info(
            "ArmGripperRelease READY — waiting for /vision/tag_reached"
        )

    def joint_state_cb(self, msg: JointState):
        self.current_joints = list(msg.position)

    def tag_reached_cb(self, msg: Bool):
        if not msg.data or self.released:
            return

        self.get_logger().info("Tag reached signal received!")

        arm_msg = Float32MultiArray()

        if self.current_joints is not None and len(self.current_joints) >= 7:
            # Preserve current joints
            arm_msg.data = self.current_joints.copy()
            arm_msg.data[6] = 1.5
            self.get_logger().info("Using current joint states for release.")
        else:
            # Fallback: safe default pose
            arm_msg.data = [0.0] * 6 + [1.5]
            self.get_logger().warn(
                "No valid joint_states — using fallback release command."
            )

        self.pub_arm.publish(arm_msg)
        self.released = True

        self.get_logger().info("✅ Gripper release command sent.")


def main(args=None):
    rclpy.init(args=args)
    node = ArmGripperRelease()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
