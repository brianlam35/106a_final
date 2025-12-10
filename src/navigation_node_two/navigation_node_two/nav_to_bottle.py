#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

# Unitree SDK imports
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient


class NavToBottleNode(Node):
    def __init__(self):
        super().__init__("nav_to_bottle")

        # -----------------------------
        # Parameters
        # -----------------------------
        # Network interface used to talk to the Go2W (same as you pass to go2w_sport_client.py)
        self.declare_parameter("network_interface", "eth0")
        iface = self.get_parameter("network_interface").get_parameter_value().string_value

        # How close to stop in front of the bottle (meters)
        self.declare_parameter("stop_distance", 0.6)
        self.stop_distance = (
            self.get_parameter("stop_distance").get_parameter_value().double_value
        )

        # Forward walking speed (m/s)
        self.declare_parameter("walk_speed", 0.3)
        self.walk_speed = (
            self.get_parameter("walk_speed").get_parameter_value().double_value
        )

        # Yaw rotation speed (rad/s)
        self.declare_parameter("yaw_speed", 0.5)
        self.yaw_speed = (
            self.get_parameter("yaw_speed").get_parameter_value().double_value
        )

        # Min yaw angle that’s worth correcting (rad)
        self.yaw_deadband = 5.0 * math.pi / 180.0  # 5 degrees

        # -----------------------------
        # Unitree communication init
        # -----------------------------
        self.get_logger().info(f"Initializing Unitree channel on interface: {iface}")
        ChannelFactoryInitialize(0, iface)

        self.sport_client = SportClient()
        self.sport_client.SetTimeout(10.0)
        self.sport_client.Init()

        # Stand up & balance once at startup (optional but convenient)
        self.get_logger().info("Sending StandUp and BalanceStand...")
        try:
            self.sport_client.StandUp()
            time.sleep(1.0)
            self.sport_client.BalanceStand()
            time.sleep(0.5)
        except Exception as e:
            self.get_logger().warn(f"Stand sequence failed or was skipped: {e}")

        # -----------------------------
        # ROS interfaces
        # -----------------------------
        # NOTE: now subscribing to PoseStamped on /vision/target_pose
        self.sub_target = self.create_subscription(
            PoseStamped,
            "/vision/target_pose",
            self.target_callback,
            10,
        )

        # State flags so we only react once per detection
        self.target_locked = False
        self.executing = False

        self.get_logger().info("NavToBottleNode initialized and waiting for target pose...")

    # -----------------------------
    # Callback: get bottle pose
    # -----------------------------
    def target_callback(self, msg: PoseStamped):
        if self.executing:
            # Already navigating, ignore extra detections
            return

        if self.target_locked:
            # We already got a target and navigated; ignore subsequent detections
            return

        # PoseStamped: position is in msg.pose.position
        X = msg.pose.position.x  # left/right in camera frame
        Z = msg.pose.position.z  # forward distance in camera frame

        self.get_logger().info(f"Received bottle pose: X={X:.2f} m, Z={Z:.2f} m")

        # Lock this target and start open-loop motion
        self.target_locked = True
        self.executing = True

        try:
            self.navigate_to_target(X, Z)
        finally:
            self.executing = False
            self.get_logger().info("Navigation sequence complete.")

    # -----------------------------
    # Core navigation logic
    # -----------------------------
    def navigate_to_target(self, X: float, Z: float):
        """
        Use a single (X, Z) measurement in the camera frame to:
        1. Rotate in place to face the bottle.
        2. Walk straight until we’re stop_distance in front of it.
        Open-loop: we do NOT re-use updated vision data.
        """

        # Safety check: if Z is nonsense, abort
        if Z <= 0.0:
            self.get_logger().warn("Bottle Z <= 0, aborting navigation.")
            return

        # 1) Rotate to face the bottle
        # -----------------------------
        # In a standard camera frame: X is right, Z is forward.
        # Yaw error = angle between robot forward (Z axis) and the bottle.
        yaw_error = math.atan2(X, Z)  # rad

        self.get_logger().info(
            f"Computed yaw_error = {yaw_error:.3f} rad ({math.degrees(yaw_error):.1f} deg)"
        )

        if abs(yaw_error) > self.yaw_deadband:
            # Choose sign of yaw_rate based on direction of yaw_error
            yaw_rate_cmd = self.yaw_speed if yaw_error > 0 else -self.yaw_speed
            rotate_time = abs(yaw_error) / self.yaw_speed

            self.get_logger().info(
                f"Rotating with yaw_rate={yaw_rate_cmd:.2f} rad/s for {rotate_time:.2f} s"
            )
            self._send_move_for_duration(
                vx=0.0,
                vy=0.0,
                yaw_rate=yaw_rate_cmd,
                duration=rotate_time,
            )

            # Briefly stop and let the robot settle
            self.sport_client.StopMove()
            time.sleep(0.3)
        else:
            self.get_logger().info("Yaw error within deadband; skipping rotation.")

        # 2) Walk forward until stop_distance from bottle
        # -----------------------------
        forward_distance = Z - self.stop_distance
        if forward_distance <= 0.0:
            self.get_logger().info(
                f"Bottle is already closer than stop_distance ({self.stop_distance:.2f} m). No forward motion."
            )
            return

        forward_time = forward_distance / self.walk_speed
        self.get_logger().info(
            f"Walking forward {forward_distance:.2f} m "
            f"at {self.walk_speed:.2f} m/s for {forward_time:.2f} s"
        )

        self._send_move_for_duration(
            vx=self.walk_speed,
            vy=0.0,
            yaw_rate=0.0,
            duration=forward_time,
        )

        self.sport_client.StopMove()
        self.get_logger().info("Arrived in front of bottle.")

    # -----------------------------
    # Helper to send Move commands
    # -----------------------------
    def _send_move_for_duration(self, vx: float, vy: float, yaw_rate: float, duration: float):
        """
        Send SportClient.Move commands in a tight loop for 'duration' seconds.
        This is open-loop and DOES block the node while it runs.

        vx, vy: linear velocities in m/s
        yaw_rate: rotational speed in rad/s (sign = direction)
        """
        start = time.time()
        period = 0.05  # 20 Hz command rate

        while time.time() - start < duration:
            # WARNING: open-loop; make sure the area around the robot is clear.
            self.sport_client.Move(vx, vy, yaw_rate)
            time.sleep(period)


def main(args=None):
    rclpy.init(args=args)
    node = NavToBottleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down nav_to_bottle node.")
    finally:
        node.sport_client.StopMove()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

