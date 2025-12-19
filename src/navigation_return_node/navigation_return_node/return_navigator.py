#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Bool
import math
import time


class ReturnNavigator(Node):
    def __init__(self):
        super().__init__('return_navigator')

        # --- TUNING ---
        self.TARGET_DIST = 0.3
        self.ALIGN_SPEED = 0.20
        self.DRIVE_SPEED = 0.3
        self.SEARCH_SPIN_SPEED = -0.25
        self.SAMPLES_NEEDED = 3

        # --- STATE ---
        self.state = "SEARCHING"
        self.samples = []

        self.move_start_time = 0.0
        self.move_duration = 0.0
        self.turn_direction = 0
        self.target_drive_dist = 0.0
        self.tag_reached_sent = False

        # --- PUBLISHERS ---
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_tag_reached = self.create_publisher(Bool, '/vision/tag_reached', 10)

        # --- SUBSCRIBER ---
        self.sub_home = self.create_subscription(
            PointStamped,
            '/vision/home_xyz',
            self.vision_cb,
            qos_profile_sensor_data
        )

        # --- TIMER ---
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("ReturnNavigator READY — spinning to search for ArUco.")


    # ============================================================
    # VISION CALLBACK
    # ============================================================
    def vision_cb(self, msg: PointStamped):
        if self.state in ("ALIGNING", "MOVING_BLIND", "DONE"):
            return

        depth = msg.point.z
        x = msg.point.x

        if depth <= 0.0:
            return

        angle = math.atan2(-x, depth)

        if self.state == "SEARCHING":
            self.get_logger().info("Aruco spotted — sampling...")
            self.state = "SAMPLING"
            self.samples = []

        if self.state == "SAMPLING" and len(self.samples) < self.SAMPLES_NEEDED:
            self.samples.append((depth, angle))
            self.get_logger().info(
                f"Sample {len(self.samples)}/{self.SAMPLES_NEEDED}: "
                f"d={depth:.2f}m  a={math.degrees(angle):.1f}deg"
            )


    # ============================================================
    # MAIN CONTROL LOOP
    # ============================================================
    def control_loop(self):
        if self.state == "DONE":
            return

        cmd = Twist()
        now = time.time()

        # ---------------- SEARCH ----------------
        if self.state == "SEARCHING":
            cmd.linear.x = 0.0
            cmd.angular.z = self.SEARCH_SPIN_SPEED
            self.pub_vel.publish(cmd)

        # ---------------- SAMPLING ----------------
        elif self.state == "SAMPLING":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.pub_vel.publish(cmd)

            if len(self.samples) >= self.SAMPLES_NEEDED:
                self.plan_approach()

        # ---------------- ALIGN ----------------
        elif self.state == "ALIGNING":
            if (now - self.move_start_time) < self.move_duration:
                cmd.angular.z = self.ALIGN_SPEED * self.turn_direction
                self.pub_vel.publish(cmd)
            else:
                self.get_logger().info("Aligned — driving forward.")
                self.start_blind_move()

        # ---------------- DRIVE ----------------
        elif self.state == "MOVING_BLIND":
            if (now - self.move_start_time) < self.move_duration:
                cmd.linear.x = self.DRIVE_SPEED
                self.pub_vel.publish(cmd)
            else:
                self.finish_task()


    # ============================================================
    # PLAN TURN + DRIVE
    # ============================================================
    def plan_approach(self):
        avg_depth = sum(s[0] for s in self.samples) / len(self.samples)
        avg_angle = sum(s[1] for s in self.samples) / len(self.samples)

        turn_seconds = abs(avg_angle) / self.ALIGN_SPEED
        turn_seconds = max(turn_seconds, 0.4)

        self.turn_direction = 1 if avg_angle > 0 else -1
        self.target_drive_dist = max(0.0, avg_depth - self.TARGET_DIST)

        self.get_logger().info(
            f"PLAN → turn {math.degrees(avg_angle):.1f}deg "
            f"({turn_seconds:.2f}s), drive {self.target_drive_dist:.2f}m"
        )

        self.move_duration = turn_seconds
        self.move_start_time = time.time()
        self.state = "ALIGNING"


    # ============================================================
    # DRIVE STRAIGHT (NO VISION)
    # ============================================================
    def start_blind_move(self):
        drive_seconds = self.target_drive_dist / self.DRIVE_SPEED
        drive_seconds = max(drive_seconds + 0.4, 0.6)

        self.get_logger().info(
            f"DRIVE → {self.target_drive_dist:.2f}m "
            f"({drive_seconds:.2f}s)"
        )

        self.move_duration = drive_seconds
        self.move_start_time = time.time()
        self.state = "MOVING_BLIND"


    # ============================================================
    # FINISH (ROBUST SIGNAL)
    # ============================================================
    def finish_task(self):
        self.get_logger().info("Target reached — stopping.")

        stop = Twist()
        for _ in range(20):
            self.pub_vel.publish(stop)
            time.sleep(0.05)

        if not self.tag_reached_sent:
            msg = Bool()
            msg.data = True

            self.get_logger().info("Broadcasting /vision/tag_reached...")

            # 🔥 SPAM FOR ROS2 RELIABILITY
            for _ in range(20):
                self.pub_tag_reached.publish(msg)
                time.sleep(0.05)

            self.tag_reached_sent = True
            self.get_logger().info("Published /vision/tag_reached")

        self.state = "DONE"
        self.get_logger().info("RETURN COMPLETE.")


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ReturnNavigator())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
