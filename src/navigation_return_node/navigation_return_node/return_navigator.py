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

        # --- TUNING (similar to bottle navigator) ---
        self.TARGET_DIST = 0.25      # how close to stop in front of the tag (m)
        self.ALIGN_SPEED = 0.3       # rad/s
        self.DRIVE_SPEED = 0.3       # m/s
        self.SAMPLES_NEEDED = 3

        # Spin speed while searching for the Aruco
        self.SEARCH_SPIN_SPEED = 0.3  # rad/s

        # State Variables
        # SEARCHING -> SAMPLING -> ALIGNING -> MOVING_BLIND -> DONE
        self.state = "SEARCHING"
        self.samples = []   # list of (depth, angle)

        # Move Timers
        self.move_start_time = 0.0
        self.move_duration = 0.0
        self.turn_direction = 0
        self.target_drive_dist = 0.0

        # Publishers
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_stand_down = self.create_publisher(Bool, '/cmd_stand_down', 10)
        # Optional: stand up topic (if your base driver supports it)
        self.pub_stand_up = self.create_publisher(Bool, '/cmd_stand_up', 10)

        # Vision Subscriber: Aruco XYZ from your aruco_node
        self.sub_home = self.create_subscription(
            PointStamped,
            '/vision/home_xyz',
            self.vision_cb,
            qos_profile_sensor_data
        )

        # Control timer
        self.timer = self.create_timer(0.05, self.control_loop)

        # Make the dog stand up at startup (if topic is wired)
        self.send_stand_up()

        self.get_logger().info("ReturnNavigator Ready (Aruco return mode).")

    # -------------------------
    # Stand up helper
    # -------------------------
    def send_stand_up(self):
        msg = Bool()
        msg.data = True
        # spam a few times in case of missed messages
        for _ in range(10):
            self.pub_stand_up.publish(msg)
            time.sleep(0.05)
        self.get_logger().info("Sent stand-up command.")

    # -------------------------
    # Vision callback
    # -------------------------
    def vision_cb(self, msg: PointStamped):
        # Once we're aligning, moving, or done, ignore new detections
        if self.state in ("ALIGNING", "MOVING_BLIND", "DONE"):
            return

        depth = msg.point.z       # forward distance (m)
        x = msg.point.x           # left/right (m)

        if depth <= 0.0:
            # Bad depth reading, ignore
            return

        # Same convention as bottle code:
        # angle is left/right offset around vertical axis
        angle = math.atan2(-x, depth)

        if self.state == "SEARCHING":
            self.get_logger().info("Aruco tag spotted! Switching to SAMPLING...")
            self.state = "SAMPLING"
            self.samples = []

        if self.state == "SAMPLING":
            if len(self.samples) < self.SAMPLES_NEEDED:
                self.samples.append((depth, angle))
                self.get_logger().info(
                    f"Sample {len(self.samples)}/{self.SAMPLES_NEEDED}: "
                    f"depth={depth:.2f} m, angle={angle:.2f} rad "
                    f"({math.degrees(angle):.1f} deg)"
                )

    # -------------------------
    # Main control loop
    # -------------------------
    def control_loop(self):
        if self.state == "DONE":
            return

        cmd = Twist()
        now = time.time()

        if self.state == "SEARCHING":
            # Spin in place until we see the Aruco
            cmd.linear.x = 0.0
            cmd.angular.z = self.SEARCH_SPIN_SPEED
            self.pub_vel.publish(cmd)

        elif self.state == "SAMPLING":
            # Stop while we collect a few stable samples
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.pub_vel.publish(cmd)

            if len(self.samples) >= self.SAMPLES_NEEDED:
                self.plan_approach()

        elif self.state == "ALIGNING":
            if (now - self.move_start_time) < self.move_duration:
                cmd.angular.z = self.ALIGN_SPEED * self.turn_direction
                cmd.linear.x = 0.0
                self.pub_vel.publish(cmd)
            else:
                self.get_logger().info("Aligned with Aruco! Starting forward dash...")
                self.start_blind_move()

        elif self.state == "MOVING_BLIND":
            if (now - self.move_start_time) < self.move_duration:
                cmd.linear.x = self.DRIVE_SPEED
                cmd.angular.z = 0.0
                self.pub_vel.publish(cmd)
            else:
                self.get_logger().info("Time's up! Reached Aruco target distance.")
                self.finish_task()

    # -------------------------
    # Plan turn + drive based on samples
    # -------------------------
    def plan_approach(self):
        avg_depth = sum(s[0] for s in self.samples) / len(self.samples)
        avg_angle = sum(s[1] for s in self.samples) / len(self.samples)

        turn_seconds = abs(avg_angle) / self.ALIGN_SPEED
        self.turn_direction = 1 if avg_angle > 0 else -1

        self.get_logger().info(
            f"Plan: Turn {avg_angle:.2f} rad ({math.degrees(avg_angle):.1f} deg) "
            f"for {turn_seconds:.2f} s"
        )

        self.target_drive_dist = avg_depth - self.TARGET_DIST
        if self.target_drive_dist < 0:
            self.target_drive_dist = 0.0

        self.move_duration = turn_seconds
        self.move_start_time = time.time()
        self.state = "ALIGNING"

    # -------------------------
    # Start open-loop forward move
    # -------------------------
    def start_blind_move(self):
        drive_seconds = self.target_drive_dist / self.DRIVE_SPEED if self.DRIVE_SPEED > 0 else 0.0
        self.get_logger().info(
            f"Dash: {self.target_drive_dist:.2f} m forward "
            f"({drive_seconds:.2f} s at {self.DRIVE_SPEED:.2f} m/s)"
        )

        self.move_duration = drive_seconds
        self.move_start_time = time.time()
        self.state = "MOVING_BLIND"

    # -------------------------
    # Finish: brake + stand down
    # -------------------------
    def finish_task(self):
        self.get_logger().info("Aruco Target Reached! Braking...")

        # 1. SPAM STOP (same pattern as bottle navigator)
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0

        for _ in range(20):
            self.pub_vel.publish(stop_cmd)
            time.sleep(0.05)

        self.get_logger().info("Stopped. Sending Stand Down Command...")

        # 2. SPAM STAND DOWN
        stand_down_msg = Bool()
        stand_down_msg.data = True

        for _ in range(10):
            self.pub_stand_down.publish(stand_down_msg)
            time.sleep(0.1)

        self.state = "DONE"
        self.get_logger().info("STANDING DOWN. Return Task Complete.")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ReturnNavigator())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
