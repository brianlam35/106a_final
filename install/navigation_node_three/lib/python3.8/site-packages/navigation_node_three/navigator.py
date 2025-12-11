import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool
import math
import time

class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')

        # --- TUNING ---
        self.TARGET_DIST = 0.25 
        self.MAX_SPEED   = 0.3
        self.TURN_SPEED  = 0.5  # rad/s for blind turns
        
        # Sampling Config
        self.SAMPLES_NEEDED = 5
        
        # Search Config
        self.SEARCH_TURN_SPEED = 0.3
        self.SEARCH_TURN_TIME  = 1.0
        self.SEARCH_WAIT_TIME  = 2.0

        # Internal Variables
        self.state = "SEARCHING" # States: SEARCHING, SAMPLING, ALIGNING, MOVING_BLIND, DONE
        self.samples = []        # To store (depth, angle) tuples
        self.blind_cmd = Twist()
        self.blind_end_time = 0.0
        self.last_seen_time = 0

        # Publishers / Subscribers
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_laydown = self.create_publisher(Bool, '/cmd_laydown', 10)
        
        self.sub_vision = self.create_subscription(
            PoseStamped, 
            '/vision/target_pose', 
            self.vision_cb, 
            qos_profile_sensor_data 
        )

        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Search timer helpers
        self.search_state = "WAIT"
        self.search_timer = 0.0

        self.get_logger().info("Navigator Initialized. Waiting for target...")

    def vision_cb(self, msg):
        if self.state == "DONE": return
        
        # Extract data
        depth = msg.pose.position.z
        angle = -msg.pose.position.x 
        
        self.last_seen_time = self.get_clock().now().seconds_nanoseconds()[0]

        # LOGIC:
        # If we are SEARCHING, spotting the bottle triggers SAMPLING
        if self.state == "SEARCHING":
            self.get_logger().info("Target detected! Stopping to sample...")
            self.state = "SAMPLING"
            self.samples = [] # Clear previous data
            
        # If we are SAMPLING, collect data
        elif self.state == "SAMPLING":
            if len(self.samples) < self.SAMPLES_NEEDED:
                self.samples.append((depth, angle))

    def control_loop(self):
        if self.state == "DONE": return

        cmd = Twist()
        now = self.get_clock().now().seconds_nanoseconds()[0]
        
        # --- STATE 1: SEARCHING ---
        if self.state == "SEARCHING":
            # Timeout logic: If we haven't seen bottle recently, ensure we stay searching
            # (Handled implicitly by vision_cb switching us out of this state)
            
            self.search_timer += 0.1
            if self.search_state == "TURN":
                cmd.angular.z = self.SEARCH_TURN_SPEED
                if self.search_timer >= self.SEARCH_TURN_TIME:
                    self.search_state = "WAIT"
                    self.search_timer = 0.0
            elif self.search_state == "WAIT":
                if self.search_timer >= self.SEARCH_WAIT_TIME:
                    self.search_state = "TURN"
                    self.search_timer = 0.0
            
            self.pub_vel.publish(cmd)

        # --- STATE 2: SAMPLING ---
        elif self.state == "SAMPLING":
            # FORCE STOP to ensure accurate samples
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.pub_vel.publish(cmd)

            # Check if we have enough samples
            if len(self.samples) >= self.SAMPLES_NEEDED:
                self.plan_approach()

            # Safety: If vision stops updating in the middle of sampling, reset
            if (now - self.last_seen_time) > 1.0:
                self.get_logger().warn("Lost sight during sampling. Resetting to Search.")
                self.state = "SEARCHING"

        # --- STATE 3: ALIGNING (Blind Turn) ---
        elif self.state == "ALIGNING":
            if now < self.blind_end_time:
                self.pub_vel.publish(self.blind_cmd)
            else:
                self.get_logger().info("Alignment complete. Starting dash...")
                self.start_blind_move()

        # --- STATE 4: MOVING BLIND (Blind Dash) ---
        elif self.state == "MOVING_BLIND":
            if now < self.blind_end_time:
                self.pub_vel.publish(self.blind_cmd)
                # Optional: Log progress
                # self.get_logger().info(f"Dashing... {self.blind_end_time - now:.2f}s remaining")
            else:
                self.get_logger().info("Destination reached (Open Loop). Task Complete.")
                self.finish_task()

    def plan_approach(self):
        # 1. Average the samples
        avg_depth = sum([s[0] for s in self.samples]) / len(self.samples)
        avg_angle = sum([s[1] for s in self.samples]) / len(self.samples)
        
        self.get_logger().info(f"Samples collected. Avg Depth: {avg_depth:.2f}m, Avg Angle: {avg_angle:.2f}rad")

        # 2. Calculate Align Maneuver (Turn to face bottle)
        # Time = Angle / Speed
        turn_duration = abs(avg_angle) / self.TURN_SPEED
        
        # Set the command
        self.blind_cmd = Twist()
        self.blind_cmd.angular.z = self.TURN_SPEED if avg_angle > 0 else -self.TURN_SPEED
        
        # Set the timer
        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.blind_end_time = start_time + turn_duration
        
        # Save distance for next step
        self.target_drive_distance = avg_depth - self.TARGET_DIST
        
        self.state = "ALIGNING"
        self.get_logger().info(f"Aligning for {turn_duration:.2f} seconds...")

    def start_blind_move(self):
        # 1. Calculate Drive Maneuver
        if self.target_drive_distance < 0.05:
            self.get_logger().info("Target is already close enough!")
            self.finish_task()
            return

        drive_duration = self.target_drive_distance / self.MAX_SPEED
        
        self.blind_cmd = Twist()
        self.blind_cmd.linear.x = self.MAX_SPEED
        
        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.blind_end_time = start_time + drive_duration
        
        self.state = "MOVING_BLIND"
        self.get_logger().info(f"Dashing {self.target_drive_distance:.2f}m for {drive_duration:.2f} seconds...")

    def finish_task(self):
        # Stop
        stop_cmd = Twist()
        self.pub_vel.publish(stop_cmd)
        
        # Lay down
        laydown_msg = Bool()
        laydown_msg.data = True
        self.pub_laydown.publish(laydown_msg)
        
        self.state = "DONE"
        self.get_logger().info("LAYING DOWN.")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Navigator())
    rclpy.shutdown()