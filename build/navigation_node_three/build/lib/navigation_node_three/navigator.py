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
        self.TARGET_DIST = 0.15 
        self.ALIGN_SPEED = 0.25 # TWEAK: Reduced speed for finer control
        self.DRIVE_SPEED = 0.3 
        self.SAMPLES_NEEDED = 3
        
        # State Variables
        self.state = "SEARCHING" 
        self.samples = []        
        
        # Move Timers
        self.move_start_time = 0.0
        self.move_duration = 0.0
        self.turn_direction = 0 
        self.target_drive_dist = 0.0

        # Publishers
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # --- ROBUST PUBLISHERS (DAMP/Collapse) ---
        # Reverting to /cmd_laydown for the DAMP command
        self.pub_laydown = self.create_publisher(Bool, '/cmd_laydown', 10) 
        
        # Vision Subscriber
        self.sub_vision = self.create_subscription(
            PoseStamped, 
            '/vision/target_pose', 
            self.vision_cb, 
            qos_profile_sensor_data 
        )
        
        self.timer = self.create_timer(0.05, self.control_loop) 
        self.get_logger().info("Navigator Ready (DAMP Mode).") # Corrected log

    def vision_cb(self, msg):
        if self.state == "DONE" or self.state == "ALIGNING" or self.state == "MOVING_BLIND": return
        
        depth = msg.pose.position.z
        angle = math.atan2(-msg.pose.position.x, depth) 
        
        if self.state == "SEARCHING":
            self.get_logger().info("Target spotted! Stopping to sample...")
            self.state = "SAMPLING"
            self.samples = [] 
            
        elif self.state == "SAMPLING":
            if len(self.samples) < self.SAMPLES_NEEDED:
                self.samples.append((depth, angle))

    def control_loop(self):
        if self.state == "DONE": return

        cmd = Twist()
        now = time.time()
        
        if self.state == "SEARCHING":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.pub_vel.publish(cmd)

        elif self.state == "SAMPLING":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.pub_vel.publish(cmd)
            if len(self.samples) >= self.SAMPLES_NEEDED:
                self.plan_approach()

        elif self.state == "ALIGNING":
            if (now - self.move_start_time) < self.move_duration:
                cmd.angular.z = self.ALIGN_SPEED * self.turn_direction
                self.pub_vel.publish(cmd)
            else:
                self.get_logger().info("Aligned! Starting Dash...")
                self.start_blind_move()

        elif self.state == "MOVING_BLIND":
            if (now - self.move_start_time) < self.move_duration:
                cmd.linear.x = self.DRIVE_SPEED
                self.pub_vel.publish(cmd)
            else:
                self.get_logger().info("Time's up! Reached Target.")
                self.finish_task()

    def plan_approach(self):
        avg_depth = sum([s[0] for s in self.samples]) / len(self.samples)
        avg_angle = sum([s[1] for s in self.samples]) / len(self.samples)
        
        turn_seconds = abs(avg_angle) / self.ALIGN_SPEED
        
        # --- FIX ALIGNMENT: Use 35% time buffer to fix "slightly right" error ---
        turn_seconds *= 1.35 
        
        self.turn_direction = 1 if avg_angle > 0 else -1 
        
        self.target_drive_dist = avg_depth - self.TARGET_DIST
        if self.target_drive_dist < 0: self.target_drive_dist = 0.0

        self.get_logger().info(f"Plan: Turn {avg_angle:.2f} rad ({turn_seconds:.2f}s), Drive {self.target_drive_dist:.2f} m")

        # Set minimum duration to ensure the robot starts moving for small angles
        self.move_duration = max(turn_seconds, 0.4) 
        self.move_start_time = time.time()
        self.state = "ALIGNING"

    def start_blind_move(self):
        drive_seconds = self.target_drive_dist / self.DRIVE_SPEED
        
        # Add buffer for the linear dash
        self.move_duration = max(drive_seconds + 0.5, 0.5) 
        
        self.get_logger().info(f"Dash: {self.target_drive_dist:.2f}m ({self.move_duration:.2f}s total)")
        self.move_start_time = time.time()
        self.state = "MOVING_BLIND"

    def finish_task(self):
        self.get_logger().info("Target Reached! Executing Full Stop and DAMP (Lay Down)...")
        
        # 1. SPAM VELOCITY ZERO (Brake)
        zero_vel = Twist()
        zero_vel.linear.x = 0.0
        zero_vel.angular.z = 0.0
        
        # Spamming zero velocity for 1 second (20 * 0.05s) to ensure a hard stop
        for _ in range(20): 
            self.pub_vel.publish(zero_vel)      
            time.sleep(0.05)
            
        self.get_logger().info("Stopped. Sending DAMP/Lay Down Command...")

        # 2. SPAM DAMP (Lay Down)
        laydown_msg = Bool()
        laydown_msg.data = True
        
        # Spamming the laydown command for 1 second
        for _ in range(10): 
            self.pub_laydown.publish(laydown_msg) 
            time.sleep(0.1)
        
        self.state = "DONE"
        self.get_logger().info("DAMPING COMPLETE. Task Complete.")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Navigator())
    rclpy.shutdown()