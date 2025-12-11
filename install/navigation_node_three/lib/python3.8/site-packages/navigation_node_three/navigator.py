import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool

class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')

        # --- TUNING ---
        self.TARGET_DIST = 0.25 
        self.KP_LINEAR   = 0.3  
        self.KP_ANGULAR  = 0.5   
        self.MAX_SPEED   = 0.3  
        self.MIN_SPEED   = 0.15  # <--- NEW: Minimum power to overcome friction
        
        # Search Params
        self.SEARCH_TURN_SPEED = 0.3
        self.SEARCH_TURN_TIME  = 1.0
        self.SEARCH_WAIT_TIME  = 2.0
        
        self.last_seen_time = 0
        self.bottle_detected = False
        self.bottle_depth = 0.0 
        self.bottle_angle = 0.0 
        self.task_complete = False

        self.search_state = "WAIT"
        self.search_timer = 0.0

        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_laydown = self.create_publisher(Bool, '/cmd_laydown', 10)
        
        self.sub_vision = self.create_subscription(
            PoseStamped, 
            '/vision/target_pose', 
            self.vision_cb, 
            qos_profile_sensor_data 
        )

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Navigator Ready. Searching...")

    def vision_cb(self, msg):
        if self.task_complete: return
        self.bottle_detected = True
        self.last_seen_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.bottle_depth = msg.pose.position.z      
        self.bottle_angle = -msg.pose.position.x     

    def control_loop(self):
        if self.task_complete: return

        cmd = Twist()
        now = self.get_clock().now().seconds_nanoseconds()[0]

        if (now - self.last_seen_time) > 0.5:
            self.bottle_detected = False

        if self.bottle_detected:
            self.search_state = "WAIT"
            self.search_timer = 0.0

            dist_error = self.bottle_depth - self.TARGET_DIST
            
            # P-Control
            linear_vel = self.KP_LINEAR * dist_error
            angular_vel = self.KP_ANGULAR * self.bottle_angle

            # --- NEW: DEADZONE KICKSTART ---
            # If we need to move forward, make sure we are fast enough to actually move
            if abs(linear_vel) > 0.01 and abs(linear_vel) < self.MIN_SPEED:
                linear_vel = self.MIN_SPEED * (1 if linear_vel > 0 else -1)

            # Clamp Max Speed
            linear_vel = max(min(linear_vel, self.MAX_SPEED), -self.MAX_SPEED)
            angular_vel = max(min(angular_vel, 1.0), -1.0)
            
            # Stop Condition
            if abs(dist_error) < 0.05:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.pub_vel.publish(cmd)
                
                self.get_logger().info("Target Reached! Sending Lay Down Command...")
                laydown_msg = Bool()
                laydown_msg.data = True
                self.pub_laydown.publish(laydown_msg)
                self.task_complete = True
                return

            cmd.linear.x = float(linear_vel)
            cmd.angular.z = float(angular_vel)
            
            # DEBUG: Print exact command being sent
            self.get_logger().info(f"Chasing... Depth: {self.bottle_depth:.2f}m | Cmd Vel: {linear_vel:.2f}")

        else:
            # Search Pattern
            self.search_timer += 0.1
            if self.search_state == "TURN":
                cmd.linear.x = 0.0
                cmd.angular.z = self.SEARCH_TURN_SPEED
                if self.search_timer >= self.SEARCH_TURN_TIME:
                    self.search_state = "WAIT"
                    self.search_timer = 0.0
            
            elif self.search_state == "WAIT":
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                if self.search_timer >= self.SEARCH_WAIT_TIME:
                    self.search_state = "TURN"
                    self.search_timer = 0.0

        self.pub_vel.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Navigator())
    rclpy.shutdown()
