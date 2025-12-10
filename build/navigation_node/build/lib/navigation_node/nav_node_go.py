# nav_node_go.py
"""
Navigation node for Unitree Go2-W.
- Subscribes to bottle pose from YOLO + RealSense vision node
- Publishes PoseStamped /bottle_pose for compatibility
- Robot navigates to bottle using odometry
- Stops at approach point and lays down (optional)
"""

import math
from typing import Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import tf2_ros
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
import transforms3d.euler as euler


# ---------------------------
# Utility functions
# ---------------------------
def yaw_from_quat(qw, qx, qy, qz):
    roll, pitch, yaw = euler.quat2euler([qw, qx, qy, qz])
    return yaw

def clamp(x, lo, hi):
    return max(lo, min(hi, x))


# ---------------------------
# Bezier-like path generator
# ---------------------------
def cubic_bezier(p0: np.ndarray, p1: np.ndarray, p2: np.ndarray, p3: np.ndarray, t: float):
    return (1 - t)**3 * p0 + 3*(1 - t)**2 * t * p1 + 3*(1 - t) * t**2 * p2 + t**3 * p3

def generate_smooth_waypoints(x1, y1, yaw1, x2, y2, yaw2, offset=0.2, num_points=12):
    d1 = np.array([math.cos(yaw1), math.sin(yaw1)])
    d2 = np.array([-math.cos(yaw2), -math.sin(yaw2)])
    c1 = np.array([x1, y1]) + offset * d1
    c2 = np.array([x2, y2]) + offset * d2

    t_vals = np.linspace(0, 1, num_points)
    pts = [cubic_bezier(np.array([x1, y1]), c1, c2, np.array([x2, y2]), t) for t in t_vals]

    thetas = []
    for i in range(len(pts)-1):
        dx = pts[i+1][0] - pts[i][0]
        dy = pts[i+1][1] - pts[i][1]
        thetas.append(math.atan2(dy, dx))
    thetas.append(thetas[-1])

    return [(float(pts[i][0]), float(pts[i][1]), float(thetas[i])) for i in range(len(pts))]


# ---------------------------
# Navigation Node
# ---------------------------
class NavigationNode(Node):
    def __init__(self):
        super().__init__('go2_navigation_node')

        # Publishers / Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 20)
        self.goal_reached_pub = self.create_publisher(Bool, '/nav/goal_reached', 10)

        # Bottle pose subscription from vision node
        self.create_subscription(PoseStamped, '/vision/target_pose', self.bottle_cb, 10)

        # Re-publish for compatibility
        self.bottle_pub = self.create_publisher(PoseStamped, '/bottle_pose', 10)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State
        self.current_odom = None
        self.home_pose = None
        self.bottle_pose_raw = None
        self.goal_waypoints = []
        self.current_waypoint_idx = 0
        self.state = 'IDLE'  # IDLE, PLANNING, EXECUTING, ARRIVED

        # Navigation params
        self.approach_offset = 0.35
        self.waypoints_num = 12
        self.max_lin = 0.6
        self.max_lat = 0.2
        self.max_ang = 0.8
        self.cmd_rate = 0.05
        self.prev_cmd = Twist()

        # Timer
        self.timer = self.create_timer(self.cmd_rate, self.control_loop)

    # -------------------------
    # Odometry callback
    # -------------------------
    def odom_cb(self, msg: Odometry):
        self.current_odom = msg
        if self.home_pose is None:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose = msg.pose.pose
            self.home_pose = ps

    # -------------------------
    # Bottle pose callback
    # -------------------------
    def bottle_cb(self, msg: PoseStamped):
        self.bottle_pose_raw = msg
        self.bottle_pub.publish(msg)  # republish
        if self.state == 'IDLE':
            self.state = 'PLANNING'

    # -------------------------
    # Planning helpers
    # -------------------------
    def transform_pose_to_frame(self, ps: PoseStamped, target_frame: str) -> PoseStamped:
        if ps is None:
            raise ValueError("No pose provided for transform")
        for _ in range(20):
            try:
                if ps.header.frame_id == '':
                    raise ValueError("PoseStamped has empty frame_id")
                trans = self.tf_buffer.lookup_transform(target_frame, ps.header.frame_id, rclpy.time.Time())
                transformed = do_transform_pose(ps, trans)
                transformed.header.frame_id = target_frame
                return transformed
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rclpy.spin_once(self, timeout_sec=0.02)
        raise RuntimeError(f"Could not transform pose from {ps.header.frame_id} to {target_frame}")

    def compute_approach_point(self, bottle_in_odom: PoseStamped, offset: float):
        bx = bottle_in_odom.pose.position.x
        by = bottle_in_odom.pose.position.y
        if self.current_odom is None:
            raise RuntimeError("No odom yet to compute approach point")
        rx = self.current_odom.pose.pose.position.x
        ry = self.current_odom.pose.pose.position.y

        vec_x = bx - rx
        vec_y = by - ry
        dist = math.hypot(vec_x, vec_y)
        if dist < 1e-6:
            approach_x = bx - offset
            approach_y = by
        else:
            ux = vec_x / dist
            uy = vec_y / dist
            approach_x = bx - ux * offset
            approach_y = by - uy * offset

        approach_yaw = math.atan2(by - approach_y, bx - approach_x)
        return approach_x, approach_y, approach_yaw

    def plan_to_bottle(self):
        if self.bottle_pose_raw is None or self.current_odom is None:
            return False
        try:
            bottle_odom = self.transform_pose_to_frame(self.bottle_pose_raw, 'odom')
        except Exception:
            return False

        ax, ay, ayaw = self.compute_approach_point(bottle_odom, self.approach_offset)

        start_x = self.current_odom.pose.pose.position.x
        start_y = self.current_odom.pose.pose.position.y
        q = self.current_odom.pose.pose.orientation
        start_yaw = yaw_from_quat(q.w, q.x, q.y, q.z)

        wp = generate_smooth_waypoints(start_x, start_y, start_yaw, ax, ay, ayaw,
                                       offset=0.3, num_points=self.waypoints_num)
        self.goal_waypoints = wp
        self.current_waypoint_idx = 0
        return True

    # -------------------------
    # Controller
    # -------------------------
    def compute_cmd_to_waypoint(self, waypoint):
        cx = self.current_odom.pose.pose.position.x
        cy = self.current_odom.pose.pose.position.y
        q = self.current_odom.pose.pose.orientation
        cyaw = yaw_from_quat(q.w, q.x, q.y, q.z)

        wx, wy, wyaw = waypoint
        dx = wx - cx
        dy = wy - cy

        rel_x = math.cos(-cyaw) * dx - math.sin(-cyaw) * dy
        rel_y = math.sin(-cyaw) * dx + math.cos(-cyaw) * dy

        dist = math.hypot(rel_x, rel_y)
        yaw_err = (wyaw - cyaw + math.pi) % (2*math.pi) - math.pi

        vx = clamp(0.8 * rel_x, -self.max_lin, self.max_lin)
        vy = clamp(0.8 * rel_y, -self.max_lat, self.max_lat)
        wz = clamp(1.2 * yaw_err, -self.max_ang, self.max_ang)

        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.angular.z = float(wz)
        return cmd, dist, abs(yaw_err)

    def smooth_and_publish(self, cmd: Twist):
        alpha = 0.5
        sm_cmd = Twist()
        sm_cmd.linear.x = (1-alpha) * self.prev_cmd.linear.x + alpha * clamp(cmd.linear.x, -self.max_lin, self.max_lin)
        sm_cmd.linear.y = (1-alpha) * self.prev_cmd.linear.y + alpha * clamp(cmd.linear.y, -self.max_lat, self.max_lat)
        sm_cmd.angular.z = (1-alpha) * self.prev_cmd.angular.z + alpha * clamp(cmd.angular.z, -self.max_ang, self.max_ang)
        self.cmd_pub.publish(sm_cmd)
        self.prev_cmd = sm_cmd

    # -------------------------
    # Control loop
    # -------------------------
    def control_loop(self):
        try:
            if self.state == 'PLANNING':
                ok = self.plan_to_bottle()
                if ok and len(self.goal_waypoints) > 0:
                    self.state = 'EXECUTING'
                return

            elif self.state == 'EXECUTING':
                if self.current_waypoint_idx >= len(self.goal_waypoints):
                    self.cmd_pub.publish(Twist())
                    self.get_logger().info("Approach point reached. Stopping.")
                    self.goal_reached_pub.publish(Bool(data=True))
                    self.state = 'ARRIVED'
                    return

                waypoint = self.goal_waypoints[self.current_waypoint_idx]
                cmd, dist, yaw_err = self.compute_cmd_to_waypoint(waypoint)
                if dist < 0.12:
                    self.current_waypoint_idx += 1
                    self.cmd_pub.publish(Twist())
                    return
                self.smooth_and_publish(cmd)

            elif self.state == 'ARRIVED':
                self.cmd_pub.publish(Twist())
                self.get_logger().info("arrived")

            else:
                self.cmd_pub.publish(Twist())

        except Exception as e:
            self.get_logger().error(f"Control loop exception: {e}")
            self.cmd_pub.publish(Twist())
            self.state = 'IDLE'


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
