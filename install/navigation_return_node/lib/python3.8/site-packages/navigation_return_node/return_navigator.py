#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, PointStamped


class ReturnNavigator(Node):
    """
    Skeleton node for 'return home' behavior.

    Plumbing is set up:
    - subscribes to arm_done and /vision/home_xyz
    - publishes /cmd_vel and /cmd_laydown

    Actual navigation logic can be filled in or imported later.
    """

    def __init__(self):
        super().__init__('return_navigator')

        # Parameters so you can easily tweak topics later
        self.arm_done_topic = self.declare_parameter(
            'arm_done_topic', '/arm/pickup_done'
        ).get_parameter_value().string_value

        self.home_xyz_topic = self.declare_parameter(
            'home_xyz_topic', '/vision/home_xyz'
        ).get_parameter_value().string_value

        self.cmd_vel_topic = self.declare_parameter(
            'cmd_vel_topic', '/cmd_vel'
        ).get_parameter_value().string_value

        self.cmd_laydown_topic = self.declare_parameter(
            'cmd_laydown_topic', '/cmd_laydown'
        ).get_parameter_value().string_value

        # Publishers
        self.pub_vel = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.pub_laydown = self.create_publisher(Bool, self.cmd_laydown_topic, 10)

        # Subscribers
        self.sub_arm_done = self.create_subscription(
            Bool, self.arm_done_topic, self.arm_done_callback, 10
        )

        self.sub_home_xyz = self.create_subscription(
            PointStamped, self.home_xyz_topic, self.home_xyz_callback, 10
        )

        # Simple state machine scaffold
        # WAIT_FOR_ARM -> SEARCHING_HOME -> APPROACHING_HOME -> DONE
        self.state = 'WAIT_FOR_ARM'
        self.last_home_point = None

        # Control loop timer (10 Hz), where your teammate’s logic can live
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            'ReturnNavigator initialized. '
            'Waiting for arm_done signal and home_xyz messages...'
        )

    # ---------- Callbacks ----------

    def arm_done_callback(self, msg: Bool):
        if not msg.data:
            return

        if self.state == 'WAIT_FOR_ARM':
            self.get_logger().info('Arm reports pickup complete. Transitioning to SEARCHING_HOME.')
            self.state = 'SEARCHING_HOME'
            # TODO: stand up + start spin logic can go here later

    def home_xyz_callback(self, msg: PointStamped):
        """
        Called whenever ArucoNode publishes home position.
        msg.point.x, msg.point.y, msg.point.z are in camera frame.
        """
        self.last_home_point = msg.point

        # Later you can use this in SEARCHING_HOME / APPROACHING_HOME
        # to compute how to move the robot.
        if self.state in ['SEARCHING_HOME', 'APPROACHING_HOME']:
            # For now just log occasionally
            self.get_logger().debug(
                f'Received home_xyz: ({msg.point.x:.3f}, '
                f'{msg.point.y:.3f}, {msg.point.z:.3f})'
            )

    # ---------- Main control loop ----------

    def control_loop(self):
        """
        Runs at fixed rate. Actual motion logic can be slotted in here.
        Right now it just logs state; no movement commands.
        """
        if self.state == 'WAIT_FOR_ARM':
            # Do nothing, just wait for arm_done
            return

        elif self.state == 'SEARCHING_HOME':
            # TODO: spin in place, use last_home_point when available to switch
            # to APPROACHING_HOME
            # Example placeholder:
            # cmd = Twist()
            # cmd.angular.z = 0.3
            # self.pub_vel.publish(cmd)
            return

        elif self.state == 'APPROACHING_HOME':
            # TODO: use last_home_point to drive robot in front of the tag
            # using open-loop or closed-loop logic
            return

        elif self.state == 'DONE':
            # Ensure we stay stopped
            stop_cmd = Twist()
            self.pub_vel.publish(stop_cmd)
            return

    # ---------- Helpers you/teammate can fill in ----------

    def send_laydown(self):
        """Call this once you’re done navigating home."""
        msg = Bool()
        msg.data = True
        self.pub_laydown.publish(msg)
        self.get_logger().info('Published laydown command.')


def main(args=None):
    rclpy.init(args=args)
    node = ReturnNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Make sure robot is stopped on shutdown
        stop_cmd = Twist()
        node.pub_vel.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
