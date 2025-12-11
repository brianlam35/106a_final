#!/usr/bin/env python3

import socket

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


TCP_HOST = "127.0.0.1"
TCP_PORT = 5005


class PoseBridgeNode(Node):
    def __init__(self):
        super().__init__("pose_bridge_node")

        self.declare_parameter("tcp_host", TCP_HOST)
        self.declare_parameter("tcp_port", TCP_PORT)

        host = self.get_parameter("tcp_host").get_parameter_value().string_value
        port = self.get_parameter("tcp_port").get_parameter_value().integer_value

        # Connect to the SDK process
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.get_logger().info(f"Connecting to SDK bridge at {host}:{port} ...")
        self.sock.connect((host, port))
        self.f = self.sock.makefile("w")

        # Subscribe to the vision pose
        self.sub = self.create_subscription(
            PoseStamped,
            "/vision/target_pose",
            self.pose_callback,
            10,
        )

        self.get_logger().info("PoseBridgeNode ready; forwarding poses to SDK bridge.")

    def pose_callback(self, msg: PoseStamped):
        X = msg.pose.position.x
        Z = msg.pose.position.z
        self.get_logger().info(f"Forwarding pose X={X:.2f}, Z={Z:.2f}")
        try:
            self.f.write(f"{X} {Z}\n")
            self.f.flush()
        except Exception as e:
            self.get_logger().error(f"Failed to send pose to SDK bridge: {e}")

    def destroy_node(self):
        # On shutdown, tell SDK process to quit
        try:
            if hasattr(self, "f"):
                self.f.write("quit\n")
                self.f.flush()
                self.f.close()
            if hasattr(self, "sock"):
                self.sock.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PoseBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
