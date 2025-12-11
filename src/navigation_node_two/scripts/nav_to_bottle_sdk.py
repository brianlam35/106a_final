#!/usr/bin/env python3
import math
import time
import socket

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

# ---- navigation parameters ----
STOP_DISTANCE = 0.6     # meters
WALK_SPEED    = 0.3     # m/s
YAW_SPEED     = 0.5     # rad/s
YAW_DEADBAND  = 5.0 * math.pi / 180.0  # 5 degrees

# ---- TCP bridge config ----
TCP_HOST = "127.0.0.1"
TCP_PORT = 5005


def _send_move_for_duration(sport_client: SportClient, vx: float, vy: float, yaw_rate: float, duration: float):
    """Open-loop Move command for given duration."""
    start = time.time()
    period = 0.05  # 20 Hz
    while time.time() - start < duration:
        sport_client.Move(vx, vy, yaw_rate)
        time.sleep(period)


def navigate_to_target(sport_client: SportClient, X: float, Z: float):
    """
    One-shot navigate:
      1) rotate in place to face bottle
      2) walk forward until STOP_DISTANCE in front
    X: left/right (m), Z: forward (m), camera frame.
    """
    if Z <= 0.0:
        print("[nav] Z <= 0, aborting.")
        return

    # 1) rotate
    yaw_error = math.atan2(X, Z)
    print(f"[nav] yaw_error = {yaw_error:.3f} rad ({math.degrees(yaw_error):.1f} deg)")

    if abs(yaw_error) > YAW_DEADBAND:
        yaw_rate_cmd = YAW_SPEED if yaw_error > 0 else -YAW_SPEED
        rotate_time = abs(yaw_error) / YAW_SPEED
        print(f"[nav] Rotating yaw_rate={yaw_rate_cmd:.2f} for {rotate_time:.2f}s")
        _send_move_for_duration(sport_client, 0.0, 0.0, yaw_rate_cmd, rotate_time)
        sport_client.StopMove()
        time.sleep(0.3)
    else:
        print("[nav] Yaw within deadband; skipping rotation.")

    # 2) forward
    forward_distance = Z - STOP_DISTANCE
    if forward_distance <= 0.0:
        print(f"[nav] Already closer than stop distance ({STOP_DISTANCE:.2f} m); no forward motion.")
        return

    forward_time = forward_distance / WALK_SPEED
    print(f"[nav] Walking forward {forward_distance:.2f} m at {WALK_SPEED:.2f} m/s for {forward_time:.2f}s")
    _send_move_for_duration(sport_client, WALK_SPEED, 0.0, 0.0, forward_time)
    sport_client.StopMove()
    print("[nav] Arrived in front of bottle.")


def main():
    iface = "eth0"  # can parameterize later if you want
    print(f"[sdk] Initializing Unitree channel on interface: {iface}")
    ChannelFactoryInitialize(0, iface)

    sport_client = SportClient()
    sport_client.SetTimeout(10.0)
    sport_client.Init()

    print("[sdk] Sending StandUp + BalanceStand ...")
    try:
        sport_client.StandUp()
        time.sleep(1.0)
        sport_client.BalanceStand()
        time.sleep(0.5)
    except Exception as e:
        print(f"[sdk] Stand sequence failed or skipped: {e}")

    # TCP server – waits for ROS bridge
    print(f"[sdk] Starting TCP server on {TCP_HOST}:{TCP_PORT}")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((TCP_HOST, TCP_PORT))
        server.listen(1)
        print("[sdk] Waiting for connection from ROS bridge...")
        conn, addr = server.accept()
        print(f"[sdk] Connected by {addr}")

        with conn, conn.makefile("r") as rf:
            for line in rf:
                line = line.strip()
                if not line:
                    continue
                if line.lower() == "quit":
                    print("[sdk] Received quit, shutting down.")
                    break
                try:
                    x_str, z_str = line.split()
                    X = float(x_str)
                    Z = float(z_str)
                    print(f"[sdk] Received target X={X:.2f}, Z={Z:.2f}")
                    navigate_to_target(sport_client, X, Z)
                except Exception as e:
                    print(f"[sdk] Failed to parse line '{line}': {e}")

    sport_client.StopMove()
    print("[sdk] Exiting nav_to_bottle_sdk.")


if __name__ == "__main__":
    main()
