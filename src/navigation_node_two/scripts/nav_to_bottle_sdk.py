#!/usr/bin/env python3
import math
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

# ---- navigation parameters ----
STOP_DISTANCE = 0.25    # meters: desired final distance from bottle
WALK_SPEED    = 0.3     # m/s
YAW_SPEED     = 0.5     # rad/s
YAW_DEADBAND  = 5.0 * math.pi / 180.0  # 5 degrees


def _send_move_for_duration(sport_client: SportClient,
                            vx: float,
                            vy: float,
                            yaw_rate: float,
                            duration: float) -> None:
    """Open-loop Move command for given duration."""
    start = time.time()
    period = 0.05  # 20 Hz
    while time.time() - start < duration:
        # WARNING: open-loop; make sure the area around the robot is clear.
        sport_client.Move(vx, vy, yaw_rate)
        time.sleep(period)


def navigate_to_target(sport_client: SportClient, X: float, Z: float) -> None:
    """
    One-shot navigate:
      1) rotate in place to face bottle.
      2) walk forward until STOP_DISTANCE in front.
      3) ALWAYS StopMove, then Damp() at the end (as long as Z > 0).

    X: left/right (m), Z: forward (m), camera frame.
       (left is NEGATIVE X, right is POSITIVE X)
    """
    if Z <= 0.0:
        print("[nav] Z <= 0, aborting.")
        return

    # 1) rotate
    yaw_error = math.atan2(X, Z)
    print(f"[nav] yaw_error = {yaw_error:.3f} rad ({math.degrees(yaw_error):.1f} deg)")

    if abs(yaw_error) > YAW_DEADBAND:
        # Negative yaw_error (bottle to LEFT) previously made the dog turn RIGHT,
        # so invert the command sign so it turns toward the bottle.
        yaw_rate_cmd = -YAW_SPEED if yaw_error > 0 else YAW_SPEED
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
        # Already very close – skip forward motion but STILL proceed to Damp().
        print(
            f"[nav] Already closer than stop distance ({STOP_DISTANCE:.2f} m); "
            "skipping forward motion."
        )
    else:
        forward_time = forward_distance / WALK_SPEED
        print(
            f"[nav] Walking forward {forward_distance:.2f} m "
            f"at {WALK_SPEED:.2f} m/s for {forward_time:.2f}s"
        )
        _send_move_for_duration(sport_client, WALK_SPEED, 0.0, 0.0, forward_time)
        print("[nav] Forward motion done.")

    # 2.5) make absolutely sure motion is stopped before Damp
    print("[nav] Stopping any residual motion before Damp...")
    sport_client.StopMove()
    time.sleep(0.5)  # let the robot fully settle

    # 3) Damp in front of the bottle – ALWAYS reached if Z > 0
    try:
        print("[nav] Commanding Damp...")
        sport_client.Damp()
        print("[nav] Damp command sent.")
    except Exception as e:
        print(f"[nav] Damp command failed or not available: {e}")


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

    print("\n[sdk] Ready for manual targets.")
    print("     Enter X Z (meters) on one line, e.g.:  0.1  1.5")
    print("     Type 'q' or 'quit' to exit.\n")

    try:
        while True:
            line = input("Enter X Z (or 'q' to quit): ").strip()
            if not line:
                continue
            if line.lower() in ("q", "quit", "exit"):
                print("[sdk] Quitting.")
                break

            try:
                x_str, z_str = line.split()
                X = float(x_str)
                Z = float(z_str)
            except ValueError:
                print("[sdk] Could not parse input. Expected: X Z")
                continue

            print(f"[sdk] Received target X={X:.2f}, Z={Z:.2f}")
            navigate_to_target(sport_client, X, Z)

    finally:
        sport_client.StopMove()
        print("[sdk] Exiting nav_to_bottle_sdk.")


if __name__ == "__main__":
    main()


