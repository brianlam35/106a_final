#!/usr/bin/env python3
import os
os.environ["PYTORCH_NO_CUDA_MEMORY_CACHING"] = "1"
os.environ["CUDA_VISIBLE_DEVICES"] = ""  # force CPU to avoid TLS memory issues

import math
import time

import cv2
import numpy as np

import torch
torch.set_num_threads(1)

from ultralytics import YOLO

import pyrealsense2 as rs

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

# ==========================
# Navigation / control params
# ==========================
STOP_DISTANCE      = 0.25   # m, desired distance in front of bottle
YAW_DEADBAND_DEG   = 5.0    # degrees tolerance for yaw
Z_DEADBAND         = 0.05   # m tolerance for distance

WALK_SPEED_MAX     = 0.3    # m/s (max forward speed)
YAW_SPEED_MAX      = 0.5    # rad/s (max yaw rate)

K_YAW              = 1.0    # yaw_rate = -K_YAW * yaw_error, clamped
K_VX               = 0.8    # vx = K_VX * (Z - STOP_DISTANCE), clamped

DETECTION_TIMEOUT  = 0.5    # s, how long we tolerate no detection before stopping
DWELL_TIME         = 0.5    # s, must stay in good zone this long before Damp

CONTROL_PERIOD     = 0.05   # s, ~20 Hz control loop


def pixel_to_3d(u, v, depth_m, intr):
    """
    Convert pixel (u, v) + depth (m) to camera-frame coordinates (X, Y, Z).
    intr: rs.intrinsics
    """
    fx, fy = intr.fx, intr.fy
    cx, cy = intr.ppx, intr.ppy

    X = (u - cx) * depth_m / fx
    Y = (v - cy) * depth_m / fy
    Z = depth_m
    return X, Y, Z


def find_bottle_xyz(color_img, depth_img, intr, model):
    """
    Run YOLO on color_img, find largest sports ball (class 39),
    return (X, Y, Z) in camera frame, or None if no valid detection.
    """
    try:
        results = model.predict(
            source=color_img,
            conf=0.5,
            device="cpu",
            verbose=False
        )
    except Exception as e:
        print(f"[vision] YOLO prediction failed: {e}")
        return None

    if len(results) == 0 or results[0].boxes is None:
        return None

    boxes = results[0].boxes
    best_det = None
    best_area = 0.0

    # Choose the biggest sports ball in view
    for i, cls in enumerate(boxes.cls):
        if int(cls) == 39:  # COCO class index 39 = sports ball
            xyxy = boxes.xyxy[i].cpu().numpy()
            x1, y1, x2, y2 = xyxy
            area = (x2 - x1) * (y2 - y1)
            if area > best_area:
                best_area = area
                best_det = xyxy

    if best_det is None:
        return None

    x1, y1, x2, y2 = best_det
    cx = int((x1 + x2) / 2)
    cy = int((y1 + y2) / 2)

    h, w = depth_img.shape
    if cx < 0 or cx >= w or cy < 0 or cy >= h:
        print(f"[vision] Skipping bottle: ({cx},{cy}) out of depth bounds ({w},{h})")
        return None

    depth_raw = float(depth_img[cy, cx])
    if depth_raw <= 0 or np.isnan(depth_raw):
        print(f"[vision] Skipping bottle: invalid depth at ({cx},{cy}), value={depth_raw}")
        return None

    # depth_raw is in mm from RealSense
    depth_m = depth_raw * 0.001
    X, Y, Z = pixel_to_3d(cx, cy, depth_m, intr)
    print(f"[vision] Bottle at ({X:.2f}, {Y:.2f}, {Z:.2f}) m (camera frame)")
    return X, Y, Z


def init_realsense():
    """
    Initialize RealSense pipeline with color + aligned depth.
    Returns (pipeline, align, intrinsics).
    """
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    profile = pipeline.start(config)

    # Align depth to color
    align_to = rs.stream.color
    align = rs.align(align_to)

    color_profile = profile.get_stream(rs.stream.color)
    color_intr = color_profile.as_video_stream_profile().get_intrinsics()

    print("[rs] RealSense pipeline started.")
    print(f"[rs] Color intrinsics: fx={color_intr.fx:.1f}, fy={color_intr.fy:.1f}, "
          f"cx={color_intr.ppx:.1f}, cy={color_intr.ppy:.1f}")
    return pipeline, align, color_intr


def main():
    # 1) Load YOLO model
    print("[vision] Loading YOLOv8n on CPU...")
    model = YOLO("yolov8n.pt")
    model.to("cpu")
    print("[vision] YOLOv8n loaded.")

    # 2) Initialize RealSense
    pipeline, align, intr = init_realsense()

    # 3) Initialize Unitree SDK / SportClient
    iface = "eth0"
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

    # 4) Control state
    last_X = None
    last_Z = None
    last_detection_time = None
    target_reached = False
    dwell_start = None

    yaw_deadband = math.radians(YAW_DEADBAND_DEG)

    try:
        print("[main] Entering vision + navigation loop...")
        while not target_reached:
            # Get aligned frames
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            color_img = np.asanyarray(color_frame.get_data())
            depth_img = np.asanyarray(depth_frame.get_data())

            # --- Vision: find bottle ---
            det = find_bottle_xyz(color_img, depth_img, intr, model)
            now = time.time()

            if det is not None:
                X, Y, Z = det
                last_X, last_Z = X, Z
                last_detection_time = now
            else:
                # No detection this frame
                if last_detection_time is None or (now - last_detection_time) > DETECTION_TIMEOUT:
                    # Lost target completely -> stop
                    print("[nav] No valid detection; stopping.")
                    sport_client.StopMove()
                    dwell_start = None
                    continue
                # else: we can still reuse last_X, last_Z for a short while

            if last_X is None or last_Z is None:
                # Still nothing to work with yet
                sport_client.StopMove()
                continue

            X = last_X
            Z = last_Z

            if Z <= 0.0:
                print("[nav] Z <= 0 from detection; stopping.")
                sport_client.StopMove()
                dwell_start = None
                continue

            # --- Navigation control ---
            yaw_error = math.atan2(X, Z)           # rad
            z_error = Z - STOP_DISTANCE            # m

            print(f"[nav] yaw_error={math.degrees(yaw_error):.1f} deg, "
                  f"z_error={z_error:.2f} m (Z={Z:.2f})")

            # Check if we are "good enough"
            if abs(yaw_error) < yaw_deadband and abs(z_error) < Z_DEADBAND:
                if dwell_start is None:
                    dwell_start = now
                    print("[nav] In tolerance zone; starting dwell timer...")
                elif (now - dwell_start) >= DWELL_TIME:
                    # Reached target & held it long enough
                    print("[nav] Target reached; StopMove + Damp...")
                    sport_client.StopMove()
                    time.sleep(0.5)
                    try:
                        sport_client.Damp()
                        print("[nav] Damp() command sent; navigation complete.")
                    except Exception as e:
                        print(f"[nav] Damp() failed: {e}")
                    target_reached = True
                    break
                else:
                    # Still dwelling; stay still
                    sport_client.StopMove()
                    time.sleep(CONTROL_PERIOD)
                    continue
            else:
                # Not in tolerance; reset dwell
                dwell_start = None

            # Compute yaw command (sign tuned as before)
            yaw_rate = -K_YAW * yaw_error
            yaw_rate = max(min(yaw_rate, YAW_SPEED_MAX), -YAW_SPEED_MAX)

            # Compute forward speed (only move forward)
            vx = K_VX * z_error
            if vx < 0.0:
                vx = 0.0
            vx = min(vx, WALK_SPEED_MAX)

            # If we don't need to move forward but yaw is small, just stop
            if vx == 0.0 and abs(yaw_error) < yaw_deadband:
                sport_client.StopMove()
            else:
                sport_client.Move(vx, 0.0, yaw_rate)

            time.sleep(CONTROL_PERIOD)

    except KeyboardInterrupt:
        print("[main] KeyboardInterrupt; stopping.")
    finally:
        print("[main] Stopping motion and shutting down.")
        try:
            sport_client.StopMove()
        except Exception:
            pass
        try:
            pipeline.stop()
        except Exception:
            pass
        print("[main] Exit vision_nav_combine.")


if __name__ == "__main__":
    main()
