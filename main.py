"""
main.py
-------
Real-Time Webcam Fire / Flame Detection System using OpenCV and Ultralytics YOLO.
Designed for the fire-fighting robotic car and real-time laptop webcam testing.

Features:
    - GPU acceleration (device=0 on NVIDIA RTX 3050) with auto CPU fallback
    - Bounding box, confidence score, FPS, and fire region count
    - Temporal multi-frame confirmation to eliminate false positives
    - Spatial directional steering guidance (LEFT, CENTER, RIGHT)
    - Center coordinates (cx, cy) and apparent fire area for robot distance estimation
    - Robot DQN state data hook ready for integration

Usage:
    Default (Webcam index 0, Auto GPU/CPU):
        python main.py

    External Webcam:
        python main.py --camera 1

    Change Confidence Threshold:
        python main.py --conf 0.35

    Change Consecutive Confirmation Frames:
        python main.py --frames 4

    Force CPU or GPU:
        python main.py --device cpu
        python main.py --device 0

    Use Specific Model:
        python main.py --model models/fire_weights.pt
        python main.py --model models/best.pt

    Detect Smoke Too:
        python main.py --smoke

    Test on Video File:
        python main.py --video sample.mp4

Press 'Q' or 'ESC' to exit cleanly.
"""

import argparse
import os
import sys
import time
import cv2
from detector import FireDetector


def get_default_model():
    """Returns the preferred local model weights path."""
    if os.path.exists("models/fire_weights.pt"):
        return "models/fire_weights.pt"
    return "models/best.pt"


def parse_arguments():
    """Parses command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Real-Time Fire Detection System for Webcam & Fire-Fighting Robot"
    )
    parser.add_argument(
        "--camera",
        type=int,
        default=0,
        help="Webcam device index (default: 0). Use 1 or 2 for external USB cameras.",
    )
    parser.add_argument(
        "--video",
        type=str,
        default=None,
        help="Optional path to a recorded video file instead of a webcam.",
    )
    parser.add_argument(
        "--conf",
        type=float,
        default=0.35,
        help="Confidence threshold between 0.0 and 1.0 (default: 0.35).",
    )
    parser.add_argument(
        "--frames",
        type=int,
        default=4,
        help="Consecutive frames required to confirm fire detection (default: 4).",
    )
    parser.add_argument(
        "--model",
        type=str,
        default=get_default_model(),
        help="Path to YOLO model weights (default: models/fire_weights.pt or best.pt).",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="auto",
        help="Inference device: 'auto' (GPU if available, else CPU), '0' (GPU), or 'cpu'.",
    )
    parser.add_argument(
        "--smoke",
        action="store_true",
        help="Also detect smoke alongside fire.",
    )
    return parser.parse_args()


def draw_hud(frame, result, fps, conf_threshold, required_frames):
    """
    Renders visual bounding boxes, zone dividers, status banner,
    and telemetry data onto the OpenCV frame.
    """
    height, width = frame.shape[:2]
    left_x = int(width / 3.0)
    right_x = int(2.0 * width / 3.0)

    # 1. Draw vertical region boundary guidelines (LEFT | CENTER | RIGHT)
    cv2.line(frame, (left_x, 0), (left_x, height), (70, 70, 70), 1, cv2.LINE_AA)
    cv2.line(frame, (right_x, 0), (right_x, height), (70, 70, 70), 1, cv2.LINE_AA)

    # Zone labels at the top of the columns
    cv2.putText(frame, "LEFT", (int(width * 0.12), 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (140, 140, 140), 1, cv2.LINE_AA)
    cv2.putText(frame, "CENTER", (int(width * 0.46), 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (140, 140, 140), 1, cv2.LINE_AA)
    cv2.putText(frame, "RIGHT", (int(width * 0.82), 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (140, 140, 140), 1, cv2.LINE_AA)

    # 2. Draw all detected fire / smoke bounding boxes
    detections = result["detections"]
    for det in detections:
        x1, y1, x2, y2 = det["bbox"]
        conf = det["confidence"]
        class_name = det["class_name"]

        # Color: Orange/Red for FIRE (0, 69, 255), Silver/Gray for SMOKE (180, 180, 180)
        box_color = (0, 69, 255) if class_name == "FIRE" else (180, 180, 180)

        # Draw bounding rectangle
        cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)

        # Draw label header
        label = f"{class_name} {conf * 100:.1f}%"
        (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
        cv2.rectangle(
            frame,
            (x1, max(0, y1 - lh - 8)),
            (x1 + lw + 6, max(lh + 4, y1)),
            box_color,
            -1,
        )
        cv2.putText(
            frame,
            label,
            (x1 + 3, max(14, y1 - 4)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

    # 3. Highlight strongest fire center point & crosshair
    strongest = result["strongest"]
    if strongest is not None:
        cx, cy = strongest["center"]
        cv2.circle(frame, (cx, cy), 6, (0, 255, 255), -1)
        cv2.circle(frame, (cx, cy), 14, (0, 255, 255), 2)
        cv2.putText(
            frame,
            f"Center ({cx}, {cy})",
            (cx + 10, cy - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )

    # 4. Top Telemetry Dashboard (Overlay Panel)
    overlay = frame.copy()
    dashboard_h = 150
    cv2.rectangle(overlay, (10, 10), (410, 10 + dashboard_h), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
    cv2.rectangle(frame, (10, 10), (410, 10 + dashboard_h), (80, 80, 80), 1)

    # Status Header
    confirmed = result["confirmed"]
    streak = result["consecutive_count"]
    num_fires = sum(1 for d in detections if d["class_name"] == "FIRE")

    if confirmed:
        status_text = "FIRE CONFIRMED!"
        status_color = (0, 0, 255)  # Bright Red
    elif num_fires > 0:
        status_text = f"VERIFYING ({streak}/{required_frames})"
        status_color = (0, 165, 255)  # Orange
    else:
        status_text = "MONITORING - NO FIRE"
        status_color = (0, 220, 0)  # Green

    cv2.putText(
        frame,
        f"STATUS: {status_text}",
        (22, 34),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        status_color,
        2,
        cv2.LINE_AA,
    )

    # Telemetry metrics
    direction = result["fire_direction"]
    center_str = f"({strongest['center'][0]}, {strongest['center'][1]})" if strongest else "N/A"
    area_str = f"{strongest['area']:,} px2" if strongest else "N/A"
    strongest_class = strongest["class_name"] if strongest else "N/A"
    strongest_conf = f"{strongest['confidence']*100:.1f}%" if strongest else "N/A"

    cv2.putText(frame, f"Fires Detected : {num_fires} (Total: {len(detections)})", (22, 58),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (230, 230, 230), 1, cv2.LINE_AA)
    cv2.putText(frame, f"Fire Direction : {direction}", (22, 78),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255) if direction != "NONE" else (230, 230, 230), 1, cv2.LINE_AA)
    cv2.putText(frame, f"Strongest Pos  : {center_str} | Conf: {strongest_conf}", (22, 98),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (230, 230, 230), 1, cv2.LINE_AA)
    cv2.putText(frame, f"Area (Apparent): {area_str}", (22, 118),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (230, 230, 230), 1, cv2.LINE_AA)
    cv2.putText(frame, f"FPS: {fps:.1f}  |  Thresh: {conf_threshold:.2f}", (22, 138),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 180, 180), 1, cv2.LINE_AA)

    # Bottom exit helper
    cv2.putText(
        frame,
        "Press 'Q' or 'ESC' to Exit",
        (width - 210, height - 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (200, 200, 200),
        1,
        cv2.LINE_AA,
    )


def main():
    args = parse_arguments()

    if not 0.0 <= args.conf <= 1.0:
        print("[ERROR] Confidence threshold must be between 0.0 and 1.0.")
        sys.exit(1)

    if args.frames < 1:
        print("[ERROR] Consecutive frames threshold must be at least 1.")
        sys.exit(1)

    # Parse device flag (support '0' as int 0 for GPU, or 'cpu')
    target_device = 0 if args.device == "0" else args.device

    print("=" * 60)
    print("      REAL-TIME WEBCAM FIRE DETECTION SYSTEM      ")
    print("=" * 60)
    print(f"Confidence Threshold : {args.conf}")
    print(f"Consecutive Frames   : {args.frames}")
    print(f"Model Path           : {args.model}")
    print(f"Device               : {target_device}")
    print(f"Detect Smoke         : {args.smoke}")

    # 1. Initialize FireDetector
    detector = FireDetector(
        model_path=args.model,
        conf_threshold=args.conf,
        consecutive_frames_required=args.frames,
        detect_smoke=args.smoke,
        device=target_device,
    )

    # 2. Open Video Stream (Webcam or Video file)
    video_source = args.video if args.video else args.camera
    print(f"[INFO] Opening video capture from source: {video_source}")
    cap = cv2.VideoCapture(video_source)

    if not cap.isOpened():
        print(f"[ERROR] Could not open video source '{video_source}'.")
        if not args.video:
            print("[HINT] If using external USB webcam, try: python main.py --camera 1")
            print("[HINT] Ensure no other application (Zoom, Teams, Browser) is locking the webcam.")
        sys.exit(1)

    # Set camera resolution for smooth performance
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    print("[INFO] Stream started successfully. Press 'Q' or 'ESC' to exit.")

    # FPS timing variables
    prev_time = time.time()
    fps = 0.0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[INFO] End of video stream or failed to grab frame.")
                break

            # Calculate FPS
            curr_time = time.time()
            time_diff = curr_time - prev_time
            prev_time = curr_time
            if time_diff > 0:
                current_fps = 1.0 / time_diff
                fps = current_fps if fps == 0 else (0.9 * fps + 0.1 * current_fps)

            # --- Step 1: Detect Fire in the Current Frame ---
            result = detector.detect(frame)

            # --- Step 2: Telemetry Data for Robot Navigation ---
            # =========================================================================
            # [HOOK FOR ROBOT DQN NAVIGATION SYSTEM]
            # Extracted state variables ready to use:
            #   is_fire_confirmed = result["confirmed"]         # True/False
            #   direction_str     = result["fire_direction"]     # 'LEFT', 'CENTER', 'RIGHT', 'NONE'
            #   consecutive_hits  = result["consecutive_count"]  # streak count
            #
            #   if result["strongest"]:
            #       cx, cy    = result["strongest"]["center"]    # (x, y) coordinates in pixels
            #       fire_area = result["strongest"]["area"]      # size proxy for distance
            #       fire_conf = result["strongest"]["confidence"]
            #
            #   Example DQN state vector:
            #       state = [
            #           1.0 if is_fire_confirmed else 0.0,
            #           cx / frame.shape[1],           # Normalized horizontal center [0, 1]
            #           cy / frame.shape[0],           # Normalized vertical center [0, 1]
            #           fire_area / (frame.shape[0] * frame.shape[1]), # Normalized area
            #       ]
            #       action = dqn_agent.act(state)
            # =========================================================================

            # --- Step 3: Render Visual Annotations ---
            draw_hud(frame, result, fps, args.conf, args.frames)

            # Display the annotated frame
            cv2.imshow("Fire-Fighting Robot - Real-Time Fire Detection", frame)

            # Exit cleanly if 'q', 'Q', or ESC (key 27) is pressed
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == ord('Q') or key == 27:
                print("[INFO] Exit signal received.")
                break

    except KeyboardInterrupt:
        print("\n[INFO] KeyboardInterrupt received. Exiting...")
    finally:
        # Clean up resources
        cap.release()
        cv2.destroyAllWindows()
        print("[INFO] Video stream released and windows closed cleanly.")


if __name__ == "__main__":
    main()
