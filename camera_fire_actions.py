"""Show fire detection and the resulting robot decision on a live camera feed.

This is a visual simulation only: it assumes a clear path and does not connect
to an Arduino or control motors. Press q or Esc to quit.

Run: python camera_fire_actions.py
     python camera_fire_actions.py --camera 0 --model models/best.pt
"""
import argparse

import cv2

from config import FRONT_STOP_CM, REAR_CLEAR_CM
from detector import FireDetector
from navigation import Navigator, SensorData


def draw_status(frame, fire, decision):
    """Draw detections and the current controller decision on the frame."""
    for item in fire["detections"]:
        x1, y1, x2, y2 = item["bbox"]
        color = (0, 0, 255) if item["class_name"] == "FIRE" else (0, 165, 255)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        label = f"{item['class_name']} {item['confidence']:.0%} {item['direction']}"
        cv2.putText(frame, label, (x1, max(22, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

    lines = [
        f"FIRE: {'CONFIRMED' if fire['confirmed'] else 'not confirmed'}",
        f"DIRECTION: {fire['fire_direction']}",
        f"ROBOT STATE: {decision.state.name}",
        f"ACTION: {decision.action.value}",
        f"REASON: {decision.reason}",
        "SIMULATION ONLY - no motor control | q/Esc: quit",
    ]
    for index, line in enumerate(lines):
        y = 28 + index * 27
        cv2.putText(frame, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.58, (0, 255, 255), 1, cv2.LINE_AA)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", type=int, default=1,
                        help="OpenCV camera index (default: 1)")
    parser.add_argument("--model", default="models/fire_weights.pt",
                        help="YOLO weights path (default: models/fire_weights.pt)")
    args = parser.parse_args()

    camera = cv2.VideoCapture(args.camera)
    if not camera.isOpened():
        camera.release()
        raise RuntimeError(
            f"Could not open camera {args.camera}. Check the camera connection "
            "or try another index, such as --camera 1."
        )

    try:
        detector = FireDetector(model_path=args.model)
        navigator = Navigator(FRONT_STOP_CM, REAR_CLEAR_CM)
        last_action = None
        print("Camera fire-action simulation started. Press q or Esc to quit.")
        print("Assumed sensor readings: front=100 cm, rear=100 cm (clear path).")

        while True:
            ok, frame = camera.read()
            if not ok:
                print("Camera frame could not be read; stopping.")
                break

            fire = detector.detect(frame)
            # Camera-only demo: scripted clear-range readings allow the
            # Navigator to show its decisions. They are not real measurements.
            sensors = SensorData(front_distance=100.0, rear_distance=100.0)
            decision = navigator.update(fire, sensors, start=True)
            draw_status(frame, fire, decision)

            if decision.action != last_action:
                print(f"{decision.state.name}: {decision.action.value} - {decision.reason}")
                last_action = decision.action

            cv2.imshow("Fire Detection and Robot Actions (Simulation)", frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
