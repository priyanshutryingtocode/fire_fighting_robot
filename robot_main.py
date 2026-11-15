"""Run the laptop-side detector + navigation prototype."""
import argparse
import time
import cv2

from detector import FireDetector
from navigation import Navigator, SensorData, Action
from arduino_communication import ArduinoLink
from config import SERIAL_PORT, BAUD_RATE, SERIAL_TIMEOUT, FRONT_STOP_CM, REAR_CLEAR_CM


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--model", default="models/fire_weights.pt")
    parser.add_argument("--port", default=SERIAL_PORT)
    parser.add_argument("--simulate", action="store_true",
                        help="Run camera and navigation logic without Arduino/motor commands")
    parser.add_argument("--start", action="store_true",
                        help="Enable prototype movement decisions (otherwise remain stopped)")
    args = parser.parse_args()

    detector = FireDetector(model_path=args.model)
    navigator = Navigator(FRONT_STOP_CM, REAR_CLEAR_CM)
    link = None
    cap = cv2.VideoCapture(args.camera)

    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera index {args.camera}")

    if not args.simulate:
        link = ArduinoLink(args.port, BAUD_RATE, SERIAL_TIMEOUT)

    latest = None
    last_command = None
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break

            fire = detector.detect(frame)
            if link:
                incoming = link.read_telemetry()
                if incoming is not None:
                    latest = incoming
            else:
                # Simulation intentionally supplies no valid range, so the
                # navigator stays SAFE_STOP until simulated ranges are added.
                latest = None

            sensors = SensorData(
                front_distance=latest.get("front_distance") if latest else None,
                rear_distance=latest.get("rear_distance") if latest else None,
                flame_detected=latest.get("flame_detected") if latest else None,
                pir_detected=latest.get("pir_detected") if latest else None,
            )
            decision = navigator.update(fire, sensors, start=args.start)

            # Do not issue repeated identical commands unnecessarily.
            if link and decision.action != last_command:
                link.send_action(decision.action)
                last_command = decision.action

            cv2.putText(frame, f"STATE: {decision.state.name}", (15, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
            cv2.putText(frame, f"ACTION: {decision.action.value}", (15, 58),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            cv2.putText(frame, f"WHY: {decision.reason}", (15, 86),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            if latest:
                cv2.putText(frame, f"Front: {latest['front_distance']:.1f} cm  "
                                   f"Rear: {latest['rear_distance']:.1f} cm",
                            (15, 114), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (255, 255, 255), 1)

            cv2.imshow("Fire Fighting Robot - Prototype", frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            time.sleep(0.01)
    finally:
        if link:
            link.close()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
