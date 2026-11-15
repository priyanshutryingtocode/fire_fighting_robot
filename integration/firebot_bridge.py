"""Bridge the repository fire detector to the FireBot Arduino controller.

Connects THIS repo's fire detection (detector.FireDetector) to the robot car:

    webcam -> FireDetector.detect()  (YOLO + temporal confirmation + bbox/center)
           -> direction  (err  -100 .. +100)
           -> distance   (cm, calibrated)
           -> USB serial -> Arduino running fire_bot/fire_bot.ino
                              (drives motors, avoids obstacles, flame sensor -> relay -> pump, OLED)

Messages sent to the Arduino (one per line):
    F,<err>,<dist_cm>,<conf>   confirmed fire (dist 999 = unknown)
    N                          no confirmed fire
    X / G                      emergency stop / go
    w a s d x / r              manual drive / back to auto

Arduino replies every 200 ms (shown on screen + printed here, this is your "serial monitor"):
    S,<state>,<dF>,<dL>,<dR>,<flame>,<vision>,<dist>,<pump>

Run (from repo root, venv active):
    python integration/firebot_bridge.py --port COM5
    python integration/firebot_bridge.py --no-serial
    python integration/firebot_bridge.py --port COM5 --camera 0

Keys (click the video window first):
    space  E-stop on/off           q / Esc  quit (robot stops)
    w a s d x  manual drive         r  back to AUTO
    c      add a size-distance calibration point

NOTE: close Arduino IDE's Serial Monitor first - only one program can use the port.
"""

import argparse
import json
import math
import sys
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np

# Support running this script directly from the integration folder or from
# any working directory while importing the detector kept in the repo root.
INTEGRATION_DIR = Path(__file__).resolve().parent
REPO_ROOT = INTEGRATION_DIR.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from detector import FireDetector

try:
    import serial
except ImportError:
    serial = None

# ---------------------------------------------------------------- distance
class DistanceEstimator:
    """
    FLOOR : floor_calib.json (calibrate_floor.py) -> bottom of fire box mapped to floor cm. Best.
    SIZE  : size_calib.json  -> dist = a / box_height + b, from 2+ points taken with key 'c'.
    none  : 999 (Arduino then relies on flame sensor + ultrasonics to stop).
    """

    def __init__(self, floor_homography=None, size_file=None):
        self.floor_homography = (np.asarray(floor_homography, dtype=np.float64)
                                 if floor_homography is not None else None)
        self.size_file = Path(size_file) if size_file else INTEGRATION_DIR / "size_calib.json"
        self.points, self.a, self.b = [], None, None
        if self.size_file.exists():
            with self.size_file.open() as f:
                self.points = [tuple(p) for p in json.load(f)["points"]]
            self._fit()
        self.hist = deque(maxlen=5)

    @property
    def method(self):
        if self.floor_homography is not None:
            return "FLOOR"
        return f"SIZE({len(self.points)}pt)" if self.a is not None else "none"

    def _fit(self):
        if not self.points:
            self.a = self.b = None
        elif len(self.points) == 1:
            h, d = self.points[0]
            self.a, self.b = d * h, 0.0
        else:
            x = np.array([1.0 / h for h, _ in self.points])
            y = np.array([d for _, d in self.points])
            self.a, self.b = (float(v) for v in np.polyfit(x, y, 1))

    def update(self, bbox):
        x1, y1, x2, y2 = bbox
        d = None
        if self.floor_homography is not None:
            point = self.floor_homography @ np.array([(x1 + x2) / 2, y2, 1.0])
            if abs(point[2]) > 1e-9:
                fx, fy = point[0] / point[2], point[1] / point[2]
                if fy > 0:
                    d = math.hypot(fx, fy)
        if d is None and self.a is not None:
            d = self.a / max(4.0, y2 - y1) + self.b
        if d is None:
            return 999
        self.hist.append(d)
        return int(min(998, max(0, round(float(np.median(self.hist))))))

    def reset(self):
        self.hist.clear()

    def add_point(self, bbox, ref_dist):
        h = bbox[3] - bbox[1]
        self.points = [p for p in self.points if abs(p[1] - ref_dist) > 1] + [(h, ref_dist)]
        self._fit()
        with self.size_file.open("w") as f:
            json.dump({"points": self.points, "a": self.a, "b": self.b}, f, indent=2)
        print(f"[CAL] box {h}px at {ref_dist:.0f} cm saved ({len(self.points)} point(s); use 2+, e.g. 50 & 150 cm)")


# ---------------------------------------------------------------- detector -> messages
class Bridge:
    """Turns one FireDetector.detect() result into protocol lines."""

    def __init__(self, dist_est):
        self.dist = dist_est
        self.smooth_err = 0.0
        self.last_dist = 999

    @staticmethod
    def fire_target(fire):
        s = fire.get("strongest")
        if fire.get("confirmed") and s is not None and s.get("class_name") == "FIRE":
            return s
        return None

    def messages(self, fire, frame):
        _, w = frame.shape[:2]
        msgs = []
        s = self.fire_target(fire)
        if s is not None:
            cx = s["center"][0]
            err = (cx / w - 0.5) * 200.0
            self.smooth_err = 0.6 * self.smooth_err + 0.4 * err
            self.last_dist = self.dist.update(s["bbox"])
            msgs.append(f"F,{int(round(self.smooth_err))},{self.last_dist},{int(round(s['confidence'] * 100))}")
        else:
            msgs.append("N")
            self.dist.reset()
            self.last_dist = 999
        return msgs


def parse_telemetry(line):
    p = line.split(",")
    if len(p) < 9 or p[0] != "S":
        return None
    try:
        return {"state": p[1], "dF": int(p[2]), "dL": int(p[3]), "dR": int(p[4]),
                "flame": p[5] == "1", "vision": p[6], "dist": int(p[7]), "pump": p[8] == "1"}
    except ValueError:
        return None


# ---------------------------------------------------------------- main
def parse_args():
    ap = argparse.ArgumentParser(description="FireDetector -> FireBot Arduino bridge")
    ap.add_argument("--camera", default="1", help="webcam index (repo default 1) or stream URL")
    ap.add_argument("--model", default=str(REPO_ROOT / "models" / "fire_weights.pt"))
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--frames", type=int, default=3, help="consecutive frames to confirm fire")
    ap.add_argument("--device", default="auto", help="auto / cpu / 0")
    ap.add_argument("--port", help="Arduino port: COM5, /dev/ttyACM0 ...")
    ap.add_argument("--baud", type=int, default=115200, help="115200 USB, 9600 HC-05")
    ap.add_argument("--no-serial", action="store_true")
    ap.add_argument("--calib", default=str(INTEGRATION_DIR / "floor_calib.json"),
                    help="optional camera-to-floor calibration for distance estimates")
    ap.add_argument("--ref-dist", type=float, default=100, help="cm, for key 'c'")
    return ap.parse_args()


def open_serial(port, baud):
    if serial is None:
        raise SystemExit("pip install pyserial")
    ser = serial.Serial(port, baud, timeout=0)
    print(f"[SERIAL] {port} @ {baud} - waiting for Arduino reset...")
    time.sleep(2.0)
    ser.reset_input_buffer()
    ser.write(b"G\n")                 # clear any E-stop from a previous run
    return ser


def main():
    args = parse_args()

    floor_homography = None
    calibration_path = Path(args.calib)
    if calibration_path.exists():
        with calibration_path.open() as f:
            floor_homography = json.load(f).get("H")
        if floor_homography is None:
            raise SystemExit(f"Calibration file has no 'H' matrix: {calibration_path}")
    dist_est = DistanceEstimator(floor_homography)
    bridge = Bridge(dist_est)
    print(f"[DIST] method: {dist_est.method}"
          + ("" if dist_est.method != "none" else "  (press 'c' with fire at --ref-dist cm to calibrate)"))

    detector = FireDetector(model_path=args.model, conf_threshold=args.conf,
                            consecutive_frames_required=args.frames, device=args.device)

    src = int(args.camera) if str(args.camera).isdigit() else args.camera
    cap = cv2.VideoCapture(src)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if not cap.isOpened():
        raise SystemExit(f"Could not open camera {args.camera} (try --camera 0)")

    ser = None
    if not args.no_serial:
        if not args.port:
            raise SystemExit("Give --port COMx (or --no-serial)")
        ser = open_serial(args.port, args.baud)

    estop, manual = False, False
    telem, telem_raw, rx = None, "", b""
    t_prev, fps = time.time(), 0.0
    last_print = 0.0

    def send(line):
        if ser:
            ser.write((line + "\n").encode("ascii"))

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                send("N")
                time.sleep(0.05)
                continue

            fire = detector.detect(frame)
            msgs = bridge.messages(fire, frame)
            for m in msgs:
                send(m)

            if ser:
                rx += ser.read(ser.in_waiting or 0)
                while b"\n" in rx:
                    line, rx = rx.split(b"\n", 1)
                    line = line.decode(errors="ignore").strip()
                    t = parse_telemetry(line)
                    if t:
                        telem, telem_raw = t, line
                    elif line:
                        print("[ARDUINO]", line)

            now = time.time()
            fps = 0.9 * fps + 0.1 / max(1e-3, now - t_prev)
            t_prev = now
            if now - last_print > 0.5:
                last_print = now
                print(f"\r{fps:4.1f} fps | sent {' '.join(msgs):28s} | {telem_raw:45s}", end="", flush=True)

            # ---------------- display
            vis = frame.copy()
            h, w = vis.shape[:2]
            cv2.line(vis, (w // 2, 0), (w // 2, h), (255, 255, 255), 1)
            for d in fire["detections"]:
                x1, y1, x2, y2 = d["bbox"]
                col = (0, 0, 255) if d["class_name"] == "FIRE" else (0, 165, 255)
                cv2.rectangle(vis, (x1, y1), (x2, y2), col, 2)
                cv2.putText(vis, f"{d['class_name']} {d['confidence']:.0%}", (x1, max(18, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)
            target = Bridge.fire_target(fire)
            status = (f"FIRE CONFIRMED  {bridge.last_dist if bridge.last_dist < 999 else '?'} cm"
                      if target else f"no fire ({fire['consecutive_count']}/{args.frames})")
            lines = [f"{fps:4.1f} fps   dist: {dist_est.method}",
                     status,
                     "send: " + " | ".join(msgs)]
            if telem:
                lines.append(f"ROBOT: {telem['state']}  flame:{'YES' if telem['flame'] else 'no'}"
                             f"  pump:{'ON' if telem['pump'] else 'off'}")
                lines.append(f"sonar F{telem['dF']} L{telem['dL']} R{telem['dR']}")
            elif ser:
                lines.append("ROBOT: no telemetry yet")
            if manual:
                lines.append("MANUAL (w a s d x, r = auto)")
            for i, t in enumerate(lines):
                cv2.putText(vis, t, (10, 22 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            if estop:
                cv2.putText(vis, "E-STOP", (w - 140, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
            cv2.imshow("FireBot bridge", vis)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord(" "):
                estop = not estop
                send("X" if estop else "G")
            if key in (ord("w"), ord("a"), ord("s"), ord("d"), ord("x")):
                manual = True
                send(chr(key))
            if key == ord("r"):
                manual = False
                send("R")
            if key == ord("c"):
                if target is None:
                    print(f"\n[CAL] no confirmed fire - put fire at {args.ref_dist:.0f} cm first")
                else:
                    dist_est.add_point(target["bbox"], args.ref_dist)
    finally:
        if ser:
            ser.write(b"X\n")           # leave robot stopped
            ser.close()
        cap.release()
        cv2.destroyAllWindows()
        print()


if __name__ == "__main__":
    main()
