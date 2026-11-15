"""
One-time floor calibration for the camera planner.

Setup:
  1. Fix the webcam on the robot (tilted down so it sees ~1.5-2 m of floor).
     After calibration NEVER move or re-tilt it (or calibrate again).
  2. Put the robot on the floor. Stick 4 tape marks forming a square
     (default 40 x 40 cm), centred on the robot's centre line,
     with the near edge NEAR cm in front of the robot's front bumper.

          FL ●───────● FR        far edge  = NEAR + SIZE cm from bumper
             │       │
             │       │
          NL ●───────● NR        near edge = NEAR cm from bumper
                 ▲
             [ robot ]

Run:
  python calibrate_floor.py --cam 0 --near 30 --size 40

Keys:
  p      freeze / unfreeze frame (freeze first, then click)
  click  the 4 marks in order: near-left, near-right, far-right, far-left
  r      reset clicks
  s      save -> floor_calib.json
  q      quit
"""

import argparse
import json

import cv2
import numpy as np

ORDER = ["near-left", "near-right", "far-right", "far-left"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam", default="0")
    ap.add_argument("--image", help="use a saved photo instead of the camera")
    ap.add_argument("--near", type=float, default=30, help="cm from bumper to near tape edge")
    ap.add_argument("--size", type=float, default=40, help="square side length cm")
    ap.add_argument("--out", default="floor_calib.json")
    args = ap.parse_args()

    cap = None
    if args.image:
        still = cv2.imread(args.image)
        if still is None:
            raise SystemExit("cannot read image")
    else:
        src = int(args.cam) if args.cam.isdigit() else args.cam
        cap = cv2.VideoCapture(src)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)    # MUST match fire_nav.py
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        if not cap.isOpened():
            raise SystemExit("cannot open camera")
        still = None

    half = args.size / 2
    world = np.float32([[-half, args.near], [half, args.near],
                        [half, args.near + args.size], [-half, args.near + args.size]])
    pts = []
    frozen = still
    H = None

    def on_click(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(pts) < 4:
            pts.append((x, y))

    cv2.namedWindow("calibrate")
    cv2.setMouseCallback("calibrate", on_click)

    while True:
        if frozen is None:
            ok, frame = cap.read()
            if not ok:
                continue
        else:
            frame = frozen
        vis = frame.copy()
        for i, p in enumerate(pts):
            cv2.circle(vis, p, 6, (0, 0, 255), -1)
            cv2.putText(vis, ORDER[i], (p[0] + 8, p[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        if len(pts) == 4:
            cv2.polylines(vis, [np.int32(pts)], True, (0, 255, 0), 2)
            H = cv2.getPerspectiveTransform(np.float32(pts), world)
            # preview: top-down view of x -100..100, y 0..200 cm at 2 px/cm
            G = np.array([[2, 0, 200], [0, -2, 400], [0, 0, 1]], dtype=np.float64)
            top = cv2.warpPerspective(frame, G @ H, (400, 400))
            sq = (G @ np.vstack([world.T, np.ones(4)]))
            sq = (sq[:2] / sq[2]).T.astype(np.int32)
            cv2.polylines(top, [sq], True, (0, 255, 0), 1)
            cv2.putText(top, "tape square should look SQUARE here", (8, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.imshow("top-down preview", top)
        else:
            msg = f"click {ORDER[len(pts)]}" + ("" if frozen is not None else "  (press p to freeze)")
            cv2.putText(vis, msg, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.imshow("calibrate", vis)

        k = cv2.waitKey(20) & 0xFF
        if k == ord("q"):
            break
        if k == ord("p") and still is None:
            frozen = None if frozen is not None else frame.copy()
        if k == ord("r"):
            pts.clear()
            H = None
        if k == ord("s"):
            if H is None:
                print("click 4 points first")
                continue
            h, w = frame.shape[:2]
            with open(args.out, "w") as f:
                json.dump({"H": H.tolist(), "near_cm": args.near, "size_cm": args.size,
                           "frame_w": w, "frame_h": h, "img_pts": pts}, f, indent=2)
            print("saved", args.out)

    if cap:
        cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
