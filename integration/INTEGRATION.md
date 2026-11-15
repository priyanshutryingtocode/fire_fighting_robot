# Connecting `fire_fighting_robot` (fire detection) to the FireBot car

## What goes where

Copy these into the **root of the `fire_fighting_robot` repo** (next to `detector.py`):

```
fire_fighting_robot/
├── detector.py            (theirs, unchanged - YOLO + confirmation)
├── firebot_bridge.py      NEW  detector -> USB serial -> Arduino
├── planner.py             NEW  (optional) A* path planner
├── calibrate_floor.py     NEW  (optional) floor calibration for distance + A*
└── fire_bot/fire_bot.ino  NEW  upload THIS to the Arduino (not arduino/robot_controller.ino)
```

Why not the repo's `arduino/robot_controller.ino`? It's a scaffold: turning disabled, no relay/pump/OLED, and IN3/IN4 on pins 9/10 clash with relay/servo. `fire_bot.ino` has the full state machine (search -> drive -> avoid -> stop -> flame sensor -> relay -> pump), the OLED, and the safety rules.

## Data flow

```
webcam ─▶ FireDetector.detect(frame)          (repo code: YOLO, 3-frame confirmation)
            │  strongest fire box, confidence
            ▼
         firebot_bridge.py
            │  err  = fire centre left/right  (-100..+100)
            │  dist = cm (floor or size calibration)
            ▼  USB serial 115200, ~10-30 lines/s
         "F,-20,85,82"  or  "N"
            ▼
         Arduino fire_bot.ino ─▶ motors (L298N), flame sensor, relay ─▶ pump, OLED
            │
            └─▶ "S,TRACK,400,400,400,0,F,85,0"  (telemetry, shown in bridge window/console)
```

The Arduino IDE is only for **uploading**. While running, Python owns the USB port, so **close Serial Monitor**. The bridge console shows what Serial Monitor would.

## Steps

1. **Arduino**: open `fire_bot/fire_bot.ino`, check the settings block at the top (`USE_ULTRASONIC`, `RELAY_ACTIVE_LOW`, motor inverts — same as your test sketches), upload. Close Serial Monitor.
2. **Laptop** (repo venv active):
   ```
   pip install pyserial
   ```
3. **Camera only** (no robot):
   ```
   python firebot_bridge.py --no-serial
   ```
   Fire box should go red and status says `FIRE CONFIRMED`. Wrong camera → `--camera 0`.
4. **Distance calibration** (pick one):
   - Size (quick): fire 50 cm from bumper → `python firebot_bridge.py --no-serial --ref-dist 50` → press `c`. Fire at 150 cm → same with `--ref-dist 150` → `c`. Saved to `size_calib.json`.
   - Floor (best, also enables A*): `python calibrate_floor.py --cam 1 --near 30 --size 40` (see FireBot README).
5. **Robot, wheels in the air**:
   ```
   python firebot_bridge.py --port COM5
   ```
   Window shows `ROBOT: SEARCH ...` = Arduino is talking. Show fire → state becomes TRACK, wheels steer toward it.
6. **Floor run**: robot on floor, candle 2 m away in a tray, water nearby.

## Keys (click the video window first)

| Key | Action |
|---|---|
| space | E-stop on/off |
| w a s d x | manual drive (forward / left / back / right / stop) |
| r | back to AUTO |
| c | add size-calibration point (fire at `--ref-dist`) |
| q / Esc | quit (robot stops) |

## Options

`--camera 1` (repo default) · `--model models/fire_weights.pt` · `--conf 0.35` · `--frames 3` (confirmation frames) · `--device auto|cpu|0` · `--baud 9600` for HC-05 Bluetooth.

## Troubleshooting

| Problem | Fix |
|---|---|
| `could not open port` | Serial Monitor still open, or wrong COM port (Arduino IDE → Tools → Port shows it) |
| Window says `no telemetry yet` | wrong port / baud, or old sketch on Arduino |
| Robot state stays SEARCH with fire visible | fire not confirmed: check status line, lower `--conf`, `--frames 2` |
| Robot shows camera offline (`-`) on OLED | detection too slow: use GPU (`--device 0`) — firmware treats >0.8 s silence as offline |
| Steers the wrong way | swap `INVERT_LEFT_MOTOR`/`INVERT_RIGHT_MOTOR`, or camera mounted upside down |
