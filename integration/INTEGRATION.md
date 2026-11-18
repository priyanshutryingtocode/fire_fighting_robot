# FireBot integration

This folder connects the repository's `detector.py` to the FireBot Arduino
firmware. The scripts import `detector.py` from the repository root, so you can
run them from the repository root or from this folder without copying files.

## Files and data flow

```text
webcam -> FireDetector -> firebot_bridge.py -> USB serial -> fire_bot/fire_bot.ino
                                  |                            |
                                  +-- distance calibration     +-- motors, sonar,
                                                                  flame sensor, relay,
                                                                  OLED
```

- `firebot_bridge.py` detects and confirms fire, sends direction, confidence,
  and estimated distance. Fire-center offset drives local visual steering; the
  Uno uses that offset to turn and move toward the target.
- `calibrate_floor.py` maps image points to floor coordinates. This is used
  only for an optional floor-based distance estimate. Fire centering and
  obstacle avoidance do not require this calibration.
- `fire_bot/fire_bot.ino` performs camera-based tracking, ultrasonic obstacle
  avoidance, close approach, arm/nozzle aiming, flame-sensor verification, and
  pump control. After extinguishing and verification it returns to SEARCH.

The intended cycle is: timed in-place scan -> confirmed fire -> keep the fire
centered while moving -> use ultrasonic readings to avoid obstacles -> reacquire
and center the fire -> creep until the flame sensor detects it -> stop and aim
the arm/nozzle -> pump -> verify -> scan again.
`SEARCH_REVOLUTION_MS` must be calibrated to the robot's measured 360-degree
turn time; without wheel encoders or a compass, timed rotation is approximate.

## Run from the repository root

Install the integration dependencies if needed:

```powershell
python -m pip install ultralytics opencv-python numpy pyserial
```

Camera only:

```powershell
python integration/firebot_bridge.py --no-serial
```

Connect to an Arduino (close Arduino IDE Serial Monitor first):

```powershell
python integration/firebot_bridge.py --port COM5
```

Fire centering works without floor calibration. The optional calibration
improves fire-distance estimates, which the Arduino uses for approach and pump
interlocks. Ultrasonic obstacle avoidance runs on the Arduino.

Use `--camera 0` to choose another camera. The default model path points to
`models/fire_weights.pt` in the repository root. You can override it with
`--model path/to/weights.pt`.

## Distance calibration

For floor-based distance, mount and aim the camera, then calibrate four tape
corners on the floor:

```powershell
python integration/calibrate_floor.py --cam 1 --near 30 --size 40
```

Freeze with `p`, click near-left, near-right, far-right, far-left, then press
`s`. The default output is `integration/floor_calib.json`; the bridge loads it
automatically. Recalibrate if the camera moves.

Alternatively, gather at least two fire-box-size samples at known distances by
running the bridge with `--ref-dist 50`, pressing `c`, then repeating at another
distance. Samples are saved as `integration/size_calib.json`.

## Controls

Click the video window before using the bridge keys:

| Key | Action |
| --- | --- |
| Space | Toggle emergency stop |
| `w`, `a`, `s`, `d`, `x` | Manual forward, left, reverse, right, stop |
| `r` | Return to automatic mode |
| `c` | Save a size calibration sample at `--ref-dist` |
| `q` or Esc | Quit; the bridge sends an emergency stop |

The bridge sends `F` with confirmed fire center, distance, and confidence, or
`N` when there is no confirmed fire, plus emergency/manual commands. The Arduino
uses the camera center offset for local steering and its ultrasonic sensors for
obstacle avoidance. There is no room map or global route planner.

### Control from Arduino IDE Serial Monitor

Close the Python bridge first because it owns the Arduino serial port. Open
Serial Monitor at **115200 baud** with line ending set to **Newline**, then send
one command at a time:

| Command | Action |
| --- | --- |
| `W` | Forward |
| `A` | Turn left in place |
| `S` | Reverse |
| `D` | Turn right in place |
| `x` | Stop motors and remain in manual mode |
| `r` | Return to automatic search |
| `X` | Latch software emergency stop |
| `G` | Clear software emergency stop |

Movement commands remain active until another command is sent. Send `x` to
stop before changing modes. `X` is the emergency stop; it is distinct from the
lowercase `x` manual stop. Keep a physical power cutoff within reach.

## Hardware configuration

Before uploading `fire_bot/fire_bot.ino`, verify the pin map, motor direction,
flame sensor polarity, relay polarity, and driver type against the actual
hardware. The sketch defaults to `CALIBRATION_MODE 0`, `USE_ULTRASONIC 0`, and
`USE_SERVO 0`. Configure and verify these for the actual
hardware before operation. With `USE_SERVO 0`, the firmware will hold in AIM
while the flame sensor is active and will not activate the pump. Only enable the servo after checking its
mechanism, pin, travel limits, and the `SERVO_CENTER`/`SERVO_RANGE` calibration.
Test with the pump disconnected and wheels raised. Do not test near a real
flame.
