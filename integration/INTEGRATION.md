# FireBot dual-servo integration

The camera detector runs on the laptop. The Arduino Uno receives confirmed
fire center and distance over serial, steers the car using the fire's horizontal
camera error, avoids obstacles using one front ultrasonic sensor, and aims the
two arm servos before controlling the pump. There is no global map or A* planner.

## Wiring used by the complete sketch

| Part | Arduino connection |
| --- | --- |
| Flame sensor digital OUT | A0, active LOW |
| Relay input | D10, active LOW |
| Pan servo (left/right) | D9, same servo signal as the pump test |
| Tilt/base servo (up/down) | D4 |
| Front HC-SR04 TRIG | D2 |
| Front HC-SR04 ECHO with L298N | D3 |
| Front HC-SR04 ECHO with L293D board | D7 |
| OLED SH1106 | SDA A4, SCL A5 |
| Status LED | D13 |

The motor pin map is selected by `MOTOR_DRIVER` near the top of
`fire_bot/fire_bot.ino`: `0` is the separate-enable L298N map; `1` is the L293D
board map. Verify all motor pins and motor inversion against your wiring.

Use a separate regulated supply for both servos and connect that supply's
ground to Arduino ground. Keep the pump on its separate battery through the
relay as in the pump test. Check both servo travel limits with the linkage
clear before attaching the arm. Adjust `PAN_MIN/MAX`, `TILT_MIN/MAX`, and the
reversal flags to match the mechanism.

## Behavior

1. At startup the car scans in place. `SEARCH_REVOLUTION_MS` is a timed estimate
   of one 360-degree turn and must be calibrated on the actual floor.
2. Laptop YOLO confirms fire and sends horizontal and vertical image offsets.
   The car turns/moves to center the fire; the vertical offset is used to aim
   the tilt servo.
3. The one front sonar triggers local avoidance. The car stops, pivots briefly,
   checks the front distance again, and tries the other direction if still
   blocked. It does not reverse because there is no rear sensor.
4. At close range the car creeps until the flame sensor confirms flame, then
   stops. Pan and tilt aim, the pan servo sweeps while the pump runs, and the
   flame sensor controls the spray hold and verification cycle.
5. When the flame stays clear, the car returns to scanning.

The serial vision packet is `F,<x_error>,<y_error>,<distance_cm>,<confidence>`;
`N` means no confirmed fire. Camera centering works without floor calibration.
Optional `integration/floor_calib.json` or size calibration supplies a distance
estimate for approach/pump interlocks.

## Upload and run

Install the U8g2 library through Arduino IDE Library Manager if the OLED is
enabled. Open `integration/fire_bot/fire_bot.ino`, select Arduino Uno and its
port, verify the driver/pin settings, then upload. Close Serial Monitor before
starting the laptop bridge because both cannot own the USB serial port.

From the repository root, run:

```powershell
python integration/firebot_bridge.py --port COM5
```

Change `COM5` to the Arduino's port. Use `--camera 0` if needed. For camera-only
preview, run `python integration/firebot_bridge.py --no-serial`.

## Commissioning

Test the standalone sketches in `integration/uno_tests` first. Then test this
full sketch with motor wheels raised and pump disconnected. Confirm flame and
sonar readings, motor directions, servo movement, relay polarity, and the stop
command before connecting the pump. The front sensor cannot see obstacles
behind or beside the car; supervise every movement and keep a physical power
cutoff within reach. Do not test near a real flame.

Serial Monitor manual commands, when the bridge is closed: `W/A/S/D` drive,
`x` stops, `r` returns to automatic mode, `X` latches the software emergency
stop, and `G` clears it. Commands require Newline at 115200 baud.
