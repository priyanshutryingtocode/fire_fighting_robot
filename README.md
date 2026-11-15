# Fire-Fighting Robotic Car — Software Prototype

This folder combines the existing laptop-side fire detector (`main.py` and
`detector.py`) with a first navigation/Arduino integration scaffold.

## What is included

```text
fire_fighting_robot/
├── main.py                      # Original camera + YOLO demo (preserved)
├── detector.py                  # Original FireDetector class (preserved)
├── robot_main.py                # New laptop-side robot application
├── navigation.py                # Finite-state navigation decisions
├── obstacle_avoidance.py        # Ultrasonic reading validation helper
├── arduino_communication.py     # USB serial protocol
├── fire_response.py             # Future fire-response workflow placeholder
├── config.py                    # Ports and safety thresholds
├── requirements.txt
├── tests/
│   └── test_navigation.py
└── arduino/
    └── robot_controller.ino     # Arduino telemetry + motor watchdog scaffold
```

## Architecture

1. The camera is connected to the laptop.
2. `detector.py` runs YOLO and returns fire detections, temporal confirmation,
   direction (`LEFT`, `CENTER`, `RIGHT`, `NONE`) and the strongest detection.
3. `navigation.py` combines that result with front/rear ultrasonic readings.
4. `arduino_communication.py` exchanges newline-terminated messages over USB.
5. The Arduino reads sensors and controls the motor driver. It must enforce
   its own stop-distance checks and command watchdog; laptop decisions are
   not a substitute for local safety.

The current navigation policy is intentionally conservative:
- No start request -> STOP.
- Missing front range -> SAFE_STOP.
- Front obstacle -> reverse only if rear clearance is known and sufficient;
  otherwise STOP.
- Confirmed fire -> request left/right alignment, or STOP if centered.
- No confirmed fire -> request forward search, subject to Arduino safety checks.

**Important limitation:** two front/rear sensors do not measure left/right
clearance. This prototype does not autonomously choose a turn direction for
obstacle avoidance. Do not use it in a cluttered or hazardous environment.

## Setup

Use Python 3.10 or newer (Python 3.11 is a good choice).

```bash
python -m venv .venv
```

Activate it:

Windows:
```powershell
.venv\Scripts\activate
```

macOS/Linux:
```bash
source .venv/bin/activate
```

Install dependencies:
```bash
python -m pip install --upgrade pip
pip install -r requirements.txt
```

Place your trained model weights at `models/fire_weights.pt` or
`models/best.pt`. The existing detector contains a fallback model download
path if the configured weights are absent; inspect that behavior before
using the project in an offline deployment.

## Test navigation without motors

Run the unit tests:
```bash
python -m unittest discover -s tests -v
```

Run the original fire-detection camera demo:
```bash
python main.py
```

Run the new robot application in simulation mode:
```bash
python robot_main.py --simulate
```

Simulation mode deliberately has no valid ultrasonic readings, so it should
remain stopped. It is for verifying camera/detector/UI integration, not
simulating a complete physical environment.

## Arduino setup and serial protocol

1. Open `arduino/robot_controller.ino` in Arduino IDE.
2. Review every pin assignment against your actual wiring before uploading.
3. Select Arduino UNO and the correct COM/serial port.
4. Upload the sketch.
5. Set `SERIAL_PORT` in `config.py` (for example `COM5` on Windows).
6. Start with wheels lifted off the ground and motor power disconnected.
7. Verify telemetry first; only then perform supervised low-speed motor tests.

Telemetry line:
```text
SENSORS,<front_cm>,<rear_cm>,<flame_0_or_1>,<pir_0_or_1>
```

Laptop commands:
```text
STOP
FORWARD
REVERSE
TURN_LEFT
TURN_RIGHT
```

Every command is newline terminated. The sketch stops the motors if no valid
command arrives before its watchdog expires. It also blocks forward movement
when its front ultrasonic measurement is too close, and reverse movement when
rear clearance is too small.

## Hardware pin assumptions in the example sketch

The Arduino sketch includes an example pin map for:
- L293D motor inputs and enable pins
- front and rear HC-SR04-style ultrasonic sensors
- flame sensor digital output
- PIR digital output

These are **example assignments only**, not verified against your robot.
Check the wiring and avoid pin conflicts before powering the circuit. Servo
pins are reserved as future integration work. No water pump is activated by
this software.

## Current scope and known limitations

- This is an integration scaffold, not a finished autonomous firefighting
  robot.
- No mapping, SLAM, wheel odometry, encoder feedback or trained DQN is present.
- Camera bounding-box area is not a calibrated distance measurement.
- A single flame sensor is not a reliable directional sensor.
- PIR detects motion/changes in infrared radiation; it does not confirm fire.
- The laptop-side `--start` flag enables movement decisions, but physical
  motion must still be tested with wheels raised and a hardware emergency stop.
- The firmware motor pin mapping and sensor polarity must be checked against
  the actual circuit.
- Do not test near real flames. Use a safe visual test target and keep the
  water/pump system disconnected during software commissioning.

## Recommended implementation order

1. Verify camera and YOLO output using `python main.py`.
2. Run navigation unit tests.
3. Upload Arduino sketch with motor power disconnected; verify serial telemetry.
4. Test STOP and watchdog with wheels raised.
5. Test each motor direction at low power under supervision.
6. Verify ultrasonic stop behavior and sensor failure behavior.
7. Only then integrate alignment and arm actuation.
8. Design a separately tested suppression subsystem with suitable safety
   interlocks and a controlled test environment.
