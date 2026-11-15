"""Central configuration for the firefighting robot prototype."""

# Serial settings. Set the correct port for your computer:
# Windows example: "COM5"; Linux example: "/dev/ttyACM0"
SERIAL_PORT = "COM5"
BAUD_RATE = 115200
SERIAL_TIMEOUT = 0.15

# Safety thresholds in centimetres. Tune only after sensor testing.
FRONT_STOP_CM = 25.0
REAR_CLEAR_CM = 20.0

# A fire direction is considered centered when its normalized x coordinate
# is within this fraction of the image center.
FIRE_CENTER_DEADBAND = 0.12

# Arduino command watchdog must be implemented on the Arduino too.
COMMAND_REFRESH_SECONDS = 0.20
