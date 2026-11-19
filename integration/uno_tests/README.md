# Arduino Uno function tests

Each subfolder contains a standalone Arduino IDE sketch. Open the `.ino` file
inside its matching folder, select **Arduino Uno**, verify pins/polarity against
your wiring, and upload one sketch at a time. These tests are separate from the
integrated `../fire_bot/fire_bot.ino` firmware.

## Recommended order

1. `flame_sensor_test` — confirm the sensor output and polarity.
2. `ultrasonic_test` — verify each distance channel.
3. `motor_wasd_test` — lift the wheels, then test direction with short pulses.
4. `servo_arm_test` — disconnect/linkage-clear the arm while finding safe angles.
5. `relay_test` — test with the pump disconnected first; use water and a safe
   setup before testing a pump.

These sketches use the connections from the pump test plus the complete
implementation's added hardware: front sonar on D2/D3 with the L298N map (D2/D7
with the L293D map), pan servo D9, and tilt servo D4. Flame is A0, relay D10,
OLED A4/A5, and LED D13. The motor test supports `MOTOR_DRIVER`: `0` is the
separate-enable L298N map; `1` is the L293D board map. Change it if your driver
differs. Check the active-low settings against your own sensor and relay tests.

For Serial Monitor tests, close the Python bridge, select **115200 baud** and
**Newline** line ending. Send one command per line. Never work near a real fire.
Keep a physical battery disconnect within reach; a software stop is not a
replacement for it.
