"""Line-based serial protocol for the Arduino UNO."""
import time
import serial


class ArduinoLink:
    def __init__(self, port, baud=115200, timeout=0.15):
        self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)
        # Opening an Arduino serial port commonly resets the board.
        time.sleep(2.0)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.last_rx = time.monotonic()

    def send_action(self, action):
        self.ser.write((action.value + "\n").encode("ascii"))

    def read_telemetry(self):
        """Return latest parsed telemetry dict, or None if no complete line."""
        latest = None
        while self.ser.in_waiting:
            raw = self.ser.readline().decode("ascii", errors="ignore").strip()
            if not raw:
                continue
            self.last_rx = time.monotonic()
            parts = raw.split(",")
            try:
                if parts[0] == "SENSORS" and len(parts) >= 5:
                    latest = {
                        "front_distance": float(parts[1]),
                        "rear_distance": float(parts[2]),
                        "flame_detected": bool(int(parts[3])),
                        "pir_detected": bool(int(parts[4])),
                    }
            except (ValueError, IndexError):
                continue
        return latest

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.write(b"STOP\n")
            self.ser.close()
