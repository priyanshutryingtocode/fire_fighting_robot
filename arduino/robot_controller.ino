/*
 * Arduino UNO controller scaffold.
 *
 * IMPORTANT:
 * - Example pin map only. Verify against your actual wiring before upload.
 * - Test with motor power disconnected and wheels raised.
 * - Use a physical emergency stop and a suitable motor power supply.
 * - Do not power motors/servos from the Arduino 5V pin.
 */

const int ENA = 5;   // L293D enable motor A (PWM)
const int IN1 = 7;
const int IN2 = 8;
const int ENB = 6;   // L293D enable motor B (PWM)
const int IN3 = 9;
const int IN4 = 10;

const int FRONT_TRIG = 2;
const int FRONT_ECHO = 3;
const int REAR_TRIG = 4;
const int REAR_ECHO = 11;
const int FLAME_PIN = 12; // digital output; confirm sensor polarity
const int PIR_PIN = 13;

const float FRONT_STOP_CM = 25.0;
const float REAR_CLEAR_CM = 20.0;
const unsigned long WATCHDOG_MS = 700;
const unsigned long TELEMETRY_MS = 150;

String commandBuffer;
String currentCommand = "STOP";
unsigned long lastCommandMs = 0;
unsigned long lastTelemetryMs = 0;

float readDistanceCm(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 25000UL);
  if (duration == 0) return -1.0;
  return duration * 0.0343 / 2.0;
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  currentCommand = "STOP";
}

void setMotorPins(bool a1, bool a2, bool b1, bool b2, int speedA, int speedB) {
  digitalWrite(IN1, a1); digitalWrite(IN2, a2);
  digitalWrite(IN3, b1); digitalWrite(IN4, b2);
  analogWrite(ENA, speedA);
  analogWrite(ENB, speedB);
}

void applyCommand(String cmd, float frontCm, float rearCm) {
  cmd.trim();
  if (cmd == "STOP") {
    stopMotors();
  } else if (cmd == "FORWARD") {
    if (frontCm > 0 && frontCm > FRONT_STOP_CM) {
      setMotorPins(HIGH, LOW, HIGH, LOW, 120, 120);
      currentCommand = cmd;
    } else {
      stopMotors();
    }
  } else if (cmd == "REVERSE") {
    if (rearCm > REAR_CLEAR_CM) {
      setMotorPins(LOW, HIGH, LOW, HIGH, 100, 100);
      currentCommand = cmd;
    } else {
      stopMotors();
    }
  } else if (cmd == "TURN_LEFT" || cmd == "TURN_RIGHT") {
    // Side clearance is not measured. Turning is deliberately disabled.
    stopMotors();
  } else {
    stopMotors();
  }
}

void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(FRONT_TRIG, OUTPUT); pinMode(FRONT_ECHO, INPUT);
  pinMode(REAR_TRIG, OUTPUT); pinMode(REAR_ECHO, INPUT);
  pinMode(FLAME_PIN, INPUT);
  pinMode(PIR_PIN, INPUT);

  Serial.begin(115200);
  stopMotors();
  lastCommandMs = millis();
}

void loop() {
  float frontCm = readDistanceCm(FRONT_TRIG, FRONT_ECHO);
  float rearCm = readDistanceCm(REAR_TRIG, REAR_ECHO);

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      commandBuffer.trim();
      applyCommand(commandBuffer, frontCm, rearCm);
      lastCommandMs = millis();
      commandBuffer = "";
    } else if (c != '\r' && commandBuffer.length() < 40) {
      commandBuffer += c;
    } else if (commandBuffer.length() >= 40) {
      commandBuffer = "";
      stopMotors();
    }
  }

  if (millis() - lastCommandMs > WATCHDOG_MS) {
    stopMotors();
  }

  if (millis() - lastTelemetryMs >= TELEMETRY_MS) {
    int flame = digitalRead(FLAME_PIN);
    int pir = digitalRead(PIR_PIN);
    Serial.print("SENSORS,");
    Serial.print(frontCm, 1); Serial.print(",");
    Serial.print(rearCm, 1); Serial.print(",");
    Serial.print(flame); Serial.print(",");
    Serial.println(pir);
    lastTelemetryMs = millis();
  }
}
