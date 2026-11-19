/*
 * Standalone two-axis FireBot arm test.
 * Pan/horizontal servo: D9 (same as the pump test). Tilt/base servo: D4.
 * Remove the arm linkage or keep clear while identifying safe angle limits.
 * Serial Monitor: 115200 baud, Newline. Commands: P 90, T 90, C (both home).
 */

#include <Servo.h>

const uint8_t PIN_PAN_SERVO = 9;
const uint8_t PIN_TILT_SERVO = 4;
const int PAN_HOME = 90, TILT_HOME = 90;
const int PAN_MIN_SAFE = 55, PAN_MAX_SAFE = 125;
const int TILT_MIN_SAFE = 60, TILT_MAX_SAFE = 120;

Servo panServo, tiltServo;
char lineBuffer[16];
uint8_t lineLength = 0;

void applyCommand() {
  lineBuffer[lineLength] = '\0';
  if ((lineBuffer[0] == 'C' || lineBuffer[0] == 'c') && lineLength == 1) {
    panServo.write(PAN_HOME);
    tiltServo.write(TILT_HOME);
    Serial.println(F("both servos -> home"));
    return;
  }
  char axis = lineBuffer[0];
  if (axis >= 'a' && axis <= 'z') axis -= ('a' - 'A');
  char* endPtr;
  long requested = strtol(lineBuffer + 1, &endPtr, 10);
  if ((*endPtr != '\0') || (axis != 'P' && axis != 'T')) {
    Serial.println(F("Use P <angle>, T <angle>, or C."));
    return;
  }
  if (axis == 'P') {
    int angle = constrain(requested, PAN_MIN_SAFE, PAN_MAX_SAFE);
    panServo.write(angle);
    Serial.print(F("pan=")); Serial.println(angle);
  } else {
    int angle = constrain(requested, TILT_MIN_SAFE, TILT_MAX_SAFE);
    tiltServo.write(angle);
    Serial.print(F("tilt=")); Serial.println(angle);
  }
}

void setup() {
  Serial.begin(115200);
  panServo.attach(PIN_PAN_SERVO, 544, 2400);
  tiltServo.attach(PIN_TILT_SERVO, 544, 2400);
  panServo.write(PAN_HOME);
  tiltServo.write(TILT_HOME);
  Serial.println(F("Dual servo test ready: P angle, T angle, C=home."));
}

void loop() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n') {
      if (lineLength) applyCommand();
      lineLength = 0;
    } else if (ch != '\r' && lineLength < sizeof(lineBuffer) - 1) {
      lineBuffer[lineLength++] = ch;
    } else if (lineLength >= sizeof(lineBuffer) - 1) {
      lineLength = 0;
    }
  }
}
