/*
 * Standalone single-servo arm/nozzle test on the FireBot servo pin.
 * Detach/clear the arm linkage before discovering safe travel limits.
 * Serial Monitor: 115200 baud, Newline. Send an angle (for example 80) or C.
 */

#include <Servo.h>

const uint8_t PIN_SERVO = 9;
const int SERVO_MIN_SAFE = 60;   // adjust only after checking the linkage
const int SERVO_MAX_SAFE = 120;  // adjust only after checking the linkage
const int SERVO_HOME = 90;

Servo armServo;
char inputLine[12];
uint8_t inputLength = 0;

void applyCommand() {
  inputLine[inputLength] = '\0';
  if (inputLength == 1 && (inputLine[0] == 'c' || inputLine[0] == 'C')) {
    armServo.write(SERVO_HOME);
    Serial.print(F("home=")); Serial.println(SERVO_HOME);
    return;
  }
  char* endPtr;
  long angle = strtol(inputLine, &endPtr, 10);
  if (endPtr == inputLine || *endPtr != '\0') {
    Serial.println(F("Send a numeric angle or C for home."));
    return;
  }
  angle = constrain(angle, SERVO_MIN_SAFE, SERVO_MAX_SAFE);
  armServo.write((int)angle);
  Serial.print(F("commanded angle=")); Serial.println(angle);
}

void setup() {
  Serial.begin(115200);
  armServo.attach(PIN_SERVO);
  armServo.write(SERVO_HOME);
  Serial.println(F("Servo test ready. Enter angle or C (home)."));
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (inputLength) applyCommand();
      inputLength = 0;
    } else if (c != '\r' && inputLength < sizeof(inputLine) - 1) {
      inputLine[inputLength++] = c;
    } else if (inputLength >= sizeof(inputLine) - 1) {
      inputLength = 0;
    }
  }
}
