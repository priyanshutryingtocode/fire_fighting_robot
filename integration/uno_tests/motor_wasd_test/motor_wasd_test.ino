/*
 * Standalone motor direction test for the FireBot example pin map.
 * Wheels must be raised. Each W/A/S/D command runs for 600 ms then stops.
 * Serial Monitor: 115200 baud, Newline. Send W/A/S/D (case-insensitive), X to stop.
 */

#define MOTOR_DRIVER 0  // 0 = L298N with EN pins; 1 = L293D board with enable jumpers

#if MOTOR_DRIVER == 1
const uint8_t PIN_IN1 = 5, PIN_IN2 = 6;    // left motor channel
const uint8_t PIN_IN3 = 3, PIN_IN4 = 11;   // right motor channel
#else
const uint8_t PIN_ENA = 5, PIN_IN1 = 7, PIN_IN2 = 8;
const uint8_t PIN_ENB = 6, PIN_IN3 = 11, PIN_IN4 = 12;
#endif

const bool INVERT_LEFT = false;
const bool INVERT_RIGHT = false;
const int TEST_SPEED_PERCENT = 35;
const unsigned long PULSE_MS = 600;

int leftCommand = 0;
int rightCommand = 0;
unsigned long commandStartedMs = 0;
char inputLine[12];
uint8_t inputLength = 0;

void stopMotors() {
#if MOTOR_DRIVER == 1
  analogWrite(PIN_IN1, 0); analogWrite(PIN_IN2, 0);
  analogWrite(PIN_IN3, 0); analogWrite(PIN_IN4, 0);
#else
  analogWrite(PIN_ENA, 0); analogWrite(PIN_ENB, 0);
  digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, LOW);
#endif
  leftCommand = rightCommand = 0;
}

void driveOne(uint8_t a, uint8_t b, int percent, bool invert) {
  if (invert) percent = -percent;
  percent = constrain(percent, -100, 100);
  if (percent == 0) {
    analogWrite(a, 0); analogWrite(b, 0);
    return;
  }
  int pwm = map(abs(percent), 1, 100, 110, 255);
  if (percent > 0) { analogWrite(b, 0); analogWrite(a, pwm); }
  else             { analogWrite(a, 0); analogWrite(b, pwm); }
}

#if MOTOR_DRIVER == 0
void driveL298(uint8_t en, uint8_t inA, uint8_t inB, int percent, bool invert) {
  if (invert) percent = -percent;
  percent = constrain(percent, -100, 100);
  digitalWrite(inA, percent > 0 ? HIGH : LOW);
  digitalWrite(inB, percent < 0 ? HIGH : LOW);
  analogWrite(en, percent == 0 ? 0 : map(abs(percent), 1, 100, 110, 255));
}
#endif

void applyMotors(int left, int right) {
  leftCommand = left;
  rightCommand = right;
#if MOTOR_DRIVER == 1
  driveOne(PIN_IN1, PIN_IN2, left, INVERT_LEFT);
  driveOne(PIN_IN3, PIN_IN4, right, INVERT_RIGHT);
#else
  driveL298(PIN_ENA, PIN_IN1, PIN_IN2, left, INVERT_LEFT);
  driveL298(PIN_ENB, PIN_IN3, PIN_IN4, right, INVERT_RIGHT);
#endif
  commandStartedMs = millis();
}

void handleCommand(char command) {
  if (command >= 'a' && command <= 'z') command -= ('a' - 'A');
  switch (command) {
    case 'W': applyMotors(TEST_SPEED_PERCENT, TEST_SPEED_PERCENT); Serial.println(F("forward pulse")); break;
    case 'S': applyMotors(-TEST_SPEED_PERCENT, -TEST_SPEED_PERCENT); Serial.println(F("reverse pulse")); break;
    case 'A': applyMotors(-TEST_SPEED_PERCENT, TEST_SPEED_PERCENT); Serial.println(F("left pivot pulse")); break;
    case 'D': applyMotors(TEST_SPEED_PERCENT, -TEST_SPEED_PERCENT); Serial.println(F("right pivot pulse")); break;
    case 'X': stopMotors(); Serial.println(F("stopped")); break;
    default: Serial.println(F("Use W/A/S/D for a short pulse, X to stop.")); break;
  }
}

void setup() {
#if MOTOR_DRIVER == 1
  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
#else
  pinMode(PIN_ENA, OUTPUT); pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
#endif
  stopMotors();
  Serial.begin(115200);
  Serial.println(F("Motor test ready. Wheels raised; W/A/S/D = 600 ms pulse, X = stop."));
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (inputLength) handleCommand(inputLine[0]);
      inputLength = 0;
    } else if (c != '\r' && inputLength < sizeof(inputLine) - 1) {
      inputLine[inputLength++] = c;
    } else if (inputLength >= sizeof(inputLine) - 1) {
      inputLength = 0;
    }
  }
  if ((leftCommand || rightCommand) && millis() - commandStartedMs >= PULSE_MS) {
    stopMotors();
    Serial.println(F("pulse complete; motors stopped"));
  }
}
