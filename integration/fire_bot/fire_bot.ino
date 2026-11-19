/*
 * FireBot complete controller for the tested wiring plus the requested arm.
 * Vision and fire centering run on the laptop; the Uno controls motors, one
 * front ultrasonic sensor, two arm servos, flame sensor, relay/pump, and OLED.
 *
 * Laptop protocol at 115200 baud, newline terminated:
 *   F,<x_error>,<y_error>,<distance_cm>,<confidence>
 *   N                         no confirmed fire
 *   X / G                     latch / clear software emergency stop
 *   W/A/S/D, x, r             manual drive / stop / return to auto
 *
 * Existing test wiring retained:
 *   flame DO -> A0; relay IN -> D10 (active LOW); OLED SH1106 SDA=A4/SCL=A5;
 *   horizontal servo signal -> D9; LED -> D13.
 * Added for the requested hardware:
 *   front HC-SR04 TRIG -> D2, ECHO -> D3 with L298N (D7 with L293D map);
 *   vertical/base servo signal -> D4.
 *
 * Servo supplies must be external and share ground with Arduino. Keep the pump
 * supply electrically isolated as in the tested relay wiring. Verify every pin
 * and polarity against the actual robot before connecting motor/pump power.
 */

#define MOTOR_DRIVER 0       // 0=L298N with EN pins; 1=L293D module with EN jumpers
#define USE_OLED 1           // set 0 if the SH1106/U8g2 display is not installed
#define LINK_BLUETOOTH 0     // 0=USB Serial; 1=HC-05 on A1/A2 at 9600 baud

#include <Servo.h>
#include <avr/wdt.h>
#if USE_OLED
#include <Wire.h>
#include <U8g2lib.h>
U8G2_SH1106_128X64_NONAME_1_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);
#endif
#if LINK_BLUETOOTH
#include <SoftwareSerial.h>
SoftwareSerial btLink(A1, A2);  // Arduino RX=A1 (HC-05 TX), TX=A2 (through divider)
#define LINK btLink
#define LINK_BAUD 9600
#else
#define LINK Serial
#define LINK_BAUD 115200
#endif

// ------------------------------- board configuration
const uint8_t PIN_FLAME = A0;
const uint8_t PIN_RELAY = 10;
const uint8_t PIN_LED = 13;
const uint8_t PIN_PAN_SERVO = 9;    // existing test servo, sweeps left/right while spraying
const uint8_t PIN_TILT_SERVO = 4;   // added base servo, aims up/down from camera y error
const uint8_t PIN_TRIG_FRONT = 2;
#if MOTOR_DRIVER == 1
const uint8_t PIN_ECHO_FRONT = 7;
const uint8_t PIN_IN1 = 5, PIN_IN2 = 6;     // left channel
const uint8_t PIN_IN3 = 3, PIN_IN4 = 11;    // right channel
#else
const uint8_t PIN_ECHO_FRONT = 3;
const uint8_t PIN_ENA = 5, PIN_IN1 = 7, PIN_IN2 = 8;
const uint8_t PIN_ENB = 6, PIN_IN3 = 11, PIN_IN4 = 12;
#endif

// Match these with the tests that worked on this robot.
const bool RELAY_ACTIVE_LOW = true;
const bool FLAME_ACTIVE_LOW = true;
const bool INVERT_LEFT_MOTOR = false;
const bool INVERT_RIGHT_MOTOR = false;
const bool PAN_REVERSED = false;
const bool TILT_REVERSED = false;

// Servo safe travel limits. Adjust for arm geometry before enabling pump power.
const int PAN_CENTER = 90, PAN_MIN = 55, PAN_MAX = 125;
const int TILT_CENTER = 90, TILT_MIN = 60, TILT_MAX = 120;
const int PAN_AIM_RANGE = 30;       // camera horizontal error -> arm pan offset
const int TILT_AIM_RANGE = 25;      // camera vertical error -> arm tilt offset
const int PAN_SWEEP_DEG = 30;       // left/right nozzle sweep around aimed target
const unsigned long PAN_SWEEP_PERIOD_MS = 1200;

// Motion and sensor tuning (centimetres unless noted).
const int FRONT_OBSTACLE_CM = 25;
const int FRONT_CLEAR_CM = 45;
const int CREEP_STOP_CM = 12;
const int STOP_APPROACH_CM = 40;    // camera distance at which slow approach begins
const int SPRAY_MAX_CM = 70;
const int VIS_MIN_CONF = 40;
const int CENTER_ERROR = 15;        // normalized camera error: +/-15 is centered
const int SPEED_FORWARD = 65;
const int SPEED_SLOW = 42;
const int SPEED_CREEP = 32;
const int SPEED_TURN = 55;
const int MOTOR_MIN_PWM = 110;
const float STEER_KP = 0.55;

// Timing guards.
const unsigned long FLAME_DEBOUNCE_MS = 100;
const unsigned long FLAME_HOLD_MS = 1000;
const unsigned long MAX_PUMP_BURST_MS = 5000;
const unsigned long PUMP_REST_MS = 2000;
const unsigned long MAX_PUMP_TOTAL_MS = 60000;
const unsigned long ARM_SETTLE_MS = 700;
const unsigned long VERIFY_MS = 700;
const unsigned long VISION_TIMEOUT_MS = 800;
const unsigned long VISION_MEMORY_MS = 1500;
const unsigned long LOST_FIRE_MS = 2500;
const unsigned long SEARCH_REVOLUTION_MS = 4000;  // calibrate for one in-place 360-degree turn
const unsigned long SEARCH_PAUSE_MS = 450;
const unsigned long SONAR_PERIOD_MS = 50;

enum RobotState : uint8_t { SEARCH, TRACK, AVOID, APPROACH, AIM, SPRAY, VERIFY, SAFE_STOP, ESTOP, MANUAL };
const char* const STATE_NAMES[] = {
  "SEARCH", "TRACK", "AVOID", "APPROACH", "AIM", "SPRAY", "VERIFY", "SAFE_STOP", "ESTOP", "MANUAL"
};

struct VisionData {
  bool fire = false;
  int errX = 0, errY = 0, distanceCm = 999, confidence = 0;
  unsigned long lastPacketMs = 0, lastFireMs = 0;
} vision;

struct SensorData {
  int frontCm = 400;
  bool flameRaw = false, flame = false, flameGone = true;
} sensors;

Servo panServo, tiltServo;
RobotState state = SEARCH;
unsigned long stateStartedMs = 0, lastFlameMs = 0;
unsigned long pumpStartedMs = 0, pumpTotalMs = 0, pumpRestUntilMs = 0;
unsigned long pumpChangedMs = 0, sweepStartedMs = 0;
bool pumpOn = false, estopRequested = false, pumpLockedOut = false;
int panAimAngle = PAN_CENTER, tiltAimAngle = TILT_CENTER;
int8_t avoidDirection = 1;
uint8_t avoidPhase = 0, burstCount = 0;
int manualLeft = 0, manualRight = 0;
const int MANUAL_SPEED = 55;
char serialLine[40];
uint8_t serialLength = 0;

// ------------------------------- motor helpers
int clampPercent(int value) { return constrain(value, -100, 100); }

#if MOTOR_DRIVER == 1
void driveMotor(uint8_t inA, uint8_t inB, int percent, bool invert) {
  percent = clampPercent(invert ? -percent : percent);
  if (!percent) { analogWrite(inA, 0); analogWrite(inB, 0); return; }
  int pwm = map(abs(percent), 1, 100, MOTOR_MIN_PWM, 255);
  if (percent > 0) { analogWrite(inB, 0); analogWrite(inA, pwm); }
  else { analogWrite(inA, 0); analogWrite(inB, pwm); }
}
void setMotors(int left, int right) {
  driveMotor(PIN_IN1, PIN_IN2, left, INVERT_LEFT_MOTOR);
  driveMotor(PIN_IN3, PIN_IN4, right, INVERT_RIGHT_MOTOR);
}
#else
void driveMotor(uint8_t en, uint8_t inA, uint8_t inB, int percent, bool invert) {
  percent = clampPercent(invert ? -percent : percent);
  if (!percent) {
    analogWrite(en, 0); digitalWrite(inA, LOW); digitalWrite(inB, LOW);
    return;
  }
  digitalWrite(inA, percent > 0 ? HIGH : LOW);
  digitalWrite(inB, percent < 0 ? HIGH : LOW);
  analogWrite(en, map(abs(percent), 1, 100, MOTOR_MIN_PWM, 255));
}
void setMotors(int left, int right) {
  driveMotor(PIN_ENA, PIN_IN1, PIN_IN2, left, INVERT_LEFT_MOTOR);
  driveMotor(PIN_ENB, PIN_IN3, PIN_IN4, right, INVERT_RIGHT_MOTOR);
}
#endif

void stopMotors() { setMotors(0, 0); }

void setPump(bool on) {
  if (on == pumpOn) return;
  unsigned long now = millis();
  if (on) {
    if (pumpLockedOut || pumpTotalMs >= MAX_PUMP_TOTAL_MS) return;
    pumpStartedMs = now;
    burstCount++;
    sweepStartedMs = now;
  } else {
    pumpTotalMs += now - pumpStartedMs;
  }
  pumpOn = on;
  pumpChangedMs = now;
  digitalWrite(PIN_RELAY, (on ^ RELAY_ACTIVE_LOW) ? HIGH : LOW);
  digitalWrite(PIN_LED, on ? HIGH : LOW);
  LINK.println(on ? F("PUMP ON") : F("PUMP OFF"));
}

void setArmAngles(int panAngle, int tiltAngle) {
  panAngle = constrain(panAngle, PAN_MIN, PAN_MAX);
  tiltAngle = constrain(tiltAngle, TILT_MIN, TILT_MAX);
  if (PAN_REVERSED) panAngle = PAN_MIN + PAN_MAX - panAngle;
  if (TILT_REVERSED) tiltAngle = TILT_MIN + TILT_MAX - tiltAngle;
  panAimAngle = panAngle;
  tiltAimAngle = tiltAngle;
  panServo.write(panAngle);
  tiltServo.write(tiltAngle);
}

bool visionOnline() { return vision.lastPacketMs && millis() - vision.lastPacketMs < VISION_TIMEOUT_MS; }
bool fireNow() { return visionOnline() && vision.fire && vision.confidence >= VIS_MIN_CONF; }
bool fireRecent() { return vision.lastFireMs && millis() - vision.lastFireMs < VISION_MEMORY_MS; }
bool distanceKnown() { return vision.distanceCm > 0 && vision.distanceCm < 999; }

bool sprayAllowed() {
  if (!sensors.flame || pumpLockedOut || millis() < pumpRestUntilMs) return false;
  if (!visionOnline()) return true;  // flame-sensor fallback when laptop is offline
  if (!fireRecent()) return false;
  bool cameraNear = distanceKnown() && vision.distanceCm <= SPRAY_MAX_CM;
  bool sonarNear = sensors.frontCm > 0 && sensors.frontCm <= SPRAY_MAX_CM;
  return cameraNear || sonarNear || !distanceKnown();
}

void enterState(RobotState next) {
  if (state == next) return;
  state = next;
  stateStartedMs = millis();
  stopMotors();
  if (next != SPRAY) setPump(false);
  if (next == SEARCH) { burstCount = 0; setArmAngles(PAN_CENTER, TILT_CENTER); }
  if (next == AVOID) {
    avoidPhase = 0;
    avoidDirection = -avoidDirection;  // try the opposite direction on each avoidance
  }
}

// ------------------------------- serial protocol
void handleLine(char* line) {
  unsigned long now = millis();
  char command = line[0];
  if (command == 'F' || command == 'N') vision.lastPacketMs = now;
  int x, y, distance, confidence;
  switch (command) {
    case 'F':
      if (sscanf(line + 1, ",%d,%d,%d,%d", &x, &y, &distance, &confidence) == 4) {
        vision.fire = true;
        vision.errX = constrain(x, -100, 100);
        vision.errY = constrain(y, -100, 100);
        vision.distanceCm = constrain(distance, 0, 999);
        vision.confidence = constrain(confidence, 0, 100);
        if (vision.confidence >= VIS_MIN_CONF) vision.lastFireMs = now;
      }
      break;
    case 'N': vision.fire = false; break;
    case 'X': estopRequested = true; break;
    case 'G': estopRequested = false; break;
    case 'w': case 'W': manualLeft = MANUAL_SPEED; manualRight = MANUAL_SPEED; enterState(MANUAL); break;
    case 's': case 'S': manualLeft = -MANUAL_SPEED; manualRight = -MANUAL_SPEED; enterState(MANUAL); break;
    case 'a': case 'A': manualLeft = -MANUAL_SPEED; manualRight = MANUAL_SPEED; enterState(MANUAL); break;
    case 'd': case 'D': manualLeft = MANUAL_SPEED; manualRight = -MANUAL_SPEED; enterState(MANUAL); break;
    case 'x': manualLeft = manualRight = 0; enterState(MANUAL); break;
    case 'r': case 'R': enterState(SEARCH); break;
  }
}

void readSerialLines() {
  while (LINK.available()) {
    char ch = (char)LINK.read();
    if (ch == '\n') {
      serialLine[serialLength] = '\0';
      if (serialLength) handleLine(serialLine);
      serialLength = 0;
    } else if (ch != '\r' && serialLength < sizeof(serialLine) - 1) {
      serialLine[serialLength++] = ch;
    } else if (serialLength >= sizeof(serialLine) - 1) {
      serialLength = 0;
    }
  }
}

// ------------------------------- sensors
int readFrontCm() {
  digitalWrite(PIN_TRIG_FRONT, LOW);
  delayMicroseconds(3);
  digitalWrite(PIN_TRIG_FRONT, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG_FRONT, LOW);
  unsigned long duration = pulseIn(PIN_ECHO_FRONT, HIGH, 25000UL);
  return duration == 0 ? 400 : (int)(duration / 58UL);
}

void updateFrontSonar(unsigned long now) {
  static unsigned long lastRead = 0;
  if (now - lastRead < SONAR_PERIOD_MS) return;
  lastRead = now;
  sensors.frontCm = readFrontCm();
}

void updateFlame(unsigned long now) {
  static bool previousRaw = false, firstRead = true;
  static unsigned long flameOnSince = 0, flameOffSince = 0;
  bool raw = digitalRead(PIN_FLAME) == (FLAME_ACTIVE_LOW ? LOW : HIGH);
  if (firstRead || raw != previousRaw) {
    if (raw) flameOnSince = now;
    else flameOffSince = now;
    previousRaw = raw;
    firstRead = false;
  }
  sensors.flameRaw = raw;
  sensors.flame = raw && now - flameOnSince >= FLAME_DEBOUNCE_MS;
  sensors.flameGone = !raw && now - flameOffSince >= FLAME_HOLD_MS;
  if (sensors.flame) lastFlameMs = now;
}

// ------------------------------- mission states
void doSearch(unsigned long now) {
  if (sprayAllowed()) { enterState(AIM); return; }
  if (fireNow()) { enterState(TRACK); return; }
  if (sensors.frontCm <= FRONT_OBSTACLE_CM) { enterState(AVOID); return; }
  unsigned long phase = (now - stateStartedMs) % (SEARCH_REVOLUTION_MS + SEARCH_PAUSE_MS);
  if (phase < SEARCH_REVOLUTION_MS) setMotors(SPEED_TURN, -SPEED_TURN);
  else stopMotors();
}

void steerTowardFire(int baseSpeed) {
  float direction = vision.errX / 100.0f;
  if (abs(vision.errX) > 60) {
    if (direction > 0) setMotors(SPEED_TURN, -SPEED_TURN);
    else setMotors(-SPEED_TURN, SPEED_TURN);
    return;
  }
  int turn = (int)(STEER_KP * baseSpeed * direction);
  setMotors(baseSpeed + turn, baseSpeed - turn);
}

void doTrack() {
  if (sprayAllowed()) { enterState(AIM); return; }
  if (sensors.frontCm <= FRONT_OBSTACLE_CM) {
    bool fireIsAhead = fireNow() && abs(vision.errX) < 25
        && (!distanceKnown() || vision.distanceCm <= sensors.frontCm + 30);
    enterState(fireIsAhead ? APPROACH : AVOID);
    return;
  }
  if (!fireNow()) {
    if (!fireRecent()) enterState(SEARCH);
    else setMotors(SPEED_SLOW, SPEED_SLOW);
    return;
  }
  if (distanceKnown() && vision.distanceCm <= STOP_APPROACH_CM) {
    enterState(APPROACH);
    return;
  }
  int speed = distanceKnown() && vision.distanceCm < 100 ? SPEED_SLOW : SPEED_FORWARD;
  steerTowardFire(speed);
}

void doAvoid(unsigned long now) {
  unsigned long elapsed = now - stateStartedMs;
  if (avoidPhase == 0) {
    stopMotors();
    if (elapsed >= 150) { avoidPhase = 1; stateStartedMs = now; }
    return;
  }
  if (avoidPhase == 1) {
    // Only one forward sonar is fitted: pivot a short amount, then recheck.
    setMotors(avoidDirection * SPEED_TURN, -avoidDirection * SPEED_TURN);
    if (elapsed >= 350) {
      stopMotors();
      if (sensors.frontCm > FRONT_CLEAR_CM) {
        avoidPhase = 2;
        stateStartedMs = now;
      } else {
        avoidDirection = -avoidDirection;
        stateStartedMs = now;
      }
    }
    return;
  }
  if (sensors.frontCm <= FRONT_OBSTACLE_CM) {
    enterState(AVOID);
    return;
  }
  setMotors(SPEED_SLOW, SPEED_SLOW);
  if (elapsed >= 650) enterState(fireRecent() ? TRACK : SEARCH);
}

void doApproach(unsigned long now) {
  if (sprayAllowed()) { enterState(AIM); return; }
  if (!fireRecent()) {
    stopMotors();
    if (now - stateStartedMs > LOST_FIRE_MS) enterState(SEARCH);
    return;
  }
  if (fireNow() && distanceKnown() && vision.distanceCm > STOP_APPROACH_CM + 25) {
    enterState(TRACK);
    return;
  }
  if (now - stateStartedMs > 6000) { enterState(SEARCH); return; }
  if (sensors.frontCm <= FRONT_OBSTACLE_CM) {
    bool targetInFront = fireNow() && abs(vision.errX) <= CENTER_ERROR
        && (!distanceKnown() || vision.distanceCm <= sensors.frontCm + 30);
    if (!targetInFront) { enterState(AVOID); return; }
  }
  if (fireNow() && abs(vision.errX) > CENTER_ERROR) {
    if ((now / 120) % 2) {
      if (vision.errX > 0) setMotors(SPEED_TURN / 2, -SPEED_TURN / 2);
      else setMotors(-SPEED_TURN / 2, SPEED_TURN / 2);
    } else stopMotors();
    return;
  }
  if (sensors.frontCm > CREEP_STOP_CM && (!distanceKnown() || vision.distanceCm > 15))
    setMotors(SPEED_CREEP, SPEED_CREEP);
  else stopMotors();
}

void doAim() {
  stopMotors();
  setPump(false);
  if (!sprayAllowed()) {
    if (fireRecent()) enterState(APPROACH);
    else enterState(SEARCH);
    return;
  }
  int xError = visionOnline() ? vision.errX : 0;
  int yError = visionOnline() ? vision.errY : 0;
  int panOffset = map(constrain(xError, -100, 100), -100, 100, -PAN_AIM_RANGE, PAN_AIM_RANGE);
  int tiltOffset = map(constrain(yError, -100, 100), -100, 100, -TILT_AIM_RANGE, TILT_AIM_RANGE);
  setArmAngles(PAN_CENTER + panOffset, TILT_CENTER + tiltOffset);
  if (millis() - stateStartedMs >= ARM_SETTLE_MS) enterState(SPRAY);
}

void updatePanSweep(unsigned long now) {
  unsigned long half = PAN_SWEEP_PERIOD_MS / 2;
  unsigned long phase = (now - sweepStartedMs + PAN_SWEEP_PERIOD_MS / 4) % PAN_SWEEP_PERIOD_MS;
  long triangle = (phase < half) ? (long)phase : (long)(PAN_SWEEP_PERIOD_MS - phase);
  int offset = map(triangle, 0, half, -PAN_SWEEP_DEG, PAN_SWEEP_DEG);
  int angle = constrain(panAimAngle + (PAN_REVERSED ? -offset : offset), PAN_MIN, PAN_MAX);
  panServo.write(angle);
  tiltServo.write(tiltAimAngle);
}

void doSpray(unsigned long now) {
  stopMotors();
  if (pumpLockedOut) { setPump(false); enterState(SAFE_STOP); return; }
  if (!pumpOn) setPump(true);
  if (!pumpOn) { pumpLockedOut = true; enterState(SAFE_STOP); return; }
  updatePanSweep(now);
  if (now - pumpStartedMs >= MAX_PUMP_BURST_MS) {
    setPump(false);
    pumpRestUntilMs = now + PUMP_REST_MS;
    if (pumpTotalMs >= MAX_PUMP_TOTAL_MS) pumpLockedOut = true;
    enterState(VERIFY);
    return;
  }
  if (!sensors.flame && now - lastFlameMs >= FLAME_HOLD_MS) {
    setPump(false);
    enterState(VERIFY);
  }
}

void doVerify(unsigned long now) {
  stopMotors();
  if (now - stateStartedMs < VERIFY_MS || now < pumpRestUntilMs) return;
  if (sensors.flameGone) {
    burstCount = 0;
    enterState(SEARCH);
    return;
  }
  if (pumpLockedOut || burstCount >= 6) { enterState(SAFE_STOP); return; }
  if (sprayAllowed()) { enterState(AIM); return; }
  if (fireNow()) enterState(distanceKnown() && vision.distanceCm <= STOP_APPROACH_CM ? APPROACH : TRACK);
  else enterState(SEARCH);
}

void doManual() {
  if (sprayAllowed()) { enterState(AIM); return; }
  setMotors(manualLeft, manualRight);
}

// ------------------------------- display and reporting
#if USE_OLED
void centeredText(int y, const char* text) {
  oled.drawStr((128 - oled.getStrWidth(text)) / 2, y, text);
}
void drawOled() {
  char sensorLine[24], pumpLine[24];
  snprintf(sensorLine, sizeof(sensorLine), "Front:%d Flame:%s", sensors.frontCm,
           sensors.flame ? "YES" : "no");
  snprintf(pumpLine, sizeof(pumpLine), "Pump:%s Fire:%u", pumpOn ? "ON" : "off", burstCount);
  oled.firstPage();
  do {
    oled.setFont(u8g2_font_6x10_tr);
    oled.drawStr(0, 10, "FIREBOT");
    oled.drawStr(128 - oled.getStrWidth(STATE_NAMES[state]), 10, STATE_NAMES[state]);
    oled.drawHLine(0, 13, 128);
    oled.setFont(u8g2_font_helvB10_tr);
    centeredText(35, pumpOn ? "SPRAYING" : (state == AIM ? "AIMING" : STATE_NAMES[state]));
    oled.setFont(u8g2_font_6x10_tr);
    oled.drawStr(0, 52, sensorLine);
    oled.drawStr(0, 63, pumpLine);
  } while (oled.nextPage());
}
#endif

void sendTelemetry(unsigned long now) {
  static unsigned long lastSent = 0;
  if (now - lastSent < 200) return;
  lastSent = now;
  LINK.print(F("S,")); LINK.print(STATE_NAMES[state]);
  LINK.print(','); LINK.print(sensors.frontCm);
  LINK.print(','); LINK.print(sensors.flame ? 1 : 0);
  LINK.print(','); LINK.print(!visionOnline() ? '-' : fireNow() ? 'F' : 'N');
  LINK.print(','); LINK.print(vision.distanceCm);
  LINK.print(','); LINK.println(pumpOn ? 1 : 0);
}

void setup() {
  // De-energize relay before configuring other outputs.
  digitalWrite(PIN_RELAY, RELAY_ACTIVE_LOW ? HIGH : LOW);
  pinMode(PIN_RELAY, OUTPUT);
  pinMode(PIN_FLAME, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  pinMode(PIN_TRIG_FRONT, OUTPUT);
  pinMode(PIN_ECHO_FRONT, INPUT);
  digitalWrite(PIN_TRIG_FRONT, LOW);
#if MOTOR_DRIVER == 0
  pinMode(PIN_ENA, OUTPUT); pinMode(PIN_ENB, OUTPUT);
#endif
  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
  stopMotors();

  panServo.attach(PIN_PAN_SERVO, 544, 2400);
  tiltServo.attach(PIN_TILT_SERVO, 544, 2400);
  setArmAngles(PAN_CENTER, TILT_CENTER);

  LINK.begin(LINK_BAUD);
  LINK.println(F("\nFireBot complete controller ready"));
  LINK.println(F("WASD=manual x=stop r=auto X=E-stop"));
#if USE_OLED
  oled.begin();
  oled.setBusClock(100000);
  Wire.setWireTimeout(3000, true);
#endif
  delay(500);
  stateStartedMs = millis();
  wdt_enable(WDTO_2S);
}

void loop() {
  wdt_reset();
  unsigned long now = millis();
  readSerialLines();
  updateFrontSonar(now);
  updateFlame(now);

  if (estopRequested && state != ESTOP) enterState(ESTOP);
  switch (state) {
    case SEARCH:    doSearch(now); break;
    case TRACK:     doTrack(); break;
    case AVOID:     doAvoid(now); break;
    case APPROACH:  doApproach(now); break;
    case AIM:       doAim(); break;
    case SPRAY:     doSpray(now); break;
    case VERIFY:    doVerify(now); break;
    case SAFE_STOP:
      stopMotors(); setPump(false);
      if (!sensors.flameGone && pumpLockedOut) break;
      if (sensors.flameGone && !pumpLockedOut) enterState(SEARCH);
      break;
    case ESTOP:
      stopMotors(); setPump(false);
      if (!estopRequested) enterState(SEARCH);
      break;
    case MANUAL: doManual(); break;
  }

  // Pump may only run in the SPRAY state. This also protects state transitions.
  if (state != SPRAY) setPump(false);
#if USE_OLED
  static unsigned long lastDraw = 0, lastReinit = 0;
  if (millis() - pumpChangedMs >= 150 && pumpChangedMs > lastReinit) {
    oled.initDisplay(); oled.setPowerSave(0); lastReinit = millis();
  }
  if (now - lastDraw >= 150) { lastDraw = now; drawOled(); }
#endif
  sendTelemetry(now);
}
