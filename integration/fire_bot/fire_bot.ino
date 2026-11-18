/*
 * ============================================================================
 *  FIRE FIGHTER ROBOT CAR  —  Arduino Uno firmware  (v2)
 *  Hardware: 3x HC-SR04 (left / centre / right), 1x flame sensor,
 *            1-channel relay -> water pump, L298N + 4 DC motors,
 *            optional servo for the nozzle.
 * ============================================================================
 *
 *  WHO DOES WHAT
 *    Laptop (fire_vision.py) : webcam + YOLO -> fire direction + DISTANCE (cm)
 *                              + optional calibrated camera distance. Sends over serial.
 *    Arduino (this file)     : drives motors, avoids obstacles with ultrasonics,
 *                              stops near the fire, and switches the relay ON
 *                              only when the FLAME SENSOR detects the flame.
 *
 *  SERIAL PROTOCOL (one text line per message, ends with '\n')
 *    Laptop -> Arduino
 *      F,<err>,<dist>,<conf>  fire seen
 *                               err  : -100 (far left) .. 0 (centre) .. +100 (far right)
 *                               dist : estimated distance to fire in cm (999 = unknown)
 *                               conf : YOLO confidence 0..100
 *      N                      no fire in this frame
 *      X / G                  emergency stop / go again
 *    Arduino -> Laptop (every 200 ms)
 *      S,<state>,<dF>,<dL>,<dR>,<flame 0/1>,<vision F/N/->,<dist>,<pump 0/1>
 *
 *  MISSION (state machine)
 *    SEARCH     spin in small steps until the camera sees fire
 *    TRACK      drive toward fire using camera direction,
 *               slow down as distance drops, avoid obstacles with ultrasonics
 *    AVOID      stop, pivot toward the freer measured side, pass, resume TRACK
 *    APPROACH   fire is close (camera dist <= STOP_DIST_CM): centre on it and
 *               creep forward until the flame sensor fires
 *    AIM        stop, point nozzle/arm toward the fire, then settle
 *    EXTINGUISH relay ON -> pump sprays (servo sweeps)
 *               until flame sensor OFF for 1 s (or 6 s max burst)
 *    VERIFY     pump off, wait, check again -> spray again or search for next fire
 *    ESTOP      everything off until 'G'
 *
 *  If the laptop is not connected, SEARCH still rotates. A flame sensor event
 *  stops the motors; spraying also requires USE_SERVO and its aim interlock.
 * ============================================================================
 */

// ----------------------------------------------------------------------------
//  1. BUILD OPTIONS
// ----------------------------------------------------------------------------
#define CALIBRATION_MODE 0   // 1 = print sensor values only, motors + pump OFF
#define USE_SERVO        0   // 1 only after an arm/nozzle servo is wired and calibrated (pin 9)
#define USE_ULTRASONIC   0   // 1 when the 3 ultrasonic sensors are connected
#define USE_OLED         1   // SH1106 128x64 I2C OLED on A4 (SDA) / A5 (SCL), needs U8g2 library
#define LINK_BLUETOOTH   0   // 1 = laptop talks via HC-05 on A1/A2 instead of USB
#define MOTOR_DRIVER     0   // 1 = L293D board with enable jumpers ON (PWM on the IN pins)
                             // 0 = driver with separate EN pins (L298N, ENA/ENB jumpers removed)

#if USE_SERVO
#include <Servo.h>
#endif

#if USE_OLED
#include <Wire.h>
#include <U8g2lib.h>
U8G2_SH1106_128X64_NONAME_1_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);
#endif

#if LINK_BLUETOOTH && USE_ULTRASONIC
#error "Bluetooth (A1/A2) and ultrasonics (A1/A2/A3) share pins - pick one"
#endif

#if LINK_BLUETOOTH
#include <SoftwareSerial.h>
SoftwareSerial btLink(A1, A2);     // Arduino RX = A1 (to HC-05 TXD), TX = A2 (to HC-05 RXD via divider)
#define LINK      btLink
#define LINK_BAUD 9600             // HC-05 default
#else
#define LINK      Serial
#define LINK_BAUD 115200
#endif

// ----------------------------------------------------------------------------
//  2. PINS  (Servo uses Timer1 -> no PWM on 9/10, motors use 5/6)
// ----------------------------------------------------------------------------
#if MOTOR_DRIVER == 1
// L293D board: EN jumpers stay ON, speed = PWM on the input pins (all 4 must be PWM pins)
const uint8_t PIN_IN1 = 5,  PIN_IN2 = 6;    // -> M1-IN terminal (left motors)
const uint8_t PIN_IN3 = 3,  PIN_IN4 = 11;   // -> M2-IN terminal (right motors)
#else
const uint8_t PIN_ENA = 5,  PIN_IN1 = 7,  PIN_IN2 = 8;    // left motors
const uint8_t PIN_ENB = 6,  PIN_IN3 = 11, PIN_IN4 = 12;   // right motors
#endif
const uint8_t PIN_SERVO = 9;
const uint8_t PIN_RELAY = 10;     // single-channel relay IN
const uint8_t PIN_LED   = 13;
const uint8_t PIN_FLAME = A0;     // flame sensor DO (or AO, see FLAME_USE_ANALOG)
#if MOTOR_DRIVER == 1
const uint8_t TRIG_F = 2,  ECHO_F = 7;    // centre ultrasonic (pin 3 is used by motors)
#else
const uint8_t TRIG_F = 2,  ECHO_F = 3;    // centre ultrasonic
#endif
const uint8_t TRIG_L = A2, ECHO_L = A3;   // left ultrasonic
const uint8_t TRIG_R = A1, ECHO_R = 4;    // right ultrasonic
// A4 = OLED SDA, A5 = OLED SCL

// ----------------------------------------------------------------------------
//  3. HARDWARE OPTIONS
// ----------------------------------------------------------------------------
const bool RELAY_ACTIVE_LOW   = false;  // use the SAME value that worked in the relay test sketch
const bool FLAME_USE_ANALOG   = false;  // false = DO pin (module pot sets range), true = AO pin
const bool FLAME_DO_ACTIVE_LOW = true;  // most modules: DO goes LOW when flame seen
const int  FLAME_AO_THRESHOLD = 500;    // AO only: intensity (0..1023) above = flame
const bool FLAME_AO_LOW_IS_FIRE = true; // AO only: most modules read lower near flame
const bool INVERT_LEFT_MOTOR  = false;
const bool INVERT_RIGHT_MOTOR = false;
const bool INVERT_SERVO       = false;

// ----------------------------------------------------------------------------
//  4. TUNING
// ----------------------------------------------------------------------------
// distances from the CAMERA (cm)
const int SLOW_DIST_CM   = 100;  // closer than this: slow down
const int STOP_DIST_CM   = 40;   // closer than this: stop driving, go to APPROACH
const int SPRAY_MAX_CM   = 70;   // flame sensor only allowed to trigger pump if fire this close
// ultrasonics (cm)
const int OBSTACLE_CM    = 25;   // obstacle in front (reactive mode)
const int CLEAR_CM       = 45;
const int SIDE_MIN_CM    = 12;
const int CREEP_MIN_CM   = 12;   // never creep closer than this to anything

const int VIS_MIN_CONF   = 40;   // ignore YOLO boxes below this confidence
const int CENTRE_ERR     = 15;   // |err| below this = fire centred

const int SPEED_FWD   = 70;      // % power
const int SPEED_SLOW  = 45;
const int SPEED_CREEP = 35;
const int SPEED_TURN  = 60;
const int SPEED_ALIGN = 45;
const int MOTOR_MIN_PWM = 110;   // raise if motors hum but don't move (L293D loses ~2 V)
const float STEER_KP  = 0.55;

const uint8_t SERVO_CENTER = 90, SERVO_RANGE = 35, SWEEP_DEG = 15;
const uint16_t SWEEP_PERIOD_MS = 1200;

const unsigned long FLAME_DEBOUNCE_MS = 150;   // flame must be seen this long = real
const unsigned long FLAME_CLEAR_MS    = 1000;  // flame gone this long = fire out
const unsigned long SPRAY_MAX_MS      = 6000;  // one burst max
const unsigned long VERIFY_MS         = 1500;
const uint8_t       MAX_BURSTS        = 6;
const unsigned long PUMP_TOTAL_MAX_MS = 60000; // tank-empty guard
const unsigned long CREEP_MAX_MS      = 5000;  // APPROACH give-up time
const unsigned long ARM_AIM_MS        = 700;   // allow arm/nozzle to reach target before pumping
const unsigned long VISION_TIMEOUT_MS = 800;   // YOLO at 640px on CPU can be ~5 fps
const unsigned long VISION_MEMORY_MS  = 1500;
const unsigned long LOST_MS           = 2500;
const unsigned long SEARCH_REVOLUTION_MS = 4000; // calibrate: measured time for one full in-place turn
const unsigned long SEARCH_PAUSE_MS   = 450;

// ----------------------------------------------------------------------------
//  TYPES & GLOBALS
// ----------------------------------------------------------------------------
enum State : uint8_t { SEARCH, TRACK, AVOID, APPROACH, AIM, EXTINGUISH, VERIFY, ESTOP, MANUAL };
const char* const STATE_NAME[] = {"SEARCH", "TRACK", "AVOID", "APPROACH", "AIM", "EXTINGUISH", "VERIFY", "ESTOP", "MANUAL"};

struct Vision {
  bool fire = false;
  int err = 0, dist = 999, conf = 0;
  unsigned long lastPacketMs = 0, lastFireMs = 0;
} vis;

struct Sensors {
  int dF = 400, dL = 400, dR = 400;
  bool flameRaw = false;       // instantaneous reading
  bool flame = false;          // debounced: flame really detected
  bool flameGone = true;       // no flame for FLAME_CLEAR_MS
  int  flameAnalog = 0;
} sen;

#if USE_SERVO
Servo nozzle;
#endif
State state = SEARCH;
unsigned long stateStartMs = 0, lastEvidenceMs = 0, pumpOnSinceMs = 0, pumpTotalMs = 0;
bool pumpOn = false, estopRequested = false;
int manualL = 0, manualR = 0;          // MANUAL mode wheel commands (%)
const int MANUAL_SPEED = 60;
void enter(State s);
uint8_t burstCount = 0, firesPutOut = 0;
int8_t avoidDir = 1;
uint8_t avoidPhase = 0;
unsigned long avoidPhaseMs = 0;

bool visionOnline();
bool fireNow();

// ----------------------------------------------------------------------------
//  ACTUATORS
// ----------------------------------------------------------------------------
#if MOTOR_DRIVER == 1
// PWM on one input, the other input LOW. Direction = which input gets the PWM.
void driveOne(uint8_t a, uint8_t b, int pct, bool invert) {
  pct = constrain(pct, -100, 100);
  if (invert) pct = -pct;
  if (pct == 0) { analogWrite(a, 0); analogWrite(b, 0); return; }
  int pwm = map(abs(pct), 1, 100, MOTOR_MIN_PWM, 255);
  if (pct > 0) { analogWrite(b, 0); analogWrite(a, pwm); }
  else         { analogWrite(a, 0); analogWrite(b, pwm); }
}

void setMotors(int left, int right) {
#if CALIBRATION_MODE
  left = right = 0;
#endif
  driveOne(PIN_IN1, PIN_IN2, left, INVERT_LEFT_MOTOR);
  driveOne(PIN_IN3, PIN_IN4, right, INVERT_RIGHT_MOTOR);
}
#else
void driveOne(uint8_t en, uint8_t a, uint8_t b, int pct, bool invert) {
  pct = constrain(pct, -100, 100);
  if (invert) pct = -pct;
  if (pct == 0) { analogWrite(en, 0); digitalWrite(a, LOW); digitalWrite(b, LOW); return; }
  digitalWrite(a, pct > 0 ? HIGH : LOW);
  digitalWrite(b, pct > 0 ? LOW : HIGH);
  analogWrite(en, map(abs(pct), 1, 100, MOTOR_MIN_PWM, 255));
}

void setMotors(int left, int right) {
#if CALIBRATION_MODE
  left = right = 0;
#endif
  driveOne(PIN_ENA, PIN_IN1, PIN_IN2, left, INVERT_LEFT_MOTOR);
  driveOne(PIN_ENB, PIN_IN3, PIN_IN4, right, INVERT_RIGHT_MOTOR);
}
#endif
void stopMotors() { setMotors(0, 0); }

// The ONLY place the relay is switched.
void setPump(bool on) {
#if CALIBRATION_MODE
  on = false;
#endif
  if (on && pumpTotalMs >= PUMP_TOTAL_MAX_MS) on = false;
  if (on == pumpOn) return;
  unsigned long now = millis();
  if (on) pumpOnSinceMs = now; else pumpTotalMs += now - pumpOnSinceMs;
  pumpOn = on;
  digitalWrite(PIN_RELAY, (on ^ RELAY_ACTIVE_LOW) ? HIGH : LOW);
}

void aimNozzle(int offsetDeg) {
#if USE_SERVO
  if (INVERT_SERVO) offsetDeg = -offsetDeg;
  nozzle.write(constrain(SERVO_CENTER + offsetDeg, 0, 180));
#else
  (void)offsetDeg;
#endif
}

// ----------------------------------------------------------------------------
//  SENSORS
// ----------------------------------------------------------------------------
int pingCm(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long us = pulseIn(echo, HIGH, 25000UL);
  return us == 0 ? 400 : (int)(us / 58UL);
}

int median3(int a, int b, int c) {
  if (a > b) { int t = a; a = b; b = t; }
  if (b > c) { int t = b; b = c; c = t; }
  if (a > b) { int t = a; a = b; b = t; }
  return b;
}

// one sensor every 25 ms, order F L F R (centre read twice as often), median of 3
void updateSonars() {
#if !USE_ULTRASONIC
  return;              // not connected yet: distances stay 400 (= nothing in front)
#endif
  static unsigned long lastMs = 0;
  static uint8_t seq = 0;
  static int hist[3][3] = {{400,400,400},{400,400,400},{400,400,400}};
  static uint8_t idx[3] = {0, 0, 0};
  const uint8_t order[4] = {0, 1, 0, 2};
  if (millis() - lastMs < 25) return;
  lastMs = millis();
  uint8_t w = order[seq];
  seq = (seq + 1) & 3;
  int d = (w == 0) ? pingCm(TRIG_F, ECHO_F) : (w == 1) ? pingCm(TRIG_L, ECHO_L) : pingCm(TRIG_R, ECHO_R);
  hist[w][idx[w]] = d;
  idx[w] = (idx[w] + 1) % 3;
  int m = median3(hist[w][0], hist[w][1], hist[w][2]);
  if (w == 0) sen.dF = m; else if (w == 1) sen.dL = m; else sen.dR = m;
}

void updateFlame() {
  static unsigned long onSince = 0, offSince = 0;
  static bool wasOn = false, first = true;
  bool raw;
  if (FLAME_USE_ANALOG) {
    int a = analogRead(PIN_FLAME);
    sen.flameAnalog = FLAME_AO_LOW_IS_FIRE ? 1023 - a : a;
    raw = sen.flameAnalog > FLAME_AO_THRESHOLD;
  } else {
    raw = (digitalRead(PIN_FLAME) == (FLAME_DO_ACTIVE_LOW ? LOW : HIGH));
  }
  unsigned long now = millis();
  if (first || raw != wasOn) { if (raw) onSince = now; else offSince = now; wasOn = raw; first = false; }
  sen.flameRaw  = raw;
  sen.flame     = raw && now - onSince >= FLAME_DEBOUNCE_MS;
  sen.flameGone = !raw && now - offSince >= FLAME_CLEAR_MS;
}

// ----------------------------------------------------------------------------
//  COMMUNICATION
// ----------------------------------------------------------------------------
void handleLine(char* s) {
  unsigned long now = millis();
  if (s[0] == 'F' || s[0] == 'N') vis.lastPacketMs = now;   // camera alive
  int a, b, c;
  switch (s[0]) {
    case 'F':
      if (sscanf(s + 1, ",%d,%d,%d", &a, &b, &c) == 3) {
        vis.fire = true;
        vis.err  = constrain(a, -100, 100);
        vis.dist = constrain(b, 0, 999);
        vis.conf = constrain(c, 0, 100);
        if (vis.conf >= VIS_MIN_CONF) vis.lastFireMs = now;
      }
      break;
    case 'N': vis.fire = false; break;
    case 'X': estopRequested = true;  break;
    case 'G': estopRequested = false; break;
    // One command per line in Serial Monitor; movement is case-insensitive.
    case 'w': case 'W': manualL =  MANUAL_SPEED; manualR =  MANUAL_SPEED; enter(MANUAL); break;
    case 's': case 'S': manualL = -MANUAL_SPEED; manualR = -MANUAL_SPEED; enter(MANUAL); break;
    case 'a': case 'A': manualL = -MANUAL_SPEED; manualR =  MANUAL_SPEED; enter(MANUAL); break;
    case 'd': case 'D': manualL =  MANUAL_SPEED; manualR = -MANUAL_SPEED; enter(MANUAL); break;
    case 'x': manualL = 0;             manualR = 0;             enter(MANUAL); break;
    case 'r': case 'R': enter(SEARCH); break;                        // back to AUTO
  }
}

void readLink() {
  static char buf[32];
  static uint8_t len = 0;
  while (LINK.available()) {
    char ch = LINK.read();
    if (ch == '\n') { buf[len] = 0; if (len) handleLine(buf); len = 0; }
    else if (ch == '\r') {}
    else if (len < sizeof(buf) - 1) buf[len++] = ch;
    else len = 0;
  }
}

void sendTelemetry() {
  static unsigned long lastMs = 0;
  if (millis() - lastMs < 200) return;
  lastMs = millis();
  LINK.print(F("S,"));  LINK.print(STATE_NAME[state]);
  LINK.print(',');      LINK.print(sen.dF);
  LINK.print(',');      LINK.print(sen.dL);
  LINK.print(',');      LINK.print(sen.dR);
  LINK.print(',');      LINK.print(sen.flame ? 1 : 0);
  LINK.print(',');      LINK.print(!visionOnline() ? '-' : fireNow() ? 'F' : 'N');
  LINK.print(',');      LINK.print(vis.dist);
  LINK.print(',');      LINK.println(pumpOn ? 1 : 0);
}

// ----------------------------------------------------------------------------
//  DECISIONS
// ----------------------------------------------------------------------------
bool visionOnline() { return vis.lastPacketMs && millis() - vis.lastPacketMs < VISION_TIMEOUT_MS; }
bool fireNow()      { return visionOnline() && vis.fire && vis.conf >= VIS_MIN_CONF; }
bool fireRecent()   { return vis.lastFireMs && millis() - vis.lastFireMs < VISION_MEMORY_MS; }
bool distKnown()    { return vis.dist < 999; }

// Request the AIM stage only when the flame sensor detects flame. With camera,
// also require recent fire evidence and near range (camera or centre sonar).
// The AIM stage separately requires USE_SERVO before it can enable the pump.
bool sprayAllowed() {
  if (!sen.flame) return false;
  if (!visionOnline()) return true;
  if (!fireRecent()) return false;
  bool nearByCam   = distKnown() && vis.dist <= SPRAY_MAX_CM;
  bool nearBySonar = sen.dF <= SPRAY_MAX_CM;
  return nearByCam || nearBySonar || !distKnown();
}

// ----------------------------------------------------------------------------
//  STATE MACHINE
// ----------------------------------------------------------------------------
void enter(State s) {
  if (s == state) return;
  state = s;
  stateStartMs = millis();
  stopMotors();
  if (s != EXTINGUISH) setPump(false);
  if (s == AVOID)  { avoidPhase = 0; avoidPhaseMs = millis(); avoidDir = (sen.dL > sen.dR) ? -1 : 1; }
  if (s == SEARCH) { burstCount = 0; aimNozzle(0); }
  if (s == TRACK)  lastEvidenceMs = millis();
}
unsigned long inState() { return millis() - stateStartMs; }

void spinToward(float d, int speed) {          // d < 0 left, d > 0 right
  if (d > 0) setMotors(speed, -speed); else setMotors(-speed, speed);
}

void doSearch() {
  if (sprayAllowed()) { enter(AIM); return; }
  if (fireNow())      { enter(TRACK); return; }
  unsigned long t = inState() % (SEARCH_REVOLUTION_MS + SEARCH_PAUSE_MS);
  if (t < SEARCH_REVOLUTION_MS) setMotors(SPEED_TURN, -SPEED_TURN);
  else stopMotors();
}

void doTrack() {
  if (sprayAllowed()) { enter(AIM); return; }

  if (fireNow()) lastEvidenceMs = millis();
  else if (millis() - lastEvidenceMs > LOST_MS) { enter(SEARCH); return; }

  if (fireNow() && distKnown() && vis.dist <= STOP_DIST_CM) { enter(APPROACH); return; }

  if (sen.dF < OBSTACLE_CM) {
    // is the thing in front the fire itself?
    bool fireAhead = fireNow() && abs(vis.err) < 25 && (!distKnown() || vis.dist <= sen.dF + 30);
    enter(fireAhead ? APPROACH : AVOID);
    return;
  }

  float d; int base;
  bool slow = (distKnown() && vis.dist < SLOW_DIST_CM) || sen.dF < 2 * OBSTACLE_CM;
  if (fireNow()) {
    d = vis.err / 100.0;
    base = slow ? SPEED_SLOW : SPEED_FWD;
  } else {                // camera briefly lost fire: keep going straight, slowly
    d = 0; base = SPEED_SLOW;
  }

  if (abs(d) > 0.6) { spinToward(d, SPEED_TURN); return; }
  int turn = (int)(STEER_KP * 100 * d);
  int left = base + turn, right = base - turn;
  if (sen.dL < SIDE_MIN_CM) { left += 20; right -= 20; }
  if (sen.dR < SIDE_MIN_CM) { left -= 20; right += 20; }
  setMotors(left, right);
}

void doAvoid() {
  unsigned long t = millis() - avoidPhaseMs;
  switch (avoidPhase) {
    case 0:
      // Pause before turning. This version has no rear sensor, so it never
      // reverses blindly into an unmeasured area.
      stopMotors();
      if (t > 100) { avoidPhase = 1; avoidPhaseMs = millis(); }
      break;
    case 1: {
      bool leftClear = sen.dL > SIDE_MIN_CM;
      bool rightClear = sen.dR > SIDE_MIN_CM;
      if (!leftClear && !rightClear) {
        stopMotors();
        avoidPhaseMs = millis();
        break;
      }
      if (avoidDir < 0 && !leftClear) avoidDir = 1;
      if (avoidDir > 0 && !rightClear) avoidDir = -1;
      setMotors(avoidDir * SPEED_TURN, -avoidDir * SPEED_TURN);
      if (sen.dF > CLEAR_CM && t > 250) {
        avoidPhase = 2;
        avoidPhaseMs = millis();
      } else if (t > 1500) {
        avoidPhaseMs = millis();  // keep turning in measured clear space
      }
      break;
    }
    case 2:
      if ((avoidDir < 0 && sen.dL <= SIDE_MIN_CM)
          || (avoidDir > 0 && sen.dR <= SIDE_MIN_CM)) {
        avoidPhase = 0;
        avoidPhaseMs = millis();
        stopMotors();
        break;
      }
      if (sen.dF <= OBSTACLE_CM) {
        avoidPhase = 0;
        avoidPhaseMs = millis();
        stopMotors();
        break;
      }
      setMotors(SPEED_SLOW, SPEED_SLOW);
      if (t > 700) enter(fireNow() || fireRecent() ? TRACK : SEARCH);
      break;
  }
}

// Fire is close. Line up with it and creep in until the flame sensor fires.
void doApproach() {
  if (sprayAllowed()) { enter(AIM); return; }

  if (!fireRecent()) {                                   // lost it
    if (inState() > LOST_MS) enter(SEARCH);
    stopMotors(); return;
  }
  if (fireNow() && distKnown() && vis.dist > STOP_DIST_CM + 25) { enter(TRACK); return; }  // drifted away
  if (inState() > CREEP_MAX_MS) { enter(SEARCH); return; }  // flame sensor never fired

  if (sen.dF <= OBSTACLE_CM) {
    // A centered, nearby fire may be the sonar return. Otherwise treat it as
    // an obstacle before pivoting or creeping in the close-approach phase.
    bool fireIsAhead = fireNow() && abs(vis.err) <= CENTRE_ERR
        && (!distKnown() || vis.dist <= sen.dF + 30);
    if (!fireIsAhead) { enter(AVOID); return; }
  }

  if (fireNow() && abs(vis.err) > CENTRE_ERR) {          // centre the fire first
    // short turn pulses so we don't overshoot
    if ((millis() / 120) % 2) spinToward(vis.err, SPEED_ALIGN); else stopMotors();
    return;
  }
  bool roomAhead = sen.dF > CREEP_MIN_CM && (!distKnown() || vis.dist > 15);
  if (roomAhead) setMotors(SPEED_CREEP, SPEED_CREEP); else stopMotors();
}

// Stop, point the arm/nozzle at the target, and allow it to settle before the
// pump can be activated. USE_SERVO must be enabled and calibrated on hardware.
void doAim() {
  stopMotors();
  setPump(false);
#if !USE_SERVO
  // Fail safe: do not spray if the arm/nozzle actuator is not configured.
  if (!sen.flame) enter(fireRecent() ? TRACK : SEARCH);
  return;
#else
  if (!sprayAllowed()) {
    if (fireNow() && distKnown() && vis.dist <= STOP_DIST_CM) enter(APPROACH);
    else if (fireRecent()) enter(TRACK);
    else enter(SEARCH);
    return;
  }

  int offset = fireNow()
      ? constrain(vis.err * SERVO_RANGE / 100, -SERVO_RANGE, SERVO_RANGE)
      : 0;
  aimNozzle(offset);
  if (inState() >= ARM_AIM_MS) enter(EXTINGUISH);
#endif
}

void doExtinguish() {
  stopMotors();
  setPump(true);
  float d = fireNow() ? vis.err / 100.0 : 0;
  unsigned long p = millis() % SWEEP_PERIOD_MS;
  int tri = (p < SWEEP_PERIOD_MS / 2) ? (int)p : (int)(SWEEP_PERIOD_MS - p);
  aimNozzle((int)(d * SERVO_RANGE) + map(tri, 0, SWEEP_PERIOD_MS / 2, -SWEEP_DEG, SWEEP_DEG));

  if (sen.flameGone || inState() > SPRAY_MAX_MS) { burstCount++; enter(VERIFY); }
}

void doVerify() {
  stopMotors();
  aimNozzle(0);
  if (inState() < VERIFY_MS) return;
  if (burstCount < MAX_BURSTS) {
    if (sprayAllowed()) { enter(AIM); return; }
    if (fireNow()) { enter(distKnown() && vis.dist <= STOP_DIST_CM ? APPROACH : TRACK); return; }
  }
  firesPutOut++;
  enter(SEARCH);
}

// Drive by keys; flame sensor still triggers the pump automatically.
void doManual() {
  if (sprayAllowed()) { enter(AIM); return; }
  setMotors(manualL, manualR);
}

#if USE_OLED
void oledCentered(int y, const char* t) { oled.drawStr((128 - oled.getStrWidth(t)) / 2, y, t); }

void drawOled() {
  char big[20], l1[24], l2[24], l3[24];
  bool blinkOn = (millis() / 300) % 2;

  switch (state) {
    case AIM:        strcpy(big, "Aiming..."); break;
    case EXTINGUISH: strcpy(big, "SPRAYING!"); break;
    case SEARCH:     strcpy(big, sen.flame ? "FLAME!" : "Searching..."); break;
    case TRACK:
    case APPROACH:
      if (fireNow() && distKnown()) snprintf(big, sizeof(big), "Fire %d cm", vis.dist);
      else strcpy(big, state == TRACK ? "Going to fire" : "Approaching");
      break;
    case AVOID:  strcpy(big, "Avoiding"); break;
    case VERIFY: strcpy(big, "Checking..."); break;
    case ESTOP:  strcpy(big, "E-STOP"); break;
    case MANUAL: strcpy(big, sen.flame ? "FLAME!" : "Manual"); break;
  }
  if (!visionOnline())      strcpy(l1, "Cam: offline");
  else if (fireNow())       snprintf(l1, sizeof(l1), "Cam: FIRE %dcm", vis.dist);
  else                      strcpy(l1, "Cam: no fire");
  snprintf(l2, sizeof(l2), "Pump:%s  Out:%u", pumpOn ? "ON" : "off", firesPutOut);
#if USE_ULTRASONIC
  snprintf(l3, sizeof(l3), "F%d L%d R%d", sen.dF, sen.dL, sen.dR);
#else
  strcpy(l3, "Sonar: off");
#endif
  bool alarm = (state == AIM) || (state == EXTINGUISH) || sen.flame;

  oled.firstPage();
  do {
    oled.setFont(u8g2_font_6x10_tr);
    oled.drawStr(0, 9, "FIREBOT");
    oled.drawStr(128 - oled.getStrWidth(STATE_NAME[state]), 9, STATE_NAME[state]);
    oled.drawHLine(0, 11, 128);

    oled.setFont(u8g2_font_helvB10_tr);
    if (alarm && blinkOn) { oled.drawBox(0, 13, 128, 20); oled.setDrawColor(0); }
    else                  oled.drawFrame(0, 13, 128, 20);
    oledCentered(28, big);
    oled.setDrawColor(1);

    oled.setFont(u8g2_font_6x10_tr);
    oled.drawStr(0, 43, l1);
    oled.drawStr(98, 43, sen.flame ? "FLM:Y" : "FLM:-");
    oled.drawStr(0, 53, l2);
    oled.drawStr(0, 63, l3);
  } while (oled.nextPage());
}

void updateOled() {
  static unsigned long last = 0;
  if (millis() - last < 250) return;   // drawing takes ~25 ms, don't do it every loop
  last = millis();
  drawOled();
}
#endif

void statusLed() {
  unsigned long period = 1000;
  switch (state) {
    case TRACK: period = 300; break;
    case AVOID: period = 150; break;
    case APPROACH: period = 500; break;
    case AIM:        period = 500; break;
    case EXTINGUISH: digitalWrite(PIN_LED, HIGH); return;
    case ESTOP: period = 80; break;
    default: break;
  }
  digitalWrite(PIN_LED, (millis() / (period / 2)) & 1);
}

// ----------------------------------------------------------------------------
//  SETUP / LOOP
// ----------------------------------------------------------------------------
void setup() {
  pinMode(PIN_RELAY, OUTPUT);                                  // pump OFF first
  digitalWrite(PIN_RELAY, RELAY_ACTIVE_LOW ? HIGH : LOW);

#if MOTOR_DRIVER == 0
  pinMode(PIN_ENA, OUTPUT); pinMode(PIN_ENB, OUTPUT);
#endif
  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
#if USE_ULTRASONIC
  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
#endif
  pinMode(PIN_FLAME, INPUT);
  pinMode(PIN_LED, OUTPUT);
  stopMotors();
#if USE_SERVO
  nozzle.attach(PIN_SERVO);
#endif
  aimNozzle(0);

  Serial.begin(115200);
#if LINK_BLUETOOTH
  LINK.begin(LINK_BAUD);
#endif
  LINK.println(F("# FireBot ready. WASD=manual, x=stop, r=auto, X=E-stop"));
#if USE_OLED
  oled.begin();
  oled.setBusClock(400000);
  oled.firstPage();
  do {
    oled.setFont(u8g2_font_helvB10_tr);
    oledCentered(28, "FIREBOT");
    oled.setFont(u8g2_font_6x10_tr);
    oledCentered(48, "starting...");
  } while (oled.nextPage());
#endif
  delay(1500);
  stateStartMs = millis();
}

void loop() {
  readLink();
  updateSonars();
  updateFlame();

#if CALIBRATION_MODE
  static unsigned long last = 0;
  if (millis() - last > 250) {
    last = millis();
    LINK.print(F("centre=")); LINK.print(sen.dF);
    LINK.print(F(" left="));  LINK.print(sen.dL);
    LINK.print(F(" right=")); LINK.print(sen.dR);
    LINK.print(F(" | flame=")); LINK.print(sen.flameRaw ? F("YES") : F("no"));
    if (FLAME_USE_ANALOG) { LINK.print(F(" ao=")); LINK.print(sen.flameAnalog); }
    LINK.print(F(" | camera=")); LINK.print(fireNow() ? F("FIRE") : (visionOnline() ? F("none") : F("offline")));
    LINK.print(F(" err=")); LINK.print(vis.err);
    LINK.print(F(" dist=")); LINK.println(vis.dist);
  }
#if USE_OLED
  updateOled();
#endif
  return;
#endif

  if (estopRequested && state != ESTOP) enter(ESTOP);

  switch (state) {
    case SEARCH:     doSearch();     break;
    case TRACK:      doTrack();      break;
    case AVOID:      doAvoid();      break;
    case APPROACH:   doApproach();   break;
    case AIM:        doAim();        break;
    case EXTINGUISH: doExtinguish(); break;
    case VERIFY:     doVerify();     break;
    case ESTOP:      stopMotors(); setPump(false); if (!estopRequested) enter(SEARCH); break;
    case MANUAL:     doManual();     break;
  }

  // safety net: pump only in EXTINGUISH, never longer than one burst
  if (state != EXTINGUISH) setPump(false);
  if (pumpOn && millis() - pumpOnSinceMs > SPRAY_MAX_MS + 500) setPump(false);

  statusLed();
#if USE_OLED
  updateOled();
#endif
  sendTelemetry();
}
