/*
 * Standalone HC-SR04 distance test for the FireBot example pin map.
 * Serial Monitor: 115200 baud. Prints front,left,right in centimetres.
 */

#define MOTOR_DRIVER 0  // keep in sync with fire_bot.ino (0=L298N, 1=L293D map)

#if MOTOR_DRIVER == 1
const uint8_t TRIG_FRONT = 2, ECHO_FRONT = 7;
#else
const uint8_t TRIG_FRONT = 2, ECHO_FRONT = 3;
#endif
const uint8_t TRIG_LEFT = A2, ECHO_LEFT = A3;
const uint8_t TRIG_RIGHT = A1, ECHO_RIGHT = 4;

long readCm(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(3);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long duration = pulseIn(echo, HIGH, 25000UL);
  if (duration == 0) return -1;  // no echo / out of range
  return (long)(duration / 58UL);
}

void setup() {
  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT); pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);
  digitalWrite(TRIG_FRONT, LOW); digitalWrite(TRIG_LEFT, LOW); digitalWrite(TRIG_RIGHT, LOW);
  Serial.begin(115200);
  Serial.println(F("front_cm,left_cm,right_cm (-1 means no echo)"));
}

void loop() {
  long front = readCm(TRIG_FRONT, ECHO_FRONT);
  delay(30);  // reduce cross-talk between ultrasonic sensors
  long left = readCm(TRIG_LEFT, ECHO_LEFT);
  delay(30);
  long right = readCm(TRIG_RIGHT, ECHO_RIGHT);
  Serial.print(front); Serial.print(',');
  Serial.print(left); Serial.print(',');
  Serial.println(right);
  delay(150);
}
