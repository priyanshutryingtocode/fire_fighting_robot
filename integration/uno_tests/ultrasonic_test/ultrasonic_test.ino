/* Standalone single-front HC-SR04 test. Serial Monitor: 115200 baud. */

#define MOTOR_DRIVER 0  // same as fire_bot.ino: 0=L298N, 1=L293D board map
const uint8_t PIN_TRIG = 2;
#if MOTOR_DRIVER == 1
const uint8_t PIN_ECHO = 7;
#else
const uint8_t PIN_ECHO = 3;
#endif

int readFrontCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(3);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long duration = pulseIn(PIN_ECHO, HIGH, 25000UL);
  return duration == 0 ? 400 : (int)(duration / 58UL);
}

void setup() {
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
  Serial.begin(115200);
  Serial.println(F("front distance (cm); 400 means no echo"));
}

void loop() {
  Serial.println(readFrontCm());
  delay(100);
}
