/* Standalone flame sensor test. No motors, relay, or pump are controlled. */

const uint8_t PIN_FLAME = A0;
const uint8_t PIN_STATUS_LED = 13;
const bool FLAME_USE_ANALOG = false;       // true for AO, false for DO
const bool FLAME_DO_ACTIVE_LOW = true;     // common modules assert LOW on flame
const bool FLAME_AO_LOW_IS_FIRE = true;
const int FLAME_AO_THRESHOLD = 500;

void setup() {
  pinMode(PIN_FLAME, INPUT);
  pinMode(PIN_STATUS_LED, OUTPUT);
  Serial.begin(115200);
  Serial.println(F("Flame sensor test started; inspect readings and adjust polarity/threshold."));
}

void loop() {
  bool detected;
  int raw;
  if (FLAME_USE_ANALOG) {
    raw = analogRead(PIN_FLAME);
    int intensity = FLAME_AO_LOW_IS_FIRE ? 1023 - raw : raw;
    detected = intensity > FLAME_AO_THRESHOLD;
    Serial.print(F("AO raw=")); Serial.print(raw);
    Serial.print(F(" intensity=")); Serial.print(intensity);
  } else {
    raw = digitalRead(PIN_FLAME);
    detected = FLAME_DO_ACTIVE_LOW ? raw == LOW : raw == HIGH;
    Serial.print(F("DO raw=")); Serial.print(raw);
  }
  digitalWrite(PIN_STATUS_LED, detected ? HIGH : LOW);
  Serial.print(F(" flame=")); Serial.println(detected ? F("YES") : F("no"));
  delay(200);
}
