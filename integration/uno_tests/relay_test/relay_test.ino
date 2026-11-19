/*
 * Standalone relay output test. Pump is OFF at boot and each ON pulse is
 * limited to 1.5 seconds. Start with pump disconnected and verify relay polarity.
 * Serial Monitor: 115200 baud, Newline. Send 1 = relay on, 0 = relay off.
 */

const uint8_t PIN_RELAY = 10;
const bool RELAY_ACTIVE_LOW = true;   // matches the pasted FireBot pump test; verify on your module
const unsigned long MAX_ON_MS = 1500;

bool relayOn = false;
unsigned long relayStartedMs = 0;

void setRelay(bool on) {
  relayOn = on;
  digitalWrite(PIN_RELAY, (on ^ RELAY_ACTIVE_LOW) ? HIGH : LOW);
  if (on) relayStartedMs = millis();
  Serial.println(on ? F("relay ON (timed pulse)") : F("relay OFF"));
}

void setup() {
  digitalWrite(PIN_RELAY, RELAY_ACTIVE_LOW ? HIGH : LOW);  // safe OFF level first
  pinMode(PIN_RELAY, OUTPUT);
  Serial.begin(115200);
  Serial.println(F("Relay test ready: 1=ON for max 1.5s, 0=OFF."));
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '1') setRelay(true);
    else if (c == '0') setRelay(false);
  }
  if (relayOn && millis() - relayStartedMs >= MAX_ON_MS) {
    setRelay(false);
    Serial.println(F("maximum ON time reached"));
  }
}
