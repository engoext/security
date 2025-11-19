// Proximity-scaled alarm (HC-SR04 + LEDs + passive buzzer)
// Pins: TRIG=D10, ECHO=D9, Green LED=D3, Red LED=D4, Buzzer=D11

#define TRIG 10
#define ECHO 9
#define LED_GREEN 3
#define LED_RED   4
#define BUZZER    11

// -------- Tuning --------
// The alarm starts ramping when distance <= FAR_MM, and gets frantic near NEAR_MM.
const int FAR_MM   = 1500;  // start giving sparse beeps inside this distance
const int NEAR_MM  = 150;   // very close (fastest pattern)

// Pattern timing (computed each loop from distance):
const int MAX_PERIOD_MS = 1600;  // was 900 — much slower when far
const int MIN_PERIOD_MS = 110;   // was 120 — a touch faster when near
const float ON_FRACTION = 0.30;  // slightly shorter ON at distance

// Pitch (you chose these — nice!)
const int TONE_MIN_HZ = 400;
const int TONE_MAX_HZ = 1600;

const unsigned long TIMEOUT_US = 38000UL; // pulseIn timeout (~6.5 m)

// Simple smoothing: median of last 3 readings to reduce jitter
int last3[3] = {-1, -1, -1};
int idx = 0;

// Pattern state
unsigned long phaseStartMs = 0;
bool phaseOn = false;

// --- Helpers ---
int readDistanceMm();
int median3(int a, int b, int c);
int mapLinear(int x, int in_min, int in_max, int out_min, int out_max);

void setup() {
  Serial.begin(115200);
  pinMode(TRIG, OUTPUT);  pinMode(ECHO, INPUT);
  pinMode(LED_GREEN, OUTPUT); pinMode(LED_RED, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  digitalWrite(TRIG, LOW);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, LOW);
  noTone(BUZZER);

  // Prime smoothing buffer with a valid reading if possible
  delay(100);
}

void loop() {
  // ---- 1) Read and smooth distance (mm) ----
  int mm = readDistanceMm();
  last3[idx] = mm; idx = (idx + 1) % 3;
  int smoothed = median3(last3[0], last3[1], last3[2]);

  // ---- 2) Decide if we’re in alarm range ----
  bool valid = (smoothed >= 0);
  bool inRange = valid && (smoothed <= FAR_MM);

  // ---- 3) Compute pattern parameters from proximity (linear ramp) ----
  int periodMs = MAX_PERIOD_MS;
  int toneHz   = TONE_MIN_HZ;

  if (inRange) {
    // p = 0 at FAR_MM … 1 at NEAR_MM
    float p = (float)(FAR_MM - smoothed) / (float)(FAR_MM - NEAR_MM);
    if (p < 0) p = 0; if (p > 1) p = 1;

    // Non-linear (cubic) easing: slow changes when far, rapid ramp when close
    float e = p * p * p;  // try p^3 (you can try p^2 for milder, p^4 for stronger)

    periodMs = (int)(MAX_PERIOD_MS - e * (MAX_PERIOD_MS - MIN_PERIOD_MS) + 0.5f);
    toneHz   = (int)(TONE_MIN_HZ  + e * (TONE_MAX_HZ  - TONE_MIN_HZ ) + 0.5f);
  }

  // ---- 4) Drive outputs (non-blocking) ----
  if (!inRange) {
    // Clear state
    phaseOn = false; phaseStartMs = 0;
    digitalWrite(LED_RED, LOW);
    noTone(BUZZER);
    digitalWrite(LED_GREEN, HIGH);      // ready/clear
  } else {
    digitalWrite(LED_GREEN, LOW);       // we're alarming now

    unsigned long now = millis();
    if (phaseStartMs == 0) {            // start a new cycle
      phaseStartMs = now;
      phaseOn = true;
      digitalWrite(LED_RED, HIGH);
      tone(BUZZER, toneHz);
    }

    // How long is the ON part this cycle?
    unsigned long onMs  = (unsigned long)(periodMs * ON_FRACTION);
    unsigned long offMs = (unsigned long)(periodMs - onMs);

    if (phaseOn) {
      if (now - phaseStartMs >= onMs) {
        // switch to OFF
        phaseOn = false;
        phaseStartMs = now;
        digitalWrite(LED_RED, LOW);
        noTone(BUZZER);
      }
    } else {
      if (now - phaseStartMs >= offMs) {
        // switch back to ON (new cycle)
        phaseOn = true;
        phaseStartMs = now;
        digitalWrite(LED_RED, HIGH);
        tone(BUZZER, toneHz);
      }
    }
  }

  // ---- 5) Serial for tuning ----
  Serial.print("mm=");
  Serial.print(smoothed);
  Serial.print("  inRange=");
  Serial.print(inRange ? 1 : 0);
  Serial.print("  periodMs=");
  Serial.print(periodMs);
  Serial.print("  toneHz=");
  Serial.println(toneHz);

  delay(30); // readable stream without affecting responsiveness
}

// ------------ Distance helpers ------------
int readDistanceMm() {
  // 10 µs trigger pulse
  digitalWrite(TRIG, LOW); delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  unsigned long us = pulseIn(ECHO, HIGH, TIMEOUT_US);
  if (us == 0) return -1;  // no echo

  // mm = (us * 0.343) / 2
  float mm = (us * 0.343f) * 0.5f;
  if (mm < 0 || mm > 6000) return -1;
  return (int)(mm + 0.5f);
}

int median3(int a, int b, int c) {
  // handle invalids (-1): prefer any valid values
  int v[3] = {a, b, c};
  int validCount = 0;
  int buf[3];
  for (int i = 0; i < 3; ++i) {
    if (v[i] >= 0) buf[validCount++] = v[i];
  }
  if (validCount == 0) return -1;
  if (validCount == 1) return buf[0];
  if (validCount == 2) {
    return (buf[0] + buf[1]) / 2;
  }
  // 3 valid: sort small array
  if (buf[0] > buf[1]) { int t = buf[0]; buf[0] = buf[1]; buf[1] = t; }
  if (buf[1] > buf[2]) { int t = buf[1]; buf[1] = buf[2]; buf[2] = t; }
  if (buf[0] > buf[1]) { int t = buf[0]; buf[0] = buf[1]; buf[1] = t; }
  return buf[1]; // median
}
