/*
  High-Tech Security System — Proximity-Scaled Alarm (HC-SR04 + LEDs + Buzzer)
  ---------------------------------------------------------------------------
  What this does:
    - Measures distance with an HC-SR04 (TRIG D10, ECHO D9)
    - Decides if someone is within the "attention zone" (≤ FAR distance)
    - Communicates by beeping/blinking: slower when far, faster/more urgent when near
    - Camera-free, privacy-friendly; matches the slides' "Sense → Decide → Communicate"

  How to tune (edit ONLY the TUNABLES section below):
    - FAR_DISTANCE_MM: where sparse beeps begin
    - NEAR_DISTANCE_MM: where the pattern is most urgent
    - MAX/MIN_PERIOD_MS: slowest vs fastest beep cycle length
    - ON_FRACTION: how much of each cycle is ON (LED/buzzer)
    - TONE_MIN/MAX_HZ: pitch range as proximity increases

  Hardware (unchanged):
    - TRIG=D10, ECHO=D9, GREEN LED=D3 (ready), RED LED=D4 (alarm), BUZZER=D11
    - LEDs use 470 Ω resistors to GND (anode on the pin via resistor, cathode to GND)
*/

#include <Arduino.h>

// ------------------------------
// Pin mapping (hardware wiring)
// ------------------------------
const uint8_t PIN_TRIG      = 10;
const uint8_t PIN_ECHO      = 9;
const uint8_t PIN_LED_GREEN = 3;   // "ready/clear" indicator
const uint8_t PIN_LED_RED   = 4;   // "alarming" indicator
const uint8_t PIN_BUZZER    = 11;  // passive buzzer (tone capable)

// Sensor reading
const unsigned long TIMEOUT_US = 38000UL; // pulseIn timeout (~6.5 m)

// ------------------------------
// TUNABLES — CHANGE ME!
// ------------------------------
// Alarm begins ramping when distance <= FAR; becomes urgent near NEAR.
const int FAR_DISTANCE_MM   = ?;   // start sparse beeps inside this distance
const int NEAR_DISTANCE_MM  = ?;    // very close (fastest pattern)

// Pattern timing (derived each loop from distance)
const int   MAX_PERIOD_MS   = ?;   // slow beeps when far
const int   MIN_PERIOD_MS   = ?;    // fast beeps when near
const float ON_FRACTION     = 0.30f;  // portion of each cycle that is ON (LED/buzzer)

// Pitch ramp (optional: proximity → higher pitch)
const int TONE_MIN_HZ       = ?;
const int TONE_MAX_HZ       = ?;

// ------------------------------------------------------------------------------------------------------------------------
// Internal state (do not edit)
// ------------------------------------------------------------------------------------------------------------------------

// Simple smoothing buffer: median of last 3 distances (in mm, -1 = invalid)
static int      distanceBuf[3] = { -1, -1, -1 };
static uint8_t  distanceIdx    = 0;

// Pattern phase state (non-blocking blinks/beeps)
struct PatternState {
  unsigned long phaseStartMs = 0; // start time of current ON or OFF phase
  bool          phaseOn      = false; // true=ON (LED/tone), false=OFF (quiet)
};
static PatternState pattern;

// ------------------------------
// Forward declarations
// ------------------------------
int  readDistanceMmOnce();
int  readAndSmoothDistanceMm();
int  medianOf3_allowInvalid(int a, int b, int c);

struct PatternParams {
  int periodMs;           // full cycle length
  int toneHz;             // tone frequency for current proximity
  unsigned long onMs;     // ON duration in this cycle
  unsigned long offMs;    // OFF duration in this cycle
};
PatternParams computePatternFromDistance(int smoothedMm, bool inRange);
void driveOutputs(bool inRange, const PatternParams& p, PatternState& s);
void logStatus(int smoothedMm, bool inRange, const PatternParams& p);

// ------------------------------
// Setup
// ------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  // Known safe outputs on boot
  digitalWrite(PIN_TRIG, LOW);
  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_RED, LOW);
  noTone(PIN_BUZZER);

  delay(100); // sensor settle
}

// ------------------------------
// Main loop (reads like pseudocode)
// ------------------------------
void loop() {
  // 1) Sense: read and smooth distance (mm), -1 if invalid/out of range
  const int smoothedMm = readAndSmoothDistanceMm();

  // 2) Decide: are we inside the attention zone?
  const bool valid   = (smoothedMm >= 0);
  const bool inRange = valid && (smoothedMm <= FAR_DISTANCE_MM);

  // 3) Map proximity → pattern parameters (period & tone), using cubic easing
  const PatternParams params = computePatternFromDistance(smoothedMm, inRange);

  // 4) Communicate: drive LEDs + buzzer with a non-blocking state machine
  driveOutputs(inRange, params, pattern);

  // 5) Observe: print status for tuning
  logStatus(smoothedMm, inRange, params);

  // Human-readable update rate without affecting responsiveness
  delay(30);
}

// ======================================================================
// Implementation
// ======================================================================

// Trigger one HC-SR04 measurement; return distance in mm, or -1 if invalid
int readDistanceMmOnce() {
  // Spec: 10 µs trigger pulse
  digitalWrite(PIN_TRIG, LOW); delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  // Echo HIGH width in microseconds (timeout prevents hanging)
  const unsigned long us = pulseIn(PIN_ECHO, HIGH, TIMEOUT_US);
  if (us == 0) return -1; // no echo (too far, bad aim, soft/angled target)

  // Distance (mm) = (us * speed_of_sound_mm_per_us) / 2
  // Speed of sound ≈ 0.343 mm/µs at room temp; divide by 2 for out-and-back
  const float mm = (us * 0.343f) * 0.5f;

  if (mm < 0.0f || mm > 6000.0f) return -1; // sanity clamp
  return (int)(mm + 0.5f);                  // round to nearest mm
}

// Update 3-sample buffer and return median (ignores invalids when possible)
int readAndSmoothDistanceMm() {
  const int mm = readDistanceMmOnce();
  distanceBuf[distanceIdx] = mm;
  distanceIdx = (uint8_t)((distanceIdx + 1) % 3);
  return medianOf3_allowInvalid(distanceBuf[0], distanceBuf[1], distanceBuf[2]);
}

// Median of 3 that tolerates -1 (invalid). If all invalid → -1.
int medianOf3_allowInvalid(int a, int b, int c) {
  int tmp[3];
  int n = 0;
  if (a >= 0) tmp[n++] = a;
  if (b >= 0) tmp[n++] = b;
  if (c >= 0) tmp[n++] = c;

  if (n == 0) return -1;
  if (n == 1) return tmp[0];
  if (n == 2) return (tmp[0] + tmp[1]) / 2;

  // n == 3: sort small array
  if (tmp[0] > tmp[1]) { int t = tmp[0]; tmp[0] = tmp[1]; tmp[1] = t; }
  if (tmp[1] > tmp[2]) { int t = tmp[1]; tmp[1] = tmp[2]; tmp[2] = t; }
  if (tmp[0] > tmp[1]) { int t = tmp[0]; tmp[0] = tmp[1]; tmp[1] = t; }
  return tmp[1];
}

// Convert proximity → pattern parameters (period & tone), using cubic easing
PatternParams computePatternFromDistance(int smoothedMm, bool inRange) {
  PatternParams p;
  // Defaults when out of range (we won't use tone/period, but keep consistent)
  p.periodMs = MAX_PERIOD_MS;
  p.toneHz   = TONE_MIN_HZ;

  if (inRange) {
    // p = 0 at FAR … 1 at NEAR
    float norm = (float)(FAR_DISTANCE_MM - smoothedMm) /
                 (float)(FAR_DISTANCE_MM - NEAR_DISTANCE_MM);
    if (norm < 0.0f) norm = 0.0f;
    if (norm > 1.0f) norm = 1.0f;

    // Cubic easing: slow change when far, rapid ramp when close
    const float ease = norm * norm * norm; // p^3

    p.periodMs = (int)(MAX_PERIOD_MS - ease * (MAX_PERIOD_MS - MIN_PERIOD_MS) + 0.5f);
    p.toneHz   = (int)(TONE_MIN_HZ  + ease * (TONE_MAX_HZ  - TONE_MIN_HZ ) + 0.5f);
  }

  // ON/OFF split for this cycle
  p.onMs  = (unsigned long)(p.periodMs * ON_FRACTION);
  p.offMs = (unsigned long)(p.periodMs - p.onMs);
  return p;
}

// Drive LEDs + buzzer based on proximity pattern (non-blocking state machine)
void driveOutputs(bool inRange, const PatternParams& p, PatternState& s) {
  if (!inRange) {
    // Clear state and show "ready"
    s.phaseOn = false;
    s.phaseStartMs = 0;
    digitalWrite(PIN_LED_RED, LOW);
    noTone(PIN_BUZZER);
    digitalWrite(PIN_LED_GREEN, HIGH);
    return;
  }

  // In alarm range: green off, run blink/beep pattern on red + buzzer
  digitalWrite(PIN_LED_GREEN, LOW);

  const unsigned long now = millis();

  // Start a new cycle if needed
  if (s.phaseStartMs == 0) {
    s.phaseStartMs = now;
    s.phaseOn = true;
    digitalWrite(PIN_LED_RED, HIGH);
    tone(PIN_BUZZER, p.toneHz);
    return;
  }

  if (s.phaseOn) {
    if (now - s.phaseStartMs >= p.onMs) {
      // Switch to OFF phase
      s.phaseOn = false;
      s.phaseStartMs = now;
      digitalWrite(PIN_LED_RED, LOW);
      noTone(PIN_BUZZER);
    }
  } else {
    if (now - s.phaseStartMs >= p.offMs) {
      // Switch back to ON phase (new cycle)
      s.phaseOn = true;
      s.phaseStartMs = now;
      digitalWrite(PIN_LED_RED, HIGH);
      tone(PIN_BUZZER, p.toneHz);
    }
  }
}

// Serial logging for tuning (kept identical keys/format)
void logStatus(int smoothedMm, bool inRange, const PatternParams& p) {
  Serial.print("mm=");
  Serial.print(smoothedMm);
  Serial.print("  inRange=");
  Serial.print(inRange ? 1 : 0);
  Serial.print("  periodMs=");
  Serial.print(p.periodMs);
  Serial.print("  toneHz=");
  Serial.println(p.toneHz);
}
