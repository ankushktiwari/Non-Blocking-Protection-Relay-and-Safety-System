// Non-Blocking Protection Relay and Safety System (ESP32, Wokwi)
// No delay() anywhere: everything runs from millis() timers and a state machine.
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <stdarg.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

// ---------- Pins ----------
#define SENSOR_PIN 34      // pot = load current sensor (0-10 A)
#define RED_LED 2
#define GREEN_LED 4
#define YELLOW_LED 5
#define BUZZER 18
#define RELAY 19           // ACTIVE LOW: LOW = load connected, HIGH = cut off
#define BTN_INJECT 25      // cycles the injected fault type
#define BTN_RESET 26       // clears LOCKOUT
#define BTN_STORM 27       // toggles continuous random fault injection

// ---------- Task periods (ms) ----------
const uint32_t SAMPLE_MS = 20;     // 50 Hz sensor + protection loop
const uint32_t LCD_MS    = 250;
const uint32_t STAT_MS   = 5000;

// ---------- Sensor validation ----------
const float LIM_MIN = -0.5, LIM_MAX = 10.5;   // valid reading range (A)
const float ADC_NOISE_A = 0.03;               // simulated ADC noise (+/-)
const int   WIN = 30;                         // sample window (0.6 s)
const float FROZEN_EPS = 0.005;               // window range below this = frozen
const int   NOISE_N = 20;                     // samples used for noise test
const float NOISE_STD = 0.6;                  // std-dev limit (A)
const int   NOISE_REV = 5;                    // direction reversals needed
const float JUMP_LIMIT = 3.5;                 // max believable change per sample (A)
const float JUMP_TOL = 1.0;                   // new level must hold within this
const int   JUMP_CONFIRM = 4;                 // samples that must hold (80 ms)
const int   SCORE_ADD = 4, SCORE_MAX = 20;    // sensor health score
const int   SCORE_TRIP = 10, SCORE_CLEAR = 2;
const uint32_t SENSOR_OK_HOLD_MS = 500;

// ---------- Protection (hysteresis + debounce) ----------
const float OC_TRIP = 8.0, OC_CLEAR = 7.0;    // 1 A hysteresis band
const uint32_t OC_TRIP_MS = 200, OC_CLEAR_MS = 1000;

// ---------- Recovery sequence ----------
const uint32_t COOLDOWN_MS = 3000;            // relay stays open
const uint32_t PROBATION_MS = 3000;           // relay closed, watched closely
const int      MAX_RETRIES = 3;               // failed re-closes before LOCKOUT
const uint32_t STABLE_RESET_MS = 10000;       // stable this long = retries cleared

// ---------- Types ----------
enum State  { ST_NORMAL, ST_TRIPPED, ST_COOLDOWN, ST_PROBATION, ST_LOCKOUT };
enum Anom   { A_NONE, A_RANGE, A_FROZEN, A_JUMP, A_NOISE };
enum Inject { INJ_NONE, INJ_FREEZE, INJ_SPIKE, INJ_RANGE, INJ_NOISE, INJ_STEP };
const char* STATE_NAME[] = {"NORMAL", "TRIPPED", "COOLDOWN", "PROBATION", "LOCKOUT"};
const char* ANOM_NAME[]  = {"NONE", "RANGE", "FROZEN", "JUMP", "NOISE"};
const char* INJ_NAME[]   = {"NONE", "FREEZE", "SPIKE", "RANGE", "NOISE", "STEP"};

// ---------- Global data ----------
State  state = ST_NORMAL;
Inject inject = INJ_NONE;
uint32_t stateSince = 0;
int    retries = 0;
bool   relayOn = true, storm = false;
bool   ocLatched = false, sensorFault = false;
float  rawCur = 0, cur = 0;                 // raw reading, validated reading
uint32_t transitions = 0, relayOps = 0, anomalies = 0, samples = 0, stalls = 0;
uint32_t maxLoopUs = 0;

// Button record (declared early so the Arduino auto-prototypes can see it)
struct Btn { uint8_t pin; bool stable, last; uint32_t t; };
Btn bInject = {BTN_INJECT, HIGH, HIGH, 0}, bReset = {BTN_RESET, HIGH, HIGH, 0}, bStorm = {BTN_STORM, HIGH, HIGH, 0};

// ---------- Event log ----------
void logf(const char* tag, const char* fmt, ...) {
  char msg[110];
  va_list ap; va_start(ap, fmt); vsnprintf(msg, sizeof msg, fmt, ap); va_end(ap);
  uint32_t t = millis();
  Serial.printf("[%5lu.%03lu] %-8s %s\n", (unsigned long)(t / 1000), (unsigned long)(t % 1000), tag, msg);
}

// ---------- Relay (the only place that drives the pin) ----------
bool resync = false;   // tells the validator the load changed on purpose
void setRelay(bool on) {
  if (on == relayOn) return;
  relayOn = on; relayOps++; resync = true;
  digitalWrite(RELAY, on ? LOW : HIGH);
  logf("RELAY", on ? "CLOSED (load connected)" : "OPEN (load cut off)");
}

// ---------- State change (the only place that changes state) ----------
void setState(State s, const char* why) {
  if (s == state) return;
  logf("STATE", "%s -> %s | %s | I=%.2fA", STATE_NAME[state], STATE_NAME[s], why, cur);
  state = s; stateSince = millis(); transitions++;
}

// ---------- Sample window (ring buffer) ----------
float win[WIN]; int wHead = 0, wCount = 0;
void wPush(float x) { win[wHead] = x; wHead = (wHead + 1) % WIN; if (wCount < WIN) wCount++; }
float wAgo(int k) { return win[(wHead - 1 - k + 2 * WIN) % WIN]; }   // k=0 newest

bool isFrozen() {
  if (wCount < WIN) return false;
  float lo = win[0], hi = win[0];
  for (int i = 1; i < WIN; i++) { if (win[i] < lo) lo = win[i]; if (win[i] > hi) hi = win[i]; }
  return (hi - lo) < FROZEN_EPS;
}

// Noise = large spread AND many direction reversals (a genuine load change
// is large but moves one way, so it has few reversals).
bool isNoisy() {
  if (wCount < NOISE_N) return false;
  float sum = 0, sq = 0;
  for (int k = 0; k < NOISE_N; k++) { float v = wAgo(k); sum += v; sq += v * v; }
  float mean = sum / NOISE_N, var = sq / NOISE_N - mean * mean;
  if (var < 0) var = 0;
  if (sqrtf(var) < NOISE_STD) return false;
  int rev = 0;
  for (int k = 0; k < NOISE_N - 2; k++) {
    float d1 = wAgo(k) - wAgo(k + 1), d2 = wAgo(k + 1) - wAgo(k + 2);
    if (fabsf(d1) > 0.2 && fabsf(d2) > 0.2 && d1 * d2 < 0) rev++;
  }
  return rev >= NOISE_REV;
}

// ---------- Jump check: spike vs genuine load step ----------
float lastGood = 0, jumpLevel = 0; bool haveGood = false, jumpPending = false; int jumpCount = 0;
Anom jumpCheck(float x) {
  Anom r = A_NONE;
  if (!haveGood || resync) { lastGood = x; haveGood = true; resync = false; jumpPending = false; return A_NONE; }
  if (jumpPending) {
    if (fabsf(x - jumpLevel) <= JUMP_TOL) {            // new level is holding
      if (++jumpCount >= JUMP_CONFIRM) {
        jumpPending = false; lastGood = x;
        logf("SENSOR", "genuine load step accepted: %.2fA", x);
      }
      return A_NONE;                                   // keep last good value meanwhile
    }
    jumpPending = false; r = A_JUMP;                   // did not hold = spike
  }
  if (fabsf(x - lastGood) > JUMP_LIMIT) { jumpPending = true; jumpLevel = x; jumpCount = 1; }
  else lastGood = x;
  return r;
}

Anom validate(float x) {
  if (x < LIM_MIN || x > LIM_MAX) return A_RANGE;      // out of range: not stored
  wPush(x);
  Anom j = jumpCheck(x);
  if (isFrozen()) return A_FROZEN;
  if (j == A_JUMP) return A_JUMP;
  if (isNoisy()) return A_NOISE;
  return A_NONE;
}

// ---------- Sensor health (leaky score = anomaly debounce) ----------
int score = 0; Anom lastAnom = A_NONE, lastLogged = A_NONE; uint32_t lastAnomLog = 0;
bool okTimerOn = false; uint32_t okSince = 0;
void healthUpdate(Anom a) {
  uint32_t now = millis();
  if (a != A_NONE) {
    score = min(score + SCORE_ADD, SCORE_MAX); anomalies++; lastAnom = a;
    if (a != lastLogged || now - lastAnomLog >= 1000) {   // rate-limited log
      logf("ANOMALY", "%s (raw=%.2fA score=%d)", ANOM_NAME[a], rawCur, score);
      lastLogged = a; lastAnomLog = now;
    }
  } else if (score > 0) score--;

  if (!sensorFault) {
    if (score >= SCORE_TRIP) { sensorFault = true; logf("SENSOR", "FAULT asserted (%s)", ANOM_NAME[lastAnom]); }
  } else if (score <= SCORE_CLEAR) {
    if (!okTimerOn) { okTimerOn = true; okSince = now; }
    else if (now - okSince >= SENSOR_OK_HOLD_MS) { sensorFault = false; okTimerOn = false; logf("SENSOR", "FAULT cleared"); }
  } else okTimerOn = false;
}

// ---------- Overcurrent comparator: hysteresis + debounce ----------
bool ocTimerOn = false; uint32_t ocT0 = 0;
void ocUpdate() {
  uint32_t now = millis();
  bool cond = ocLatched ? (cur < OC_CLEAR) : (cur > OC_TRIP);   // different limits = hysteresis
  if (!cond) { ocTimerOn = false; return; }
  if (!ocTimerOn) { ocTimerOn = true; ocT0 = now; return; }
  uint32_t need = ocLatched ? OC_CLEAR_MS : OC_TRIP_MS;         // debounce time
  if (now - ocT0 >= need) {
    ocLatched = !ocLatched; ocTimerOn = false;
    if (ocLatched) logf("PROTECT", "OVERCURRENT asserted: %.2fA > %.1fA for %lums", cur, OC_TRIP, (unsigned long)need);
    else logf("PROTECT", "overcurrent cleared: %.2fA < %.1fA for %lums", cur, OC_CLEAR, (unsigned long)need);
  }
}

// ---------- Protection state machine ----------
void fsmStep() {
  uint32_t now = millis();
  bool fault = ocLatched || sensorFault;
  switch (state) {
    case ST_NORMAL:
      if (fault) {
        setRelay(false);
        setState(ST_TRIPPED, sensorFault ? "sensor fault" : "overcurrent");
      } else if (retries && now - stateSince >= STABLE_RESET_MS) {
        retries = 0; logf("INFO", "stable, retry counter cleared");
      }
      break;
    case ST_TRIPPED:
      if (!fault) setState(ST_COOLDOWN, "fault cleared, cooldown started");
      break;
    case ST_COOLDOWN:
      if (fault) setState(ST_TRIPPED, "fault returned");
      else if (now - stateSince >= COOLDOWN_MS) {
        setRelay(true);
        setState(ST_PROBATION, "cooldown done, relay re-closed");
      }
      break;
    case ST_PROBATION:
      if (fault) {
        retries++; setRelay(false);
        if (retries >= MAX_RETRIES) setState(ST_LOCKOUT, "too many failed re-closes");
        else setState(ST_TRIPPED, "fault during probation");
      } else if (now - stateSince >= PROBATION_MS) setState(ST_NORMAL, "probation passed");
      break;
    case ST_LOCKOUT:
      break;                                            // waits for the reset button
  }
}

// ---------- Fault injection (simulates sensor failures) ----------
Inject injPrev = INJ_NONE; float held = 0; int spikeLeft = 0, spikeGap = 0;
float applyInjection(float x) {
  if (inject != injPrev) { injPrev = inject; held = x; spikeLeft = 0; spikeGap = 0; }
  switch (inject) {
    case INJ_FREEZE: return held;                       // value stuck
    case INJ_SPIKE:                                     // 2-sample glitches
      if (spikeLeft > 0) { spikeLeft--; return x + (x < 5 ? 5.0 : -5.0); }
      if (++spikeGap >= 2) { spikeGap = 0; spikeLeft = 2; }
      return x;
    case INJ_RANGE: return 14.0;                        // impossible value
    case INJ_NOISE: return x + random(-1500, 1501) / 1000.0;
    case INJ_STEP:  return x + 4.5;                     // real load step
    default: return x;
  }
}

float readSensor() {
  float amps = analogRead(SENSOR_PIN) / 4095.0 * 10.0;
  if (!relayOn) amps = 0;                               // load disconnected
  amps += random(-30, 31) / 1000.0;                     // ADC noise
  return applyInjection(amps);
}

// Storm mode: random fault types and durations, forever
uint32_t stormNext = 0;
void stormTask() {
  uint32_t now = millis();
  if (!storm || (int32_t)(now - stormNext) < 0) return;
  if (inject == INJ_NONE) {
    inject = (Inject)random(1, 6); stormNext = now + random(300, 2500);
    logf("INJECT", "storm: %s", INJ_NAME[inject]);
  } else {
    inject = INJ_NONE; stormNext = now + random(200, 1500);
    logf("INJECT", "storm: cleared");
  }
}

// ---------- Buttons (non-blocking debounce) ----------
bool pressed(Btn &b) {
  bool r = digitalRead(b.pin); uint32_t now = millis();
  if (r != b.last) { b.last = r; b.t = now; }
  if (r != b.stable && now - b.t >= 30) { b.stable = r; return r == LOW; }
  return false;
}
void buttonsTask() {
  if (pressed(bInject)) {
    if (storm) logf("INJECT", "ignored, storm mode is on");
    else { inject = (Inject)((inject + 1) % 6); logf("INJECT", "manual: %s", INJ_NAME[inject]); }
  }
  if (pressed(bStorm)) {
    storm = !storm; stormNext = millis();
    if (!storm) inject = INJ_NONE;
    logf("INJECT", storm ? "STORM MODE ON" : "STORM MODE OFF");
  }
  if (pressed(bReset)) {
    if (state == ST_LOCKOUT) { retries = 0; setState(ST_COOLDOWN, "manual reset"); }
    else logf("INFO", "reset ignored (not in lockout)");
  }
}

// ---------- LEDs and buzzer (blink from millis, no delay) ----------
void outputsTask() {
  uint32_t now = millis();
  bool slow = (now / 500) % 2, fast = (now / 125) % 2;
  bool r = false, g = false, y = false, bz = false;
  switch (state) {
    case ST_NORMAL:    g = true; break;
    case ST_TRIPPED:   r = true; bz = (now % 1000) < 200; break;
    case ST_COOLDOWN:  y = slow; break;
    case ST_PROBATION: y = fast; break;
    case ST_LOCKOUT:   r = fast; bz = (now % 400) < 200; break;
  }
  digitalWrite(RED_LED, r); digitalWrite(GREEN_LED, g);
  digitalWrite(YELLOW_LED, y); digitalWrite(BUZZER, bz);
}

// ---------- LCD ----------
void lcdLine(uint8_t row, const char* s) {
  char b[17]; snprintf(b, sizeof b, "%-16s", s);
  lcd.setCursor(0, row); lcd.print(b);
}
void lcdTask() {
  char a[24], b[24];
  snprintf(a, sizeof a, "%4.1fA %s%s", cur, STATE_NAME[state], storm ? "*" : "");
  snprintf(b, sizeof b, "Inj:%-6s %s", INJ_NAME[inject], sensorFault ? "S:BAD" : "S:OK");
  lcdLine(0, a); lcdLine(1, b);
}

// ---------- One protection cycle ----------
void sampleTask() {
  samples++;
  rawCur = readSensor();
  Anom a = validate(rawCur);
  cur = lastGood;
  healthUpdate(a);
  ocUpdate();
  fsmStep();
  stormTask();
}

void setup() {
  Serial.begin(115200);
  pinMode(RED_LED, OUTPUT); pinMode(GREEN_LED, OUTPUT); pinMode(YELLOW_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT); pinMode(RELAY, OUTPUT);
  pinMode(BTN_INJECT, INPUT_PULLUP); pinMode(BTN_RESET, INPUT_PULLUP); pinMode(BTN_STORM, INPUT_PULLUP);
  lcd.init(); lcd.backlight();
  digitalWrite(RELAY, LOW);                             // ACTIVE LOW -> ON
  logf("BOOT", "=== NON-BLOCKING PROTECTION RELAY STARTED ===");
  lcdLine(0, "Protection Relay"); lcdLine(1, "Starting...");
}

void loop() {
  static uint32_t tSample = 0, tLcd = 0, tStat = 0, lastUs = 0;
  uint32_t now = millis(), us = micros();

  // loop health: a blocking call would show up here as a stall
  if (lastUs) {
    uint32_t dt = us - lastUs;
    if (dt > maxLoopUs) maxLoopUs = dt;
    if (dt > 50000) stalls++;
  }
  lastUs = us;

  if (now - tSample >= SAMPLE_MS) {
    tSample += SAMPLE_MS;
    if (now - tSample > 5 * SAMPLE_MS) tSample = now;   // never burst to catch up
    sampleTask();
  }
  if (now - tLcd >= LCD_MS) { tLcd = now; lcdTask(); }
  if (now - tStat >= STAT_MS) {
    tStat = now;
    logf("STAT", "state=%s I=%.2fA samples=%lu trans=%lu relayOps=%lu anomalies=%lu retries=%d maxLoop=%luus stalls=%lu",
         STATE_NAME[state], cur, (unsigned long)samples, (unsigned long)transitions, (unsigned long)relayOps,
         (unsigned long)anomalies, retries, (unsigned long)maxLoopUs, (unsigned long)stalls);
  }
  buttonsTask();
  outputsTask();
}
