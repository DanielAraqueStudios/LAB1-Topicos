// Sprint 1 - open-loop characterization of the jib crane (ESP32, Arduino core 2.x or 3.x)
// Serial commands (115200): 's' = step test, 'p' = PRBS test, 'x' = abort, 'i' = print config
// Data is buffered in RAM during the run (no Serial in the sampling loop) and dumped as CSV afterwards:
//   k,t_us,u_pct,y_jib_raw,y_cable_raw     (raw = 12-bit ADC counts, 0..4095)

#include <Arduino.h>

// ---------- Hardware (EDIT to match your wiring) ----------
const int PIN_PWM   = 25;   // H-bridge enable/PWM
const int PIN_DIR   = 26;   // H-bridge direction
const int PIN_JIB   = 34;   // ADC1_CH6, potentiometer on jib angle   (ADC1 only: ADC2 conflicts with WiFi)
const int PIN_CABLE = 35;   // ADC1_CH7, potentiometer on cable angle
const int PWM_FREQ  = 20000;  // Hz, above audible range
const int PWM_BITS  = 10;     // 0..1023

// ---------- Test parameters ----------
const uint32_t TS_US      = 10000;  // sampling period 10 ms (oversampled on purpose; final Ts chosen in Sprint 2)
const float    U_PCT      = 30.0f;  // test amplitude, % duty (keep low first!)
const float    MAX_PCT    = 50.0f;  // hard output clamp
const uint32_t PRE_MS     = 2000;   // rest before the input is applied
const uint32_t STEP_MS    = 8000;   // step on-time
const uint32_t POST_MS    = 6000;   // rest after the step (free response)
const uint32_t PRBS_BIT_MS = 300;   // PRBS bit time (~ shorter than dominant time constant)
const uint32_t PRBS_MS    = 20000;  // PRBS duration
const uint16_t N_MAX      = 2500;   // 25 s at 10 ms

// ---------- Buffers ----------
static uint32_t bt[N_MAX];
static int16_t  bu[N_MAX];       // u in 0.1 % units
static uint16_t by1[N_MAX], by2[N_MAX];

volatile bool tick = false;
volatile uint32_t tickTime = 0;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
hw_timer_t *tmr;
void IRAM_ATTR onTimer() { tickTime = micros(); tick = true; }
#else
hw_timer_t *tmr;
void IRAM_ATTR onTimer() { tickTime = micros(); tick = true; }
#endif

void setOutput(float pct) {
  if (pct > MAX_PCT) pct = MAX_PCT;
  if (pct < -MAX_PCT) pct = -MAX_PCT;
  digitalWrite(PIN_DIR, pct >= 0 ? HIGH : LOW);
  uint32_t duty = (uint32_t)(fabsf(pct) * ((1 << PWM_BITS) - 1) / 100.0f);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_PWM, duty);
#else
  ledcWrite(0, duty);
#endif
}

uint16_t readAdc(int pin) {          // average 4 reads to cut ADC noise
  uint32_t s = 0;
  for (int i = 0; i < 4; i++) s += analogRead(pin);
  return s >> 2;
}

uint16_t lfsr = 0xACE1;              // 16-bit LFSR for PRBS
int prbsBit() {
  uint16_t b = ((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 5)) & 1;
  lfsr = (lfsr >> 1) | (b << 15);
  return lfsr & 1;
}

void run(bool prbs) {
  uint32_t durMs = prbs ? (PRE_MS + PRBS_MS + POST_MS) : (PRE_MS + STEP_MS + POST_MS);
  uint16_t n = durMs * 1000UL / TS_US;
  if (n > N_MAX) n = N_MAX;
  float u = 0; int bitVal = 1; uint32_t nextBitMs = PRE_MS;
  while (Serial.available()) Serial.read();
  setOutput(0);

  tick = false;
  uint32_t t0 = micros();
  for (uint16_t k = 0; k < n; k++) {
    while (!tick) { if (Serial.available() && Serial.peek() == 'x') { setOutput(0); Serial.println("# ABORTED"); return; } }
    tick = false;
    uint32_t tms = (uint32_t)(k * TS_US / 1000);
    if (prbs) {
      if (tms < PRE_MS || tms >= PRE_MS + PRBS_MS) u = 0;
      else { if (tms >= nextBitMs) { bitVal = prbsBit(); nextBitMs += PRBS_BIT_MS; } u = bitVal ? U_PCT : -U_PCT; }
    } else {
      u = (tms >= PRE_MS && tms < PRE_MS + STEP_MS) ? U_PCT : 0;
    }
    // sample first, then apply: y[k] is measured before u[k] takes effect
    bt[k]  = tickTime - t0;
    by1[k] = readAdc(PIN_JIB);
    by2[k] = readAdc(PIN_CABLE);
    bu[k]  = (int16_t)(u * 10);
    setOutput(u);
  }
  setOutput(0);

  Serial.println("k,t_us,u_pct,y_jib_raw,y_cable_raw");
  for (uint16_t k = 0; k < n; k++) {
    Serial.printf("%u,%lu,%.1f,%u,%u\n", k, (unsigned long)bt[k], bu[k] / 10.0f, by1[k], by2[k]);
  }
  Serial.println("# DONE");
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_DIR, OUTPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_JIB, ADC_11db);     // ~0..3.1 V range
  analogSetPinAttenuation(PIN_CABLE, ADC_11db);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_PWM, PWM_FREQ, PWM_BITS);
  tmr = timerBegin(1000000);                       // 1 MHz
  timerAttachInterrupt(tmr, &onTimer);
  timerAlarm(tmr, TS_US, true, 0);
#else
  ledcSetup(0, PWM_FREQ, PWM_BITS);
  ledcAttachPin(PIN_PWM, 0);
  tmr = timerBegin(0, 80, true);                   // 80 MHz / 80 = 1 MHz
  timerAttachInterrupt(tmr, &onTimer, true);
  timerAlarmWrite(tmr, TS_US, true);
  timerAlarmEnable(tmr);
#endif
  setOutput(0);
  Serial.println("# Ready. 's' step, 'p' PRBS, 'x' abort, 'i' info");
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == 's') run(false);
  else if (c == 'p') run(true);
  else if (c == 'i') Serial.printf("# Ts=%lu us, U=%.1f%%, max=%.1f%%\n", (unsigned long)TS_US, U_PCT, MAX_PCT);
}
