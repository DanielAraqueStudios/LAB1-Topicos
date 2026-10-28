#include <Wire.h>
#include <VL53L0X.h>
#include <Arduino.h>

VL53L0X sensor;

const int pinIN1 = 18;
const int pinIN2 = 19;
const int pinENA = 23;

const int pinEncLinA = 14;
const int pinEncLinB = 27;

const bool INVERTIR_MOTOR = true;

const int PWM_FREQ_HZ  = 1000;
const int PWM_RES_BITS = 8;

const uint32_t TS_US   = 10000;
const float    U_PCT   = 30.0f;
const float    MAX_PCT = 100.0f;
const uint32_t PRE_MS  = 1000;
const uint32_t STEP_MS = 4000;
const uint32_t POST_MS = 4000;
const uint16_t N_MAX   = 2500;

static uint32_t bt[N_MAX];
static int16_t  bu[N_MAX];
static int16_t  bdist[N_MAX];
static int32_t  bencLin[N_MAX];

volatile bool tick = false;
volatile uint32_t tickTime = 0;
volatile long encoderLinCount = 0;

hw_timer_t *tmr;

void IRAM_ATTR onTimer() {
  tickTime = micros();
  tick = true;
}

void IRAM_ATTR isrEncLinA() {
  bool a = digitalRead(pinEncLinA);
  bool b = digitalRead(pinEncLinB);
  if (a == b) encoderLinCount++;
  else encoderLinCount--;
}

void aplicarPWM(float pct) {
  if (INVERTIR_MOTOR) pct = -pct;
  if (pct > MAX_PCT) pct = MAX_PCT;
  if (pct < -MAX_PCT) pct = -MAX_PCT;
  digitalWrite(pinIN1, pct >= 0 ? HIGH : LOW);
  digitalWrite(pinIN2, pct >= 0 ? LOW : HIGH);
  uint32_t duty = (uint32_t)(fabsf(pct) * ((1 << PWM_RES_BITS) - 1) / 100.0f);
#if defined(ESP_ARDUINO_VERSION) && ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcWrite(pinENA, duty);
#else
  ledcWrite(0, duty);
#endif
}

int leerDistanciaMm() {
  int med = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred()) return -1;
  return med;
}

void imprimirAyuda() {
  Serial.println(F("=== Prueba motor lineal (L298N) ==="));
  Serial.println(F("Comandos manuales:"));
  Serial.println(F("  F<pwm>  -> avanza, pwm 0-255 (ej: F150)"));
  Serial.println(F("  R<pwm>  -> retrocede, pwm 0-255 (ej: R150)"));
  Serial.println(F("  S       -> stop (coast)"));
  Serial.println(F("  B       -> brake"));
  Serial.println(F("  D       -> imprime distancia (mm) y conteo del encoder lineal"));
  Serial.println(F("  ?       -> esta ayuda"));
  Serial.println(F("Caracterizacion (para funcion de transferencia):"));
  Serial.println(F("  T       -> corre prueba de escalon y vuelca CSV k,t_us,u_pct,dist_mm,enc_lin_count"));
  Serial.println(F("  X       -> aborta la prueba en curso"));
  Serial.println(F("  I       -> imprime parametros de la prueba"));
}

void aplicarAdelante(int pwm) {
  pwm = constrain(pwm, 0, 255);
  aplicarPWM(pwm * 100.0f / 255.0f);
  Serial.print(F("OK dir=FWD pwm="));
  Serial.println(pwm);
}

void aplicarReversa(int pwm) {
  pwm = constrain(pwm, 0, 255);
  aplicarPWM(-pwm * 100.0f / 255.0f);
  Serial.print(F("OK dir=REV pwm="));
  Serial.println(pwm);
}

void aplicarStop() {
  digitalWrite(pinIN1, LOW);
  digitalWrite(pinIN2, LOW);
  aplicarPWM(0);
  Serial.println(F("OK dir=STOP (coast)"));
}

void aplicarBrake() {
  digitalWrite(pinIN1, HIGH);
  digitalWrite(pinIN2, HIGH);
  aplicarPWM(0);
  Serial.println(F("OK dir=BRAKE"));
}

void correrEscalon() {
  uint32_t durMs = PRE_MS + STEP_MS + POST_MS;
  uint16_t n = durMs * 1000UL / TS_US;
  if (n > N_MAX) n = N_MAX;

  while (Serial.available()) Serial.read();
  aplicarStop();
  encoderLinCount = 0;

  tick = false;
  uint32_t t0 = micros();
  for (uint16_t k = 0; k < n; k++) {
    while (!tick) {
      if (Serial.available() && (Serial.peek() == 'x' || Serial.peek() == 'X')) {
        Serial.read();
        aplicarStop();
        Serial.println(F("# ABORTED"));
        return;
      }
    }
    tick = false;

    uint32_t tms = (uint32_t)(k * TS_US / 1000);
    float u = (tms >= PRE_MS && tms < PRE_MS + STEP_MS) ? U_PCT : 0.0f;

    bt[k]      = tickTime - t0;
    bdist[k]   = (int16_t)leerDistanciaMm();
    bencLin[k] = encoderLinCount;
    bu[k]      = (int16_t)(u * 10);

    aplicarPWM(u);
  }
  aplicarStop();

  Serial.println(F("k,t_us,u_pct,dist_mm,enc_lin_count"));
  for (uint16_t k = 0; k < n; k++) {
    Serial.printf("%u,%lu,%.1f,%d,%ld\n", k, (unsigned long)bt[k], bu[k] / 10.0f, bdist[k], (long)bencLin[k]);
  }
  Serial.println(F("# DONE"));
}

void procesarLinea(String linea) {
  linea.trim();
  if (linea.length() == 0) return;

  char c = linea.charAt(0);
  String resto = linea.substring(1);

  switch (c) {
    case 'F':
    case 'f':
      aplicarAdelante(resto.toInt());
      break;
    case 'R':
    case 'r':
      aplicarReversa(resto.toInt());
      break;
    case 'S':
    case 's':
      aplicarStop();
      break;
    case 'B':
    case 'b':
      aplicarBrake();
      break;
    case 'D':
    case 'd':
      Serial.print(F("dist_mm="));
      Serial.print(leerDistanciaMm());
      Serial.print(F(" enc_lin_count="));
      Serial.println(encoderLinCount);
      break;
    case 'T':
    case 't':
      correrEscalon();
      break;
    case 'I':
    case 'i':
      Serial.printf("# Ts=%lu us, U=%.1f%%, max=%.1f%%, pre=%lums, step=%lums, post=%lums\n",
                    (unsigned long)TS_US, U_PCT, MAX_PCT,
                    (unsigned long)PRE_MS, (unsigned long)STEP_MS, (unsigned long)POST_MS);
      break;
    case '?':
      imprimirAyuda();
      break;
    default:
      Serial.print(F("ERR comando desconocido: "));
      Serial.println(linea);
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(pinIN1, OUTPUT);
  pinMode(pinIN2, OUTPUT);

  pinMode(pinEncLinA, INPUT_PULLUP);
  pinMode(pinEncLinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinEncLinA), isrEncLinA, CHANGE);

#if defined(ESP_ARDUINO_VERSION) && ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcAttach(pinENA, PWM_FREQ_HZ, PWM_RES_BITS);
#else
  ledcSetup(0, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(pinENA, 0);
#endif

  tmr = timerBegin(1000000);
  timerAttachInterrupt(tmr, &onTimer);
  timerAlarm(tmr, TS_US, true, 0);

  Wire.begin(21, 22);
  Wire.setClock(400000);
  if (!sensor.init()) {
    Serial.println(F("ERR VL53L0X no detectado"));
  }
  sensor.setTimeout(100);
  sensor.setMeasurementTimingBudget(20000);
  sensor.startContinuous();

  aplicarStop();
  imprimirAyuda();
}

void loop() {
  if (Serial.available()) {
    String linea = Serial.readStringUntil('\n');
    procesarLinea(linea);
  }
}
