#include <Arduino.h>

const int pinAIN1 = 25;
const int pinAIN2 = 26;
const int pinPWMA = 27;
const int pinSTBY = 4;

const int pinEncA = 32;
const int pinEncB = 33;

const int PWM_FREQ_HZ  = 2000;
const int PWM_RES_BITS = 8;

const uint32_t TS_US   = 10000;
const float    U_PCT   = 30.0f;
const float    MAX_PCT = 60.0f;
const uint32_t PRE_MS  = 1000;
const uint32_t STEP_MS = 4000;
const uint32_t POST_MS = 4000;
const uint16_t N_MAX   = 2500;

static uint32_t bt[N_MAX];
static int16_t  bu[N_MAX];
static int32_t  benc[N_MAX];

volatile long encoderCount = 0;
volatile bool tick = false;
volatile uint32_t tickTime = 0;

hw_timer_t *tmr;

void IRAM_ATTR isrEncA() {
  bool a = digitalRead(pinEncA);
  bool b = digitalRead(pinEncB);
  if (a == b) encoderCount++;
  else encoderCount--;
}

void IRAM_ATTR onTimer() {
  tickTime = micros();
  tick = true;
}

void aplicarPWM(float pct) {
  if (pct > MAX_PCT) pct = MAX_PCT;
  if (pct < -MAX_PCT) pct = -MAX_PCT;
  digitalWrite(pinAIN1, pct >= 0 ? HIGH : LOW);
  digitalWrite(pinAIN2, pct >= 0 ? LOW : HIGH);
  uint32_t duty = (uint32_t)(fabsf(pct) * ((1 << PWM_RES_BITS) - 1) / 100.0f);
#if defined(ESP_ARDUINO_VERSION) && ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcWrite(pinPWMA, duty);
#else
  ledcWrite(0, duty);
#endif
}

void imprimirAyuda() {
  Serial.println(F("=== Prueba motor de rotacion (TB6612FNG) ==="));
  Serial.println(F("Comandos manuales:"));
  Serial.println(F("  F<pwm>  -> adelante, pwm 0-255 (ej: F150)"));
  Serial.println(F("  R<pwm>  -> reversa,  pwm 0-255 (ej: R150)"));
  Serial.println(F("  S       -> stop (coast)"));
  Serial.println(F("  B       -> brake"));
  Serial.println(F("  E       -> imprime conteo actual del encoder"));
  Serial.println(F("  ?       -> esta ayuda"));
  Serial.println(F("Caracterizacion (para funcion de transferencia):"));
  Serial.println(F("  T       -> corre prueba de escalon y vuelca CSV k,t_us,u_pct,enc_count"));
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
  digitalWrite(pinAIN1, LOW);
  digitalWrite(pinAIN2, LOW);
  aplicarPWM(0);
  Serial.println(F("OK dir=STOP (coast)"));
}

void aplicarBrake() {
  digitalWrite(pinAIN1, HIGH);
  digitalWrite(pinAIN2, HIGH);
  aplicarPWM(0);
  Serial.println(F("OK dir=BRAKE"));
}

void correrEscalon() {
  uint32_t durMs = PRE_MS + STEP_MS + POST_MS;
  uint16_t n = durMs * 1000UL / TS_US;
  if (n > N_MAX) n = N_MAX;

  while (Serial.available()) Serial.read();
  aplicarStop();
  encoderCount = 0;

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

    bt[k]   = tickTime - t0;
    benc[k] = encoderCount;
    bu[k]   = (int16_t)(u * 10);

    aplicarPWM(u);
  }
  aplicarStop();

  Serial.println(F("k,t_us,u_pct,enc_count"));
  for (uint16_t k = 0; k < n; k++) {
    Serial.printf("%u,%lu,%.1f,%ld\n", k, (unsigned long)bt[k], bu[k] / 10.0f, (long)benc[k]);
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
    case 'E':
    case 'e':
      Serial.print(F("enc_count="));
      Serial.println(encoderCount);
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

  pinMode(pinAIN1, OUTPUT);
  pinMode(pinAIN2, OUTPUT);
  pinMode(pinSTBY, OUTPUT);
  digitalWrite(pinSTBY, HIGH);

  pinMode(pinEncA, INPUT_PULLUP);
  pinMode(pinEncB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinEncA), isrEncA, CHANGE);

#if defined(ESP_ARDUINO_VERSION) && ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcAttach(pinPWMA, PWM_FREQ_HZ, PWM_RES_BITS);
  tmr = timerBegin(1000000);
  timerAttachInterrupt(tmr, &onTimer);
  timerAlarm(tmr, TS_US, true, 0);
#else
  ledcSetup(0, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(pinPWMA, 0);
  tmr = timerBegin(0, 80, true);
  timerAttachInterrupt(tmr, &onTimer, true);
  timerAlarmWrite(tmr, TS_US, true);
  timerAlarmEnable(tmr);
#endif

  aplicarStop();
  imprimirAyuda();
}

void loop() {
  if (Serial.available()) {
    String linea = Serial.readStringUntil('\n');
    procesarLinea(linea);
  }
}
