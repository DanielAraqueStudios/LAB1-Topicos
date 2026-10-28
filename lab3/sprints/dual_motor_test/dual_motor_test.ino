#include <Wire.h>
#include <VL53L0X.h>
#include <Arduino.h>

VL53L0X sensor;

const int pinBIN1 = 18;
const int pinBIN2 = 19;
const int pinAIN1 = 25;
const int pinAIN2 = 26;
const int pinPWMA = 27;
const int pinSTBY = 4;

const int pinEncA = 32;
const int pinEncB = 33;

const int PWM_FREQ_LIN_HZ = 1000;
const int PWM_FREQ_ROT_HZ = 2000;
const int PWM_RES_BITS    = 8;

const float MAX_PCT = 60.0f;

volatile long encoderCount = 0;

void IRAM_ATTR isrEncA() {
  bool a = digitalRead(pinEncA);
  bool b = digitalRead(pinEncB);
  if (a == b) encoderCount++;
  else encoderCount--;
}

void aplicarPWMLineal(float pct) {
  if (pct > MAX_PCT) pct = MAX_PCT;
  if (pct < -MAX_PCT) pct = -MAX_PCT;
  digitalWrite(pinBIN1, pct >= 0 ? HIGH : LOW);
  digitalWrite(pinBIN2, pct >= 0 ? LOW : HIGH);
  uint32_t duty = (uint32_t)(fabsf(pct) * ((1 << PWM_RES_BITS) - 1) / 100.0f);
#if defined(ESP_ARDUINO_VERSION) && ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcWrite(pinBIN1, pct >= 0 ? duty : 0);
  ledcWrite(pinBIN2, pct >= 0 ? 0 : duty);
#else
  ledcWrite(0, pct >= 0 ? duty : 0);
  ledcWrite(1, pct >= 0 ? 0 : duty);
#endif
}

void aplicarPWMRotacion(float pct) {
  if (pct > MAX_PCT) pct = MAX_PCT;
  if (pct < -MAX_PCT) pct = -MAX_PCT;
  digitalWrite(pinAIN1, pct >= 0 ? HIGH : LOW);
  digitalWrite(pinAIN2, pct >= 0 ? LOW : HIGH);
  uint32_t duty = (uint32_t)(fabsf(pct) * ((1 << PWM_RES_BITS) - 1) / 100.0f);
#if defined(ESP_ARDUINO_VERSION) && ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcWrite(pinPWMA, duty);
#else
  ledcWrite(2, duty);
#endif
}

int leerDistanciaMm() {
  int med = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred()) return -1;
  return med;
}

void imprimirAyuda() {
  Serial.println(F("=== Prueba dual (TB6612FNG compartido, lineal + rotacion) ==="));
  Serial.println(F("Comandos lineal:"));
  Serial.println(F("  LF<pwm> -> avanza, pwm 0-255 (ej: LF150)"));
  Serial.println(F("  LR<pwm> -> retrocede, pwm 0-255 (ej: LR150)"));
  Serial.println(F("  LS      -> stop lineal (coast)"));
  Serial.println(F("  LB      -> brake lineal"));
  Serial.println(F("Comandos rotacion:"));
  Serial.println(F("  RF<pwm> -> adelante, pwm 0-255 (ej: RF150)"));
  Serial.println(F("  RR<pwm> -> reversa, pwm 0-255 (ej: RR150)"));
  Serial.println(F("  RS      -> stop rotacion (coast)"));
  Serial.println(F("  RB      -> brake rotacion"));
  Serial.println(F("Comando simultaneo:"));
  Serial.println(F("  M<pwmLin>,<pwmRot> -> aplica ambos a la vez (ej: M150,-100)"));
  Serial.println(F("Comandos globales:"));
  Serial.println(F("  S       -> stop global (coast, ambos motores)"));
  Serial.println(F("  B       -> brake global (ambos motores)"));
  Serial.println(F("  D       -> imprime distancia actual (mm)"));
  Serial.println(F("  E       -> imprime conteo actual del encoder"));
  Serial.println(F("  ?       -> esta ayuda"));
}

void linealAdelante(int pwm) {
  pwm = constrain(pwm, 0, 255);
  aplicarPWMLineal(pwm * 100.0f / 255.0f);
  Serial.print(F("OK lin dir=FWD pwm="));
  Serial.println(pwm);
}

void linealReversa(int pwm) {
  pwm = constrain(pwm, 0, 255);
  aplicarPWMLineal(-pwm * 100.0f / 255.0f);
  Serial.print(F("OK lin dir=REV pwm="));
  Serial.println(pwm);
}

void linealStop() {
  digitalWrite(pinBIN1, LOW);
  digitalWrite(pinBIN2, LOW);
  aplicarPWMLineal(0);
  Serial.println(F("OK lin dir=STOP (coast)"));
}

void linealBrake() {
  digitalWrite(pinBIN1, HIGH);
  digitalWrite(pinBIN2, HIGH);
  aplicarPWMLineal(0);
  Serial.println(F("OK lin dir=BRAKE"));
}

void rotacionAdelante(int pwm) {
  pwm = constrain(pwm, 0, 255);
  aplicarPWMRotacion(pwm * 100.0f / 255.0f);
  Serial.print(F("OK rot dir=FWD pwm="));
  Serial.println(pwm);
}

void rotacionReversa(int pwm) {
  pwm = constrain(pwm, 0, 255);
  aplicarPWMRotacion(-pwm * 100.0f / 255.0f);
  Serial.print(F("OK rot dir=REV pwm="));
  Serial.println(pwm);
}

void rotacionStop() {
  digitalWrite(pinAIN1, LOW);
  digitalWrite(pinAIN2, LOW);
  aplicarPWMRotacion(0);
  Serial.println(F("OK rot dir=STOP (coast)"));
}

void rotacionBrake() {
  digitalWrite(pinAIN1, HIGH);
  digitalWrite(pinAIN2, HIGH);
  aplicarPWMRotacion(0);
  Serial.println(F("OK rot dir=BRAKE"));
}

void stopGlobal() {
  linealStop();
  rotacionStop();
  Serial.println(F("OK global STOP (coast)"));
}

void brakeGlobal() {
  linealBrake();
  rotacionBrake();
  Serial.println(F("OK global BRAKE"));
}

void comandoSimultaneo(String resto) {
  int coma = resto.indexOf(',');
  if (coma < 0) {
    Serial.println(F("ERR formato esperado M<pwmLin>,<pwmRot>"));
    return;
  }
  int pwmLin = constrain(resto.substring(0, coma).toInt(), -255, 255);
  int pwmRot = constrain(resto.substring(coma + 1).toInt(), -255, 255);

  aplicarPWMLineal(pwmLin * 100.0f / 255.0f);
  aplicarPWMRotacion(pwmRot * 100.0f / 255.0f);

  Serial.print(F("OK simult pwmLin="));
  Serial.print(pwmLin);
  Serial.print(F(" pwmRot="));
  Serial.println(pwmRot);
}

void procesarLinea(String linea) {
  linea.trim();
  if (linea.length() == 0) return;

  char c = linea.charAt(0);

  if (c == 'L' || c == 'l') {
    char sub = linea.length() > 1 ? linea.charAt(1) : 0;
    String resto = linea.length() > 2 ? linea.substring(2) : "";
    switch (sub) {
      case 'F': case 'f': linealAdelante(resto.toInt()); return;
      case 'R': case 'r': linealReversa(resto.toInt());  return;
      case 'S': case 's': linealStop();                  return;
      case 'B': case 'b': linealBrake();                 return;
      default: break;
    }
  } else if (c == 'R' || c == 'r') {
    char sub = linea.length() > 1 ? linea.charAt(1) : 0;
    String resto = linea.length() > 2 ? linea.substring(2) : "";
    switch (sub) {
      case 'F': case 'f': rotacionAdelante(resto.toInt()); return;
      case 'R': case 'r': rotacionReversa(resto.toInt());  return;
      case 'S': case 's': rotacionStop();                  return;
      case 'B': case 'b': rotacionBrake();                 return;
      default: break;
    }
  } else if (c == 'M' || c == 'm') {
    comandoSimultaneo(linea.substring(1));
    return;
  } else if (c == 'S' || c == 's') {
    stopGlobal();
    return;
  } else if (c == 'B' || c == 'b') {
    brakeGlobal();
    return;
  } else if (c == 'D' || c == 'd') {
    Serial.print(F("dist_mm="));
    Serial.println(leerDistanciaMm());
    return;
  } else if (c == 'E' || c == 'e') {
    Serial.print(F("enc_count="));
    Serial.println(encoderCount);
    return;
  } else if (c == '?') {
    imprimirAyuda();
    return;
  }

  Serial.print(F("ERR comando desconocido: "));
  Serial.println(linea);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(pinBIN1, OUTPUT);
  pinMode(pinBIN2, OUTPUT);
  pinMode(pinAIN1, OUTPUT);
  pinMode(pinAIN2, OUTPUT);
  pinMode(pinSTBY, OUTPUT);
  digitalWrite(pinSTBY, HIGH);

  pinMode(pinEncA, INPUT_PULLUP);
  pinMode(pinEncB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinEncA), isrEncA, CHANGE);

#if defined(ESP_ARDUINO_VERSION) && ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcAttach(pinBIN1, PWM_FREQ_LIN_HZ, PWM_RES_BITS);
  ledcAttach(pinBIN2, PWM_FREQ_LIN_HZ, PWM_RES_BITS);
  ledcAttach(pinPWMA, PWM_FREQ_ROT_HZ, PWM_RES_BITS);
#else
  ledcSetup(0, PWM_FREQ_LIN_HZ, PWM_RES_BITS);
  ledcAttachPin(pinBIN1, 0);
  ledcSetup(1, PWM_FREQ_LIN_HZ, PWM_RES_BITS);
  ledcAttachPin(pinBIN2, 1);
  ledcSetup(2, PWM_FREQ_ROT_HZ, PWM_RES_BITS);
  ledcAttachPin(pinPWMA, 2);
#endif

  Wire.begin(21, 22);
  Wire.setClock(400000);
  if (!sensor.init()) {
    Serial.println(F("ERR VL53L0X no detectado"));
  }
  sensor.setTimeout(100);
  sensor.setMeasurementTimingBudget(20000);
  sensor.startContinuous();

  stopGlobal();
  imprimirAyuda();
}

void loop() {
  if (Serial.available()) {
    String linea = Serial.readStringUntil('\n');
    procesarLinea(linea);
  }
}
