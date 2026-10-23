/*
 * =========================================================
 *  GRUA PLUMA (carro + rotación + péndulo) — ESP32 (firmware v4)
 *  Hardware:
 *    - VL53L0X (SDA=21, SCL=22)                -> posición lineal del carro
 *    - Encoder cuadratura (ENC_A=32, ENC_B=33)  -> ángulo de rotación de la base
 *    - Potenciómetro en el pivote (PEND_PIN=34) -> ángulo de péndulo (carga suspendida)
 *    - TB6612FNG canal B (BIN1=18, BIN2=19)     -> motor 2 (traslación del carro)
 *    - TB6612FNG canal A (AIN1=25, AIN2=26, PWMA=27) -> motor 1 (rotación de la base)
 *    - STBY=4 (compartido por ambos canales)
 * =========================================================
 *  Este firmware ya NO compensa fricción ni aplica zona muerta:
 *  ejecuta exactamente el PWM que manda MATLAB. Toda la
 *  inteligencia (zona muerta, rampas, controladores en cascada) está
 *  en MATLAB. El péndulo NO tiene actuador propio: solo se mide y se
 *  reporta, para que MATLAB cierre el lazo interno de la cascada
 *  (ángulo de péndulo) sobre el motor de rotación.
 *
 *  Protocolo (115200, terminador LF):
 *    MATLAB -> ESP32:  "D<refLin>,U<pwmLin>,R<refRot>,V<pwmRot>"
 *                      | "START" | "STOP" | "PING"
 *    ESP32 -> MATLAB:  "Tiempo_s,Referencia_rel_cm,Posicion_rel_cm,PWM_lin,"
 *                      "Referencia_rot_deg,Posicion_rot_deg,PWM_rot,Angulo_pendulo_deg"
 *                      cada 50 ms, más mensajes "OK:..." / "ERROR:..."
 *
 *  Protecciones (independientes por eje, salvo el watchdog que es global):
 *   1. Watchdog: sin paquetes por 300 ms -> freno en ambos motores.
 *   2. Lineal — tope superior: bloquea PWM positivo cerca de FINAL_REAL_CM.
 *   3. Lineal — tope inferior: bloquea PWM negativo por debajo del cero.
 *   4. Lineal — sensor: si hay >10 lecturas inválidas seguidas -> freno lineal.
 *   5. Rotación — tope superior/inferior: bloquea PWM fuera de
 *      [ROT_LIM_INF_DEG, ROT_LIM_SUP_DEG] si ROT_LIMITES_ACTIVOS es true.
 *
 *  Si un motor se mueve al revés, cambia INVERTIR_MOTOR / INVERTIR_MOTOR_ROT
 *  a true (no hace falta cambiar cables). Si el encoder cuenta al revés,
 *  cambia INVERTIR_ENCODER a true. Si el péndulo reporta ángulo al revés,
 *  cambia INVERTIR_PENDULO a true.
 * =========================================================
 */

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

// ── Pines TB6612FNG — canal B (lineal) ────────────────────
const int pinBIN1 = 18;
const int pinBIN2 = 19;
const int pinSTBY = 4;

// ── Pines TB6612FNG — canal A (rotación) ──────────────────
const int pinAIN1 = 25;
const int pinAIN2 = 26;
const int pinPWMA = 27;

// ── Pines encoder cuadratura (rotación) ───────────────────
const int pinEncA = 32;
const int pinEncB = 33;

// ── Pin potenciómetro de péndulo (ADC1, solo lectura) ─────
const int pinPendulo = 34;

#ifndef LED_BUILTIN
  #define LED_BUILTIN 2
#endif

// ── Ajustes — lineal ──────────────────────────────────────
const bool INVERTIR_MOTOR = true;  // true si PWM positivo ACERCA el carrito al sensor
const bool FRENO_EN_CERO  = true;   // true: freno activo con PWM 0 (menos deriva)

const float INICIO_REAL_CM  = 6;   // distancia real del sensor en el cero
const float FINAL_REAL_CM   = 31;  // tope físico
const float MARGEN_SUP_CM   = 1.5;   // bloquea avance a 32.5 cm reales
const float LIM_INF_REL_CM  = -0.8;  // bloquea retroceso bajo este valor relativo

const float FILTRO_ALPHA    = 0.5;   // EMA (más alto = menos retardo)

// ── Ajustes — rotación ─────────────────────────────────────
const bool INVERTIR_MOTOR_ROT = false; // true si PWM positivo invierte el giro esperado
const bool INVERTIR_ENCODER   = false; // true si el conteo del encoder va al revés
const bool FRENO_EN_CERO_ROT  = true;  // true: freno activo con PWM 0 en rotación

// TODO: ajustar con el datasheet del encoder o calibrando a mano
// (girar una vuelta completa y leer encoderCount).
const long  ENC_COUNTS_POR_REV = 1200;  // cuentas por revolución del encoder (placeholder)

// TODO: confirmar límites mecánicos reales de rotación (p. ej. por ausencia
// de anillo colector / cableado). Si no hay límite mecánico, poner
// ROT_LIMITES_ACTIVOS = false.
const bool  ROT_LIMITES_ACTIVOS = true;
const float ROT_LIM_INF_DEG = -170.0;
const float ROT_LIM_SUP_DEG =  170.0;

// ── Ajustes — péndulo (carga suspendida, solo medición) ────
const bool INVERTIR_PENDULO = false;
// TODO: calibrar con el potenciómetro real: leer analogRead() en reposo
// (0°, péndulo colgando vertical) y en una desviación conocida para
// ajustar PEND_ADC_CERO y PEND_GRADOS_POR_CUENTA.
const int   PEND_ADC_CERO          = 2048; // lectura ADC (0-4095) en péndulo vertical
const float PEND_GRADOS_POR_CUENTA = 0.0879; // ~ (180/4095)*2, placeholder
const float PEND_FILTRO_ALPHA      = 0.4;    // EMA del ángulo de péndulo

const unsigned long WD_TIMEOUT_MS         = 300;
const unsigned long PERIODO_TELEMETRIA_MS = 50;

const int PWM_MAX  = 255;
const int PWM_FREQ = 1000;
const int PWM_RES  = 8;

// ── Estado — lineal ───────────────────────────────────────
float distRealActual   = 0.0;
float distRelFiltrada  = 0.0;
bool  filtroInicializado = false;
int   lecturasInvalidas  = 0;

float refEco           = 0.0;   // solo se devuelve en la telemetría
int   pwmCmd           = 0;     // último PWM pedido por MATLAB
int   ultimoPWM        = 0;     // PWM realmente aplicado

// ── Estado — rotación ──────────────────────────────────────
volatile long encoderCount = 0;
float anguloRotActual  = 0.0;   // grados, relativo al arranque

float refEcoRot        = 0.0;   // solo se devuelve en la telemetría
int   pwmCmdRot        = 0;     // último PWM de rotación pedido por MATLAB
int   ultimoPWMRot     = 0;     // PWM de rotación realmente aplicado

// ── Estado — péndulo (solo medición, sin actuador propio) ──
float anguloPenduloActual   = 0.0;   // grados, filtrado
bool  filtroPenduloInicializado = false;

// ── Estado compartido ───────────────────────────────────────
bool  sistemaActivo    = false;

unsigned long tInicio        = 0;
unsigned long tUltimoEnvio   = 0;
unsigned long tUltimoPaquete = 0;

String bufferSerial = "";

// ── PWM compatible con núcleo ESP32 2.x y 3.x ─────────────
// Canales 0/1 = motor lineal (BIN1/BIN2); canal 2 = motor rotación (PWMA)
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  void pwmInit() {
    ledcAttach(pinBIN1, PWM_FREQ, PWM_RES);
    ledcAttach(pinBIN2, PWM_FREQ, PWM_RES);
    ledcAttach(pinPWMA, PWM_FREQ, PWM_RES);
  }
  void pwmWrite(int canal, int duty) {
    int pin = (canal == 0) ? pinBIN1 : (canal == 1) ? pinBIN2 : pinPWMA;
    ledcWrite(pin, duty);
  }
#else
  void pwmInit() {
    ledcSetup(0, PWM_FREQ, PWM_RES); ledcAttachPin(pinBIN1, 0);
    ledcSetup(1, PWM_FREQ, PWM_RES); ledcAttachPin(pinBIN2, 1);
    ledcSetup(2, PWM_FREQ, PWM_RES); ledcAttachPin(pinPWMA, 2);
  }
  void pwmWrite(int canal, int duty) {
    ledcWrite(canal, duty);
  }
#endif

// ── ISR encoder cuadratura ─────────────────────────────────
void IRAM_ATTR isrEncA() {
  bool a = digitalRead(pinEncA);
  bool b = digitalRead(pinEncB);
  bool adelante = (a == b);
  if (INVERTIR_ENCODER) adelante = !adelante;
  if (adelante) encoderCount++;
  else encoderCount--;
}

// ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setClock(400000);

  pinMode(pinSTBY, OUTPUT);
  digitalWrite(pinSTBY, HIGH);
  pinMode(pinAIN1, OUTPUT);
  pinMode(pinAIN2, OUTPUT);
  pwmInit();
  detenerMotor();
  detenerMotorRot();

  pinMode(pinEncA, INPUT_PULLUP);
  pinMode(pinEncB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinEncA), isrEncA, CHANGE);

  analogReadResolution(12);
  pinMode(pinPendulo, INPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH); delay(100);
    digitalWrite(LED_BUILTIN, LOW);  delay(100);
  }

  if (!sensor.init()) {
    while (1) {
      Serial.println("ERROR:VL53L0X_NO_ENCONTRADO");
      delay(1000);
    }
  }
  sensor.setTimeout(100);
  sensor.setMeasurementTimingBudget(20000);   // 20 ms por medida
  sensor.startContinuous();

  Serial.println("Tiempo_s,Referencia_rel_cm,Posicion_rel_cm,PWM_lin,Referencia_rot_deg,Posicion_rot_deg,PWM_rot,Angulo_pendulo_deg");
  tInicio = tUltimoEnvio = tUltimoPaquete = millis();
}

// ─────────────────────────────────────────────────────────
void loop() {
  leerSerial();
  leerSensor();
  leerEncoder();
  leerPendulo();

  unsigned long ahora = millis();

  // 1. Watchdog de comunicación (global, afecta a ambos ejes)
  if (sistemaActivo && (ahora - tUltimoPaquete > WD_TIMEOUT_MS)) {
    sistemaActivo = false;
    pwmCmd = 0;
    pwmCmdRot = 0;
    avisar("ERROR:WATCHDOG");
  }

  // 2. PWM efectivo lineal con protecciones
  int pwm = sistemaActivo ? pwmCmd : 0;

  if (lecturasInvalidas > 10) {
    pwm = 0;
    avisar("ERROR:SENSOR");
  }
  if (pwm > 0 && distRealActual >= (FINAL_REAL_CM - MARGEN_SUP_CM)) {
    pwm = 0;
    avisar("ERROR:TOPE_SUP");
  }
  if (pwm < 0 && (distRealActual - INICIO_REAL_CM) <= LIM_INF_REL_CM) {
    pwm = 0;
    avisar("ERROR:TOPE_INF");
  }

  ultimoPWM = pwm;
  aplicarPWM(pwm);

  // 3. PWM efectivo de rotación con protecciones
  int pwmRot = sistemaActivo ? pwmCmdRot : 0;

  if (ROT_LIMITES_ACTIVOS) {
    if (pwmRot > 0 && anguloRotActual >= ROT_LIM_SUP_DEG) {
      pwmRot = 0;
      avisar("ERROR:ROT_TOPE_SUP");
    }
    if (pwmRot < 0 && anguloRotActual <= ROT_LIM_INF_DEG) {
      pwmRot = 0;
      avisar("ERROR:ROT_TOPE_INF");
    }
  }

  ultimoPWMRot = pwmRot;
  aplicarPWMRot(pwmRot);

  // 4. Telemetría a cadencia fija
  if (ahora - tUltimoEnvio >= PERIODO_TELEMETRIA_MS) {
    tUltimoEnvio += PERIODO_TELEMETRIA_MS;
    if (ahora - tUltimoEnvio > PERIODO_TELEMETRIA_MS) tUltimoEnvio = ahora;
    float t_s = (ahora - tInicio) / 1000.0;
    Serial.print(t_s, 3);              Serial.print(",");
    Serial.print(refEco, 2);           Serial.print(",");
    Serial.print(distRelFiltrada, 2);  Serial.print(",");
    Serial.print(ultimoPWM);           Serial.print(",");
    Serial.print(refEcoRot, 2);        Serial.print(",");
    Serial.print(anguloRotActual, 2);  Serial.print(",");
    Serial.print(ultimoPWMRot);        Serial.print(",");
    Serial.println(anguloPenduloActual, 2);
  }

  // 5. LED
  digitalWrite(LED_BUILTIN, sistemaActivo ? ((ahora / 300) % 2) : LOW);
}

// ─────────────────────────────────────────────────────────
void leerSensor() {
  float med = sensor.readRangeContinuousMillimeters() / 10.0;
  if (!sensor.timeoutOccurred() && med > 0.5 && med < 400.0) {
    lecturasInvalidas = 0;
    distRealActual = med;
    float rel = med - INICIO_REAL_CM;      // puede ser negativo
    if (!filtroInicializado) {
      distRelFiltrada = rel;
      filtroInicializado = true;
    } else {
      distRelFiltrada = FILTRO_ALPHA * rel + (1.0 - FILTRO_ALPHA) * distRelFiltrada;
    }
  } else {
    lecturasInvalidas++;
  }
}

void leerEncoder() {
  noInterrupts();
  long cuentas = encoderCount;
  interrupts();
  anguloRotActual = (360.0 * cuentas) / (float)ENC_COUNTS_POR_REV;
}

// Péndulo: solo medición (no tiene actuador propio). El ángulo se mide
// relativo a PEND_ADC_CERO (péndulo colgando en reposo = 0°).
void leerPendulo() {
  int adc = analogRead(pinPendulo);
  float grados = (adc - PEND_ADC_CERO) * PEND_GRADOS_POR_CUENTA;
  if (INVERTIR_PENDULO) grados = -grados;
  if (!filtroPenduloInicializado) {
    anguloPenduloActual = grados;
    filtroPenduloInicializado = true;
  } else {
    anguloPenduloActual = PEND_FILTRO_ALPHA * grados + (1.0 - PEND_FILTRO_ALPHA) * anguloPenduloActual;
  }
}

// ─────────────────────────────────────────────────────────
void leerSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      procesarComando(bufferSerial);
      bufferSerial = "";
    } else if (c != '\r') {
      if (bufferSerial.length() < 60) bufferSerial += c;
      else bufferSerial = "";
    }
  }
}

void procesarComando(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  tUltimoPaquete = millis();

  if (cmd == "STOP") {
    sistemaActivo = false;
    pwmCmd = 0;
    pwmCmdRot = 0;
    detenerMotor();
    detenerMotorRot();
    Serial.println("OK:STOP");
    return;
  }
  if (cmd == "START") {
    sistemaActivo = true;
    pwmCmd = 0;
    pwmCmdRot = 0;
    Serial.println("OK:START");
    return;
  }
  if (cmd == "PING") {
    Serial.println("PONG");
    return;
  }

  // "D<refLin>,U<pwmLin>,R<refRot>,V<pwmRot>"
  // Los campos R/V son opcionales, por compatibilidad hacia atrás con un
  // host que todavía solo controle el eje lineal.
  if (cmd.charAt(0) == 'D') {
    int coma = cmd.indexOf(',');
    int posU = cmd.indexOf('U', coma);
    if (coma < 0 || posU < 0) return;
    refEco = cmd.substring(1, coma).toFloat();

    int posComaR = cmd.indexOf(',', posU);
    String tramoPwmLin = (posComaR < 0) ? cmd.substring(posU + 1) : cmd.substring(posU + 1, posComaR);
    pwmCmd = constrain(tramoPwmLin.toInt(), -PWM_MAX, PWM_MAX);

    if (posComaR >= 0) {
      int posR = cmd.indexOf('R', posComaR);
      int posComaV = cmd.indexOf(',', posR);
      int posV = cmd.indexOf('V', posComaV);
      if (posR >= 0 && posComaV >= 0 && posV >= 0) {
        refEcoRot = cmd.substring(posR + 1, posComaV).toFloat();
        pwmCmdRot = constrain(cmd.substring(posV + 1).toInt(), -PWM_MAX, PWM_MAX);
      }
    }

    sistemaActivo = true;
  }
}

// ─────────────────────────────────────────────────────────
void avisar(const char* msg) {
  static unsigned long t = 0;
  if (millis() - t > 1000) {
    t = millis();
    Serial.println(msg);
  }
}

void aplicarPWM(int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  if (INVERTIR_MOTOR) pwm = -pwm;
  if (pwm > 0) {
    pwmWrite(0, pwm);
    pwmWrite(1, 0);
  } else if (pwm < 0) {
    pwmWrite(0, 0);
    pwmWrite(1, -pwm);
  } else {
    frenar();
  }
}

void frenar() {
  if (FRENO_EN_CERO) {
    pwmWrite(0, PWM_MAX);
    pwmWrite(1, PWM_MAX);
  } else {
    pwmWrite(0, 0);
    pwmWrite(1, 0);
  }
}

void detenerMotor() {
  frenar();
  ultimoPWM = 0;
}

// ── Rotación — canal A (AIN1/AIN2/PWMA) ───────────────────
void aplicarPWMRot(int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  if (INVERTIR_MOTOR_ROT) pwm = -pwm;
  if (pwm > 0) {
    digitalWrite(pinAIN1, HIGH);
    digitalWrite(pinAIN2, LOW);
    pwmWrite(2, pwm);
  } else if (pwm < 0) {
    digitalWrite(pinAIN1, LOW);
    digitalWrite(pinAIN2, HIGH);
    pwmWrite(2, -pwm);
  } else {
    frenarRot();
  }
}

void frenarRot() {
  if (FRENO_EN_CERO_ROT) {
    digitalWrite(pinAIN1, HIGH);
    digitalWrite(pinAIN2, HIGH);
    pwmWrite(2, PWM_MAX);
  } else {
    digitalWrite(pinAIN1, LOW);
    digitalWrite(pinAIN2, LOW);
    pwmWrite(2, 0);
  }
}

void detenerMotorRot() {
  frenarRot();
  ultimoPWMRot = 0;
}
