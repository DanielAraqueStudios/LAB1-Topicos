# Grúa Pluma — GUI Python/PyQt6 (alternativa a `maltlab/main.m`)

Puerto a Python de la GUI MATLAB de control en cascada (`lab3/maltlab/main.m`).
Es una **alternativa paralela**, no un remplazo: `main.m` sigue siendo el host
de referencia; esta GUI implementa la misma lógica de control, el mismo
protocolo serial y habla con el mismo firmware (`lab3/fimware/main.ino`), pero
corriendo en Python/PyQt6 en lugar de MATLAB.

## Arquitectura de control (idéntica a `main.m`)

Tres lazos, cuatro familias de controlador cada uno, seleccionables desde un
único dropdown (igual que en `main.m`, que usa la misma familia para los dos
lazos de rotación):

- **Lineal** (carro, VL53L0X): PID / Lugar de Raíces (z) / Oscilaciones
  Muertas / Servosistema, con compensación de zona muerta (`U_DZ`) y
  limitador de velocidad de cambio del comando.
- **Rotación — lazo EXTERNO** (ángulo de base, encoder): mismo set de
  controladores; su salida es una **referencia de ángulo de péndulo**
  (acotada a `PEND_REF_MAX_DEG = 15°`), no un PWM.
- **Rotación — lazo INTERNO** (ángulo de péndulo, potenciómetro): toma la
  referencia que generó el lazo externo y produce el PWM final de rotación,
  con su propia compensación de zona muerta.

Cada lazo tiene banda muerta de error (freno a cero cuando el error y la
velocidad medida son pequeños) y rate-limiting de la referencia, tal como en
`main.m`.

### Dónde vive cada pieza

| Archivo | Contenido |
|---|---|
| `core/constants.py` | Todas las constantes `DEF_*`/`RAMP_*`/`BANDA_*`/`SIM_*` de `main.m`, para los 3 lazos |
| `core/design.py` | Síntesis de controladores: discretización ZOH (`scipy.signal.cont2discrete`), compensador de adelanto discreto vía Tustin, dead-beat (discretización a `Ts*N` + fórmula de polinomio dead-beat), y colocación de polos (fórmula de Ackermann, implementada a mano — no hay equivalente a `place()` del Control System Toolbox en SciPy) para Servosistema (ganancia de estado + observador) |
| `core/controllers.py` | Las ecuaciones de diferencias que corren en tiempo real (PID con anti-windup condicional y derivativo sobre velocidad medida; filtro recursivo del compensador en adelanto; recursión multirate dead-beat; estado+integrador+observador) |
| `core/cascade.py` | Orquestación por muestra: rampas de referencia, error, controlador, rate-limit del comando, banda muerta, compensación de zona muerta — puerto directo de `loopControl()` |
| `core/simulator.py` | Planta simulada (zona muerta + retardo de primer orden para lineal y base; péndulo acoplado al mismo PWM de rotación, sin zona muerta propia) — mismas simplificaciones que `avanzarPlantaSim`/`avanzarPlantaSimRot` en `main.m` |
| `core/protocol.py` | Formato de comando `D<ref>,U<pwm>,R<ref>,V<pwm>` y parseo de las 9 columnas de telemetría (la 9ª, `Encoder_lineal_cuentas`, es opcional — líneas de 8 columnas de firmwares antiguos siguen parseando, con `enc_lin_count=None`) |
| `core/serial_worker.py` | `QObject` que vive en su propio `QThread`; corre el tick de la cascada cada `SAMPLE_TIME` (50 ms), ya sea contra el puerto serial real o contra la planta simulada; emite `log_batch` (lista de tuplas `(direccion, texto)`, misma convención que `LAB 2/gui/core/serial_worker.py`) |
| `ui/` | Ventana principal, tema oscuro, paneles de gráficas (pyqtgraph), panel de log serial (`ui/widgets/log_console.py`) |

## Panel de log y selección de puerto

- **Log serial**: pestaña "Log serial" junto a las gráficas. Muestra cada
  línea con marca de tiempo, prefijo (`>>` tx, `<<` rx, `--` sys) y color por
  dirección, igual que el patrón de `LAB 2/gui/ui/widgets/log_console.py`.
  El worker agrupa varias líneas rx leídas en el mismo tick en un solo
  `log_batch.emit([...])` para no saturar la UI con una señal por línea.
- **Selección de puerto**: combobox poblado con `serial.tools.list_ports.comports()`
  más la opción fija `simulated` (sin hardware), con un botón **Actualizar**
  que vuelve a listar los puertos — mismo patrón que `SIM_PORT` +
  `port_combo` en `LAB 2/gui/ui/main_window.py`.

## Qué es exacto y qué es una simplificación

- **Exacto**: todas las ecuaciones de diferencias de los 4 controladores, para
  los 3 lazos, incluyendo anti-windup condicional, límites de `ucMax`,
  rate-limiting del comando, bandas muertas y compensación de zona muerta.
  Las constantes de diseño (`Udz`, `K`, `wc`, `tau`, ganancias de rampa,
  anchos de banda muerta) son las mismas que en `main.m`.
- **Simplificación conocida**: `place()` de MATLAB (Control System Toolbox)
  no tiene equivalente directo en SciPy; se reimplementó la **fórmula de
  Ackermann** a mano en `core/design.py::ackermann_place`. Para los polos
  usados aquí (una raíz real triple-fold o un par complejo + una raíz real,
  sistemas de 2 y 3 estados, todos controlables/observables) Ackermann y
  `place()` dan el mismo resultado exacto — no es una aproximación, es un
  método clásico equivalente, solo que sensible numéricamente para sistemas
  de orden alto (aquí no es un problema, el mayor es 3×3).
- La planta simulada del péndulo sigue siendo, igual que en `main.m`, una
  aproximación de escritorio (el péndulo no tiene actuador propio; en la
  simulación se excita con el mismo PWM del motor de rotación) — no es un
  modelo físico riguroso del acoplamiento base/péndulo.

## Protocolo serial (verificado contra `lab3/fimware/main.ino`)

```
GUI -> ESP32:   "D<refLinCm>,U<pwmLin>,R<refRotDeg>,V<pwmRot>"  | "START" | "STOP" | "PING"
ESP32 -> GUI:   "Tiempo_s,Referencia_rel_cm,Posicion_rel_cm,PWM_lin,Referencia_rot_deg,Posicion_rot_deg,PWM_rot,Angulo_pendulo_deg,Encoder_lineal_cuentas"
```

9 columnas, 115200 baudios, terminador LF, cada 50 ms. Líneas que empiezan
con `OK:`, `ERROR:`, `PONG`, o que contienen el encabezado, no son telemetría.
La 9ª columna (`Encoder_lineal_cuentas`) es el conteo crudo de cuadratura de
un encoder adicional en el motor lineal (GPIO14/27, por ISR); es solo
informativo — no alimenta ningún lazo de control, la posición lineal sigue
viniendo del VL53L0X (`Posicion_rel_cm`). Se muestra en la GUI junto al
estado del eje lineal. En modo `simulated` (`core/simulator.py`), este campo
es un valor derivado (`round(posición_simulada_cm * 100)`), no una medición
real — placeholder para no romper el formato de 9 columnas.

## Cómo correrla

```bash
cd lab3/gui_py
pip install -r requirements.txt
python main.py
```

En la ventana: elegir puerto (`simulated` para probar sin hardware, o el
puerto COM real), elegir familia de controlador, pulsar **Conectar**. Los
sliders mueven la referencia lineal (cm) y la referencia de ángulo de base
(°) en vivo. Las pestañas de la derecha muestran referencia/posición, PWM y
error para los tres lazos (lineal, base, péndulo).

Con `simulated` no se necesita hardware: corre contra `core/simulator.py` al
mismo `SAMPLE_TIME` (50 ms) que usaría el ESP32 real.

## Autorevisión hecha en esta sesión

- Se releyeron las 4 familias de controlador en `main.m` línea por línea
  contra el puerto en `core/controllers.py`/`core/design.py`: coeficientes,
  signos, orden de los buffers recursivos (`be`/`bu`) y condiciones de
  anti-windup coinciden.
- Se verificó el formato real de telemetría en `main.ino` (8 columnas, orden
  exacto) contra `core/protocol.py::parse_telemetry` — no se asumió, se leyó
  el `Serial.println(...)` del firmware.
- Bug encontrado y corregido durante las pruebas: en NumPy 2.x,
  `float(matriz_1x1)` ya no hace conversión implícita de arrays no
  0-dimensionales; se usó `.item()` en el cálculo de la innovación del
  observador (`core/controllers.py::ServoController.step`). Sin este fix,
  el controlador Servosistema lanzaba `TypeError` en los tres lazos.
- Las 4 familias se probaron corriendo 400 muestras contra
  `core/simulator.py` para los 3 lazos sin errores; el lazo lineal converge
  a la referencia en las 4 familias. Los lazos de rotación (base/péndulo)
  oscilan o saturan con las ganancias por defecto del simulador simplificado
  — mismo comportamiento esperado que con `main.m`, ya que ese acoplamiento
  simulado es una aproximación de escritorio, no calibrada (ver TODOs de
  calibración en `main.m`, sección Rotación).
