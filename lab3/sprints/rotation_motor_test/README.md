# Prueba aislada — motor de rotación (DRV8833)

Sprint de validación de hardware y **caracterización** del motor de
rotación. No es el firmware final (ese vive en `lab3/fimware/`): este
sketch sirve para (1) confirmar que el driver mueve el motor en ambos
sentidos, y (2) una vez que eso funciona, correr una prueba de escalón y
volcar los datos (tiempo, PWM aplicado, conteo del encoder) por serial para
poder identificar la función de transferencia del motor en MATLAB/Python,
igual que se hizo en `LAB 2/firmware/sprint1_characterization`.

## Hardware

Mismo driver **DRV8833** que ya mueve el motor lineal del carro en
`lab3/fimware/main.ino` — **un solo board, dos canales**, confirmado por
foto de la board real (sin pines `PWMA`/`PWMB` dedicados, solo
`AIN1`/`AIN2`/`BIN1`/`BIN2`/`STBY`, `AO1`/`AO2`/`BO1`/`BO2`, `VM`, `GND`,
y varios `NC`). El motor de rotación va en el canal A (`AIN1`/`AIN2`); el
canal B (`BIN1`/`BIN2`) es el del motor lineal, no tocado por este sketch.
`STBY` es **compartido** entre ambos canales.

El DRV8833 no tiene un pin PWM dedicado: la velocidad se logra aplicando
PWM directamente sobre el pin de dirección activo (`AIN1` para un sentido,
`AIN2` para el otro), manteniendo el otro en 0. La posición se mide con el
encoder en cuadratura ya instalado en el eje de rotación (A/B), decodificado
por interrupción.

Motor: JGA25-370, 12V, 170 RPM.

### Pinout del driver (serigrafía de la board)

| Pin driver | Conexión |
|------------|----------|
| `VM`       | Alimentación de los motores a 12V (compartida por ambos canales) |
| `GND`      | Tierra común con el ESP32 |
| `STBY`     | GPIO 4 (HIGH = driver activo) — **compartido** con el canal B (motor lineal) |
| `AIN1`     | GPIO 25 |
| `AIN2`     | GPIO 26 |
| `AO1`/`AO2`| Terminales del motor de rotación (canal A) |
| `BIN1`/`BIN2`/`BO1`/`BO2`/`NC` | Canal B — motor lineal, no tocado por este sketch |

El DRV8833 no necesita un pin `VCC` lógico separado: deriva la lógica
internamente de `VM`.

### Pines del ESP32 usados por el sketch

Idénticos a los que ya usa `lab3/fimware/main.ino` para este driver, a
propósito — así este sketch es un re-test aislado del canal A sin cambiar
ningún wiring físico.

| Señal | GPIO |
|-------|------|
| AIN1  | 25   |
| AIN2  | 26   |
| STBY  | 4    |
| ENC_A | 32   |
| ENC_B | 33   |

### Colores de cable del motor (JGA25-370 con encoder)

Confirmado por foto y por cableado real en el motor lineal (mismo
módulo/lote): módulo con conector JST de 6 pines, encoder Hall con tapa
roja. Orden físico del cable: rojo, negro, amarillo, verde, azul, blanco —
**rojo y blanco son los dos de potencia del motor** (no rojo/negro como en
un cable típico):

| Color | Función | Va a |
|---|---|---|
| Rojo | Motor + | `AO1` |
| Blanco | Motor − | `AO2` |
| Amarillo | Encoder VCC | 3.3V |
| Verde | Encoder GND | Tierra común |
| Azul | Encoder canal A | GPIO32 |
| Negro | Encoder canal B | GPIO33 |

Nota: esta asignación se verificó cable por cable en el motor lineal; en
este motor (rotación) se asume el mismo patrón por ser el mismo
módulo/lote, pero no se verificó cable por cable aquí — si algo no
coincide al cablear, priorizar lo que mida el multímetro sobre esta tabla.

### Motor de rotación — las dos terminales

El motor (JGA25-370) tiene dos cables de potencia, sin polaridad fija:
cualquiera va a `AO1` y el otro a `AO2`. Si gira al revés de lo esperado,
se intercambian los dos cables en `AO1`/`AO2` (o se invierte
`INVERTIR_MOTOR_ROT` en `main.ino`, sin re-cablear).

### Las 6 terminales del JGA25-370 con encoder

Este motor trae el encoder incorporado: el cable del motor tiene 6 hilos
en total, no solo los 2 de potencia. Todos están contemplados en el
wiring de este sketch:

| Hilo | Función | Va a |
|---|---|---|
| Motor + | Potencia del motor | `AO1` |
| Motor − | Potencia del motor | `AO2` |
| Encoder VCC | Alimentación del encoder | 3.3V (confirmar contra el datasheet del motor) |
| Encoder GND | Tierra del encoder | Tierra común |
| Encoder canal A | Señal de cuadratura | GPIO32 |
| Encoder canal B | Señal de cuadratura | GPIO33 |

El encoder del JGA25-370 es de 2 canales (A/B), sin pulso de índice/Z — no
hay un séptimo hilo esperado. Si el cable de tu motor trae más de 6 hilos,
avisa el conteo/colores para revisar que no quede ninguno sin usar.

### No confundir el encoder con las salidas del driver

El encoder (`ENC_A`=GPIO32, `ENC_B`=GPIO33) es un dispositivo **separado**
del driver DRV8833, montado en el eje del motor. **Nunca** debe compartir
cable con `AO1`/`AO2` (esas salidas llevan los 12V de potencia del motor
directo a los dos cables del motor) — conectar el encoder ahí pondría 12V
en un GPIO del ESP32 y probablemente lo dañaría. El encoder lleva su propia
alimentación (VCC del encoder a 3.3V, GND común) y sus dos salidas de señal
van únicamente a GPIO32/GPIO33.

### Pinout completo de referencia (todo el sistema, no solo este sketch)

| Dispositivo | Pin | GPIO / conexión |
|---|---|---|
| DRV8833 | STBY | GPIO4 (compartido) |
| DRV8833 | AIN1 | GPIO25 |
| DRV8833 | AIN2 | GPIO26 |
| DRV8833 | BIN1 | GPIO18 |
| DRV8833 | BIN2 | GPIO19 |
| DRV8833 | AO1/AO2 | Terminales del motor de rotación |
| DRV8833 | BO1/BO2 | Terminales del motor lineal |
| DRV8833 | VM | 12V+ |
| DRV8833 | GND | Tierra común |
| Encoder rotación | Canal A | GPIO32 |
| Encoder rotación | Canal B | GPIO33 |
| Encoder rotación | VCC/GND | 3.3V / tierra común |
| VL53L0X (lineal) | SDA | GPIO21 |
| VL53L0X (lineal) | SCL | GPIO22 |
| Potenciómetro péndulo | Wiper | GPIO34 (sin construir aún) |

## Uso — pruebas manuales

Sube el sketch y abre el monitor serial a 115200 baud. Comandos
(terminados en `\n`):

| Comando | Efecto |
|---------|--------|
| `F<pwm>` | gira adelante, pwm 0–255 (ej. `F150`) |
| `R<pwm>` | gira en reversa, pwm 0–255 (ej. `R150`) |
| `S` | stop / coast |
| `B` | brake |
| `E` | imprime el conteo actual del encoder |
| `?` | reimprime la ayuda |

Verifica primero con esto: `F100` gira un sentido, `R100` el otro, `S`
detiene, `E` muestra que el conteo del encoder sube o baja según el
sentido de giro.

## Uso — caracterización (función de transferencia)

| Comando | Efecto |
|---------|--------|
| `T` | corre la prueba de escalón y vuelca el CSV al terminar |
| `X` | aborta la prueba en curso |
| `I` | imprime los parámetros de la prueba (Ts, amplitud, duraciones) |

La prueba `T`:
1. Deja el motor detenido `PRE_MS` (1000 ms), para ver el reposo inicial.
2. Aplica un escalón de `U_PCT` (30% duty) durante `STEP_MS` (4000 ms).
3. Vuelve a 0% durante `POST_MS` (4000 ms), para capturar la respuesta libre.
4. Muestrea cada `TS_US` (10 ms) el conteo del encoder usando un timer por
   interrupción (no se usa `Serial` dentro del muestreo para no perder
   tiempos), guarda todo en RAM, y al final imprime:
   ```
   k,t_us,u_pct,enc_count
   0,0,30.0,0
   1,10000,30.0,4
   ...
   # DONE
   ```
5. Copia esas líneas (desde el encabezado hasta `# DONE`) a un `.csv` y
   procésalas en MATLAB/Python: con `t_us` y `enc_count` (conviértelo a
   ángulo con los counts/rev del encoder) se puede ajustar un modelo de
   primer o segundo orden (p. ej. `tfest` en MATLAB) para obtener la
   función de transferencia planta-motor que luego alimenta el diseño de
   los controladores en `lab3/maltlab/main.m`.

Ajusta `U_PCT`, `MAX_PCT`, `PRE_MS`, `STEP_MS`, `POST_MS` y `TS_US` en el
sketch si el escalón necesita ser más grande/largo para ver bien la
respuesta transitoria.

## Siguiente paso

Una vez confirmado el wiring/polaridad/PWM y obtenida la función de
transferencia, estos pines y la lógica de control del DRV8833 se integran
al firmware real en `lab3/fimware/main.ino` como el eje de rotación (tarea
separada, fuera del alcance de este sprint).
