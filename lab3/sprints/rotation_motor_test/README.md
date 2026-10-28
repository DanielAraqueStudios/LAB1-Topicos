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
