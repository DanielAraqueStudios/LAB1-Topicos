# Prueba aislada — motor de rotación (TB6612FNG)

Sprint de validación de hardware y **caracterización** del motor de
rotación. No es el firmware final (ese vive en `lab3/fimware/`): este
sketch sirve para (1) confirmar que el driver mueve el motor en ambos
sentidos, y (2) una vez que eso funciona, correr una prueba de escalón y
volcar los datos (tiempo, PWM aplicado, conteo del encoder) por serial para
poder identificar la función de transferencia del motor en MATLAB/Python,
igual que se hizo en `LAB 2/firmware/sprint1_characterization`.

## Hardware

Segundo breakout TB6612FNG, confirmado por foto, independiente del que mueve
el motor lineal del carro. Usa un solo canal, el A (`AIN1`/`AIN2`/`PWMA`/
`AO1`/`AO2`), con `STBY` en HIGH para que el driver esté activo. La posición
se mide con el encoder en cuadratura ya instalado en el eje de rotación
(A/B), decodificado por interrupción.

### Pinout del driver (serigrafía de la board)

| Pin driver | Conexión |
|------------|----------|
| `VM`       | Alimentación del motor (riel de potencia, según voltaje del motor de rotación) |
| `VCC`      | Alimentación lógica (3.3V desde el ESP32) |
| `GND`      | Tierra común con el ESP32 y con VM |
| `STBY`     | GPIO 14 (HIGH = driver activo) |
| `AIN1`     | GPIO 25 |
| `AIN2`     | GPIO 26 |
| `PWMA`     | GPIO 27 |
| `AO1`/`AO2`| Terminales del motor de rotación (canal A) |
| `BIN1`/`BIN2`/`PWMB`/`BO1`/`BO2` | Sin usar en este sketch (canal B libre) |

### Pines del ESP32 usados por el sketch

| Señal | GPIO |
|-------|------|
| AIN1  | 25   |
| AIN2  | 26   |
| PWMA  | 27   |
| STBY  | 14   |
| ENC_A | 32   |
| ENC_B | 33   |

La lógica del TB6612FNG funciona con 2.7–5.5V en `VCC`, pero `VM` debe
dimensionarse según la corriente/voltaje nominal del motor de rotación —
no lo alimentes desde el riel de 3.3V del ESP32.

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

Una vez confirmado el wiring, usa:

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
transferencia, estos pines y la lógica de control del TB6612FNG se integran
al firmware real en `lab3/fimware/main.ino` como el eje de rotación (tarea
separada, fuera del alcance de este sprint).
