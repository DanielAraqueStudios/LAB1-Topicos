# Prueba aislada — motor lineal del carro (TB6612FNG canal B)

Sprint de validación de hardware y **caracterización** del motor lineal,
el mismo que ya está integrado y funcionando en `lab3/fimware/main.ino`.
Este sketch aísla solo ese eje (canal B del primer TB6612FNG + sensor de
distancia VL53L0X) para poder re-verificar wiring/polaridad y, si hace
falta, re-capturar una respuesta al escalón para refinar la función de
transferencia — igual que `lab3/sprints/rotation_motor_test`.

## Hardware

Usa el driver DRV8833 (el mismo que ya está wireado al carro), canal B
(`BIN1`/`BIN2`), con PWM aplicado directamente sobre `BIN1`/`BIN2` (no hay
pin PWM dedicado en este driver — coincide con cómo lo usa `main.ino`).
Distancia medida con el sensor VL53L0X por I2C.

El motor lineal (JGA25-370, 12V, 300 RPM) también trae encoder de fábrica,
igual que el de rotación — cableado aquí como sensor adicional. El VL53L0X
sigue siendo la fuente de posición que se usaría para control; el encoder
solo se reporta.

### Pines (ESP32) — iguales a los de `main.ino`

| Señal | GPIO |
|-------|------|
| BIN1  | 18   |
| BIN2  | 19   |
| STBY  | 4    |
| ENC_A (lineal) | 14 |
| ENC_B (lineal) | 27 |
| SDA (VL53L0X) | 21 |
| SCL (VL53L0X) | 22 |

`VM` del driver debe alimentarse con el voltaje nominal del motor lineal;
`VCC` del driver va a 3.3V del ESP32; `GND` común entre ESP32, driver y
fuente de `VM`.

### Colores de cable del motor (JGA25-370 con encoder)

Confirmado por foto y por cableado real: mismo módulo/lote que el motor de
rotación, conector JST de 6 pines, encoder Hall con tapa roja. Orden físico
del cable: rojo, negro, amarillo, verde, azul, blanco — **rojo y blanco son
los dos de potencia del motor** (no rojo/negro como en un cable típico):

| Color | Función | Va a |
|---|---|---|
| Rojo | Motor + | `BO1` |
| Blanco | Motor − | `BO2` |
| Amarillo | Encoder VCC | 3.3V |
| Verde | Encoder GND | Tierra común |
| Azul | Encoder canal A | GPIO14 |
| Negro | Encoder canal B | GPIO27 |

Si el conteo del encoder se ve errático una vez cableado, lo primero a
revisar es si el amarillo (VCC del encoder) necesita 5V en vez de 3.3V.

## Uso — pruebas manuales

Sube el sketch y abre el monitor serial a 115200 baud. Comandos
(terminados en `\n`):

| Comando | Efecto |
|---------|--------|
| `F<pwm>` | avanza, pwm 0–255 (ej. `F150`) |
| `R<pwm>` | retrocede, pwm 0–255 (ej. `R150`) |
| `S` | stop / coast |
| `B` | brake |
| `D` | imprime la distancia actual del VL53L0X (mm) y el conteo del encoder lineal |
| `?` | reimprime la ayuda |

Verifica primero con esto: `F100` mueve el carro un sentido, `R100` el
otro, `S` detiene, `D` muestra que la distancia cambia según el sentido de
movimiento.

## Uso — caracterización (función de transferencia)

| Comando | Efecto |
|---------|--------|
| `T` | corre la prueba de escalón y vuelca el CSV al terminar |
| `X` | aborta la prueba en curso |
| `I` | imprime los parámetros de la prueba (Ts, amplitud, duraciones) |

La prueba `T` sigue el mismo patrón que `rotation_motor_test` y que
`LAB 2/firmware/sprint1_characterization`: reposo (`PRE_MS`) → escalón de
`U_PCT` durante `STEP_MS` → reposo (`POST_MS`), muestreando cada `TS_US`
por timer de interrupción (sin `Serial` dentro del muestreo), y al final
imprime:

```
k,t_us,u_pct,dist_mm,enc_lin_count
0,0,30.0,120,0
1,10000,30.0,118,3
...
# DONE
```

Copia esas líneas a un `.csv` y ajusta un modelo de primer/segundo orden
(p. ej. `tfest` en MATLAB) para la función de transferencia del eje
lineal, para comparar o refinar la que ya usa `lab3/maltlab/main.m`.

Ajusta `U_PCT`, `MAX_PCT`, `PRE_MS`, `STEP_MS`, `POST_MS` y `TS_US` en el
sketch si el escalón necesita ser más grande/largo para ver bien la
respuesta transitoria.
