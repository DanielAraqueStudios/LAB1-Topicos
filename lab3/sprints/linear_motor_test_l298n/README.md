# Prueba aislada — motor lineal con driver L298N

**Controla únicamente el motor B (lineal)** — no toca el motor A
(rotación) en absoluto, ni sus pines ni su lógica. El eje de rotación
sigue usando el driver original (DRV8833) y no se ve afectado por este
sketch.

Variante de `lab3/sprints/linear_motor_test` (que usa el DRV8833) pero con
un **L298N** en su lugar, para comparar desempeño. Contexto: el DRV8833
está especificado para 2.7–10.8V y se está alimentando a 12V — se
sospecha que está entrando en protección por sobre-voltaje/corriente, ya
que el motor conectado directo a la fuente de 12V gira mucho más rápido y
fuerte que a través del DRV8833. El L298N soporta hasta 46V (cómodo a
12V), así que esta prueba sirve para confirmar si el problema es
específico del DRV8833.

**Aviso**: el L298N tiene una caída de voltaje interna (~2V combinados
entre sus transistores bipolares) mayor que la del DRV8833 (MOSFET, caída
casi nula) — en teoría el motor verá ~10V en vez de los 12V completos
incluso con el L298N funcionando bien. No es la solución ideal (esa sería
un driver tipo TB6612FNG, que sí está especificado para 12V y tiene caída
baja como el DRV8833), pero sirve para aislar si el problema actual es
"protección del DRV8833 activándose" o algo más.

## Hardware

Mismo motor lineal (JGA25-370, 12V, 300 RPM, con encoder) y mismo sensor
VL53L0X que `linear_motor_test`, cambiando solo el driver.

El L298N no tiene pin PWM separado por canal como el TB6612FNG, pero sí
tiene un pin `ENA` dedicado para habilitar/controlar velocidad por PWM del
canal A (a diferencia del DRV8833, que aplica PWM sobre los pines de
dirección). Para que `ENA` controle velocidad hace falta **quitar el
jumper** que algunos breakouts de L298N traen puesto de fábrica (ese
jumper amarra `ENA` directo a 5V, dejando el canal siempre al 100% sin
control de velocidad).

### Pinout del L298N usado

| Pin L298N | Conexión |
|-----------|----------|
| `IN1`     | GPIO18 |
| `IN2`     | GPIO19 |
| `ENA`     | GPIO23 (PWM) — **requiere quitar el jumper ENA** |
| `OUT1`    | Motor rojo (Motor +) |
| `OUT2`    | Motor blanco (Motor −) |
| `12V`     | Fuente de 12V |
| `GND`     | Tierra común con el ESP32 |
| `5V`      | Si el jumper de regulador interno está puesto, el L298N puede alimentar lógica a 5V por este pin — no conectar a la salida de 5V del ESP32 si ya se usa el regulador interno del L298N, para no pelear dos fuentes |

Canal B (`IN3`/`IN4`/`ENB`/`OUT3`/`OUT4`) no se usa en este sketch.

### Pines del ESP32 — iguales a `linear_motor_test` para el encoder/sensor

| Señal | GPIO |
|-------|------|
| ENC_A (lineal) | 14 |
| ENC_B (lineal) | 27 |
| SDA (VL53L0X) | 21 |
| SCL (VL53L0X) | 22 |

### Colores de cable del motor

Igual que en `linear_motor_test` (ver su README para la tabla completa):
rojo/blanco son los dos de potencia del motor (`OUT1`/`OUT2`), y
negro/amarillo/verde/azul son los cuatro del encoder.

## Uso — pruebas manuales

Sube el sketch y abre el monitor serial a 115200 baud. Comandos
(terminados en `\n`):

| Comando | Efecto |
|---------|--------|
| `F<pwm>` | avanza, pwm 0–255 (ej. `F150`) |
| `R<pwm>` | retrocede, pwm 0–255 (ej. `R150`) |
| `S` | stop / coast |
| `B` | brake |
| `D` | imprime distancia (mm) y conteo del encoder lineal |
| `?` | reimprime la ayuda |

Compara directamente contra `linear_motor_test` (DRV8833): mismo comando,
mismo pwm, ¿el motor va notablemente más rápido/fuerte con el L298N?

## Uso — caracterización (función de transferencia)

| Comando | Efecto |
|---------|--------|
| `T` | corre la prueba de escalón y vuelca el CSV al terminar |
| `X` | aborta la prueba en curso |
| `I` | imprime los parámetros de la prueba |

Mismo formato que `linear_motor_test`:
```
k,t_us,u_pct,dist_mm,enc_lin_count
0,0,30.0,120,0
1,10000,30.0,118,3
...
# DONE
```

## Conclusión esperada

- Si el L298N mueve el motor notablemente mejor que el DRV8833 al mismo
  pwm: confirma que el DRV8833 estaba limitándose por voltaje/corriente
  fuera de su rango especificado, y el firmware final (`main.ino`) debería
  migrar a un driver rated para 12V (TB6612FNG recomendado sobre L298N,
  por menor caída de voltaje).
- Si el L298N se comporta igual de débil: el problema no es el driver,
  hay que revisar otra cosa (alimentación real en el motor durante
  arranque, fricción mecánica, etc.).
