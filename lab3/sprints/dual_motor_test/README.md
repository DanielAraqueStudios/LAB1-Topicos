# Prueba dual: ambos motores simultáneos (dos drivers separados)

Sketch de bring-up para validar que el carro (lineal) y la base (rotación)
pueden actuarse **al mismo tiempo**, cada uno desde su propia board driver,
antes de integrar esto en el firmware de control en cascada
(`lab3/fimware/main.ino`).

## Qué se está validando

Cada motor (JGA25-370, 12V) tiene su **propio driver**: el lineal (300 RPM)
usa un **TB6612FNG** (confirmado, ya probado y funcionando), el de
rotación (170 RPM) usa un **DRV8833** (confirmado por foto de la board
real). Son dos boards físicamente distintas, cada una con su propio `STBY`
y su propia alimentación `VM` — no comparten nada salvo `GND` y el ESP32.
Este sketch existe para confirmar, con las dos cargas corriendo a la vez,
que:

- Ningún motor frena/corta al otro al cambiar de dirección o arrancar/parar
  (no deberían interferir, al ser drivers independientes, pero se valida).
- La corriente combinada de ambos motores no produce brownout del ESP32 ni
  reset del sensor VL53L0X.
- Los canales PWM (LEDC) de ambos drivers no interfieren entre sí (timing,
  jitter) al correr en simultáneo con frecuencias distintas.
- La lectura de distancia (VL53L0X, I2C) y el conteo de encoder (rotación)
  siguen siendo correctos con ambos motores activos.

Si algo falla aquí (un motor se detiene al mover el otro, lecturas
erráticas, reinicios), es una señal de un problema de alimentación
compartida (fuente de 12V insuficiente para ambos motores a la vez) o de
ruido I2C/encoder inducido por el PWM, que hay que resolver antes de correr
el firmware completo.

## Pines (iguales a los sprints individuales y a main.ino)

| Señal | Pin | Notas |
|---|---|---|
| BIN1 (lineal) | 18 | TB6612FNG, canal B |
| BIN2 (lineal) | 19 | TB6612FNG, canal B |
| STBY (lineal) | 4 | Driver del motor lineal — **no** compartido con rotación |
| AIN1 (rotación) | 25 | DRV8833, canal A |
| AIN2 (rotación) | 26 | DRV8833, canal A — PWM se aplica directo sobre AIN1/AIN2 (sin pin PWM dedicado) |
| STBY (rotación) | 14 | Driver del motor de rotación — propio, independiente del lineal |
| ENC_A (rotación) | 32 | Encoder cuadratura |
| ENC_B (rotación) | 33 | Encoder cuadratura |
| SDA / SCL (VL53L0X) | 21 / 22 | I2C, distancia lineal |

PWM: canal lineal a 1000 Hz (igual que `linear_motor_test`), canal rotación
a 2000 Hz (igual que `rotation_motor_test`), ambos a 8 bits de resolución —
se mantienen las mismas frecuencias que cada sprint individual para que el
comportamiento de cada motor no cambie al correr junto al otro.

## Comandos

### Lineal (prefijo `L`)
- `LF<pwm>` — avanza, pwm 0-255 (ej: `LF150`)
- `LR<pwm>` — retrocede, pwm 0-255 (ej: `LR150`)
- `LS` — stop lineal (coast)
- `LB` — brake lineal

### Rotación (prefijo `R`)
- `RF<pwm>` — adelante, pwm 0-255 (ej: `RF150`)
- `RR<pwm>` — reversa, pwm 0-255 (ej: `RR150`)
- `RS` — stop rotación (coast)
- `RB` — brake rotación

### Simultáneo
- `M<pwmLin>,<pwmRot>` — aplica PWM con signo a ambos motores a la vez.
  Signo = dirección (positivo/negativo), igual convención que F/R en los
  sprints individuales. Ejemplo: `M150,-100` avanza el carro al 150/255 y
  gira la base en reversa al 100/255.

### Globales
- `S` — stop global (coast, ambos motores)
- `B` — brake global (ambos motores)
- `D` — imprime distancia lineal actual (mm, VL53L0X)
- `E` — imprime conteo actual del encoder de rotación
- `?` — imprime esta ayuda

## Uso / verificación

1. Cargar el sketch, abrir monitor serie a 115200 baudios, terminador LF.
2. Probar cada motor por separado primero (`LF100`, `LS`, `RF100`, `RS`)
   para confirmar que el comportamiento individual no cambió frente a los
   sprints `linear_motor_test` y `rotation_motor_test`.
3. Probar el comando simultáneo, por ejemplo `M120,120` y luego `M120,-120`,
   dejando ambos motores corriendo varios segundos. Mientras corren, usar
   `D` y `E` para confirmar que las lecturas de distancia y encoder siguen
   siendo estables y no se congelan ni saltan de forma anómala.
4. Alternar direcciones rápido en ambos ejes a la vez (`M-150,150` seguido
   de `M150,-150`) para forzar el peor caso de conmutación simultánea en la
   fuente de 12V compartida, y confirmar que ninguno de los dos motores se
   detiene inesperadamente ni el ESP32 se reinicia.
5. `S` o `B` en cualquier momento debe detener ambos motores de inmediato.

Si todo esto se comporta bien, el firmware de cascada (`main.ino`) puede
confiar en mover ambos ejes a la vez sin protecciones adicionales de
hardware más allá de las que ya tiene (watchdog, límites por eje).
