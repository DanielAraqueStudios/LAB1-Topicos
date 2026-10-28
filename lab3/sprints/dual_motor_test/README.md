# Prueba dual: ambos motores simultáneos (driver DRV8833 compartido)

Sketch de bring-up para validar que el carro (lineal) y la base (rotación)
pueden actuarse **al mismo tiempo** desde la **misma** board DRV8833, antes
de integrar esto en el firmware de control en cascada (`lab3/fimware/main.ino`).

## Qué se está validando

Ambos motores (JGA25-370, 12V) se alimentan del mismo driver **DRV8833**:
canal B para el lineal, canal A para la rotación, con **STBY compartido**
(GPIO4) y la misma alimentación `VM`. Este sketch existe para confirmar,
con las dos cargas corriendo a la vez, que:

- El STBY compartido no causa que un motor frene/corte al otro al cambiar de
  dirección o arrancar/parar.
- La corriente combinada de ambos motores no produce brownout del ESP32 ni
  reset del sensor VL53L0X.
- Los dos canales PWM (LEDC) no interfieren entre sí (timing, jitter) al
  correr en simultáneo con frecuencias distintas (ver tabla de pines).
- La lectura de distancia (VL53L0X, I2C) y el conteo de encoder (rotación)
  siguen siendo correctos con ambos motores activos.

Si algo falla aquí (un motor se detiene al mover el otro, lecturas erráticas,
reinicios), es una señal de un problema de hardware compartido (riel `VM`,
`STBY`, ruido I2C) que hay que resolver antes de correr el firmware completo.

Nota: este driver DRV8833 está especificado para 2.7–10.8V en `VM`; se está
usando a 12V (fuera de rango del datasheet) — vigilar temperatura del chip
durante estas pruebas.

## Pines (iguales a los sprints individuales y a main.ino)

| Señal | Pin | Notas |
|---|---|---|
| BIN1 (lineal) | 18 | DRV8833, canal B |
| BIN2 (lineal) | 19 | DRV8833, canal B |
| AIN1 (rotación) | 25 | DRV8833, canal A |
| AIN2 (rotación) | 26 | DRV8833, canal A |
| STBY | 4 | **Compartido** entre ambos canales |
| ENC_A (rotación) | 32 | Encoder cuadratura |
| ENC_B (rotación) | 33 | Encoder cuadratura |
| ENC_A (lineal) | 14 | Encoder cuadratura, sensor adicional (VL53L0X sigue siendo la referencia) |
| ENC_B (lineal) | 27 | Encoder cuadratura, sensor adicional |
| SDA / SCL (VL53L0X) | 21 / 22 | I2C, distancia lineal |

### Colores de cable de los motores JGA25-370 con encoder

Confirmado por foto: ambos motores (lineal 300 RPM y rotación 170 RPM) son
el mismo módulo/lote (conector JST de 6 pines, encoder Hall con tapa roja),
mismos colores en los dos:

| Color | Función | Va a (rotación) | Va a (lineal) |
|---|---|---|---|
| Rojo | Motor + | `AO1` | `BO1` |
| Negro | Motor − | `AO2` | `BO2` |
| Amarillo | Encoder VCC | 3.3V (confirmar si el encoder necesita 5V) | 3.3V |
| Verde | Encoder GND | Tierra común | Tierra común |
| Azul | Encoder canal A | GPIO32 | GPIO14 |
| Blanco | Encoder canal B | GPIO33 | GPIO27 |

Si el conteo del encoder se ve errático una vez cableado, lo primero a
revisar es si el amarillo (VCC del encoder) necesita 5V en vez de 3.3V.

El DRV8833 no tiene pin PWM dedicado: la velocidad se logra aplicando PWM
directamente sobre el pin de dirección activo (en ambos canales). PWM:
canal lineal a 1000 Hz (igual que `linear_motor_test`), canal rotación a
2000 Hz (igual que `rotation_motor_test`), ambos a 8 bits de resolución —
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
- `D` — imprime distancia lineal actual (mm, VL53L0X) y conteo del encoder lineal
- `E` — imprime conteo actual del encoder de rotación
- `?` — imprime esta ayuda

### Caracterización de la planta completa
- `T` — corre un escalón **simultáneo** en ambos ejes (mismo `U_PCT`,
  mismo perfil reposo→escalón→reposo que los sprints individuales) y
  vuelca un único CSV con ambas salidas:
  ```
  k,t_us,u_pct_lin,dist_mm,enc_lin_count,u_pct_rot,enc_count
  0,0,30.0,120,0,30.0,0
  1,10000,30.0,118,3,30.0,4
  ...
  # DONE
  ```
  Útil para identificar la planta completa (acoplamientos entre ejes
  incluidos) en lugar de cada eje por separado.
- `X` — aborta la prueba en curso
- `I` — imprime los parámetros de la prueba (Ts, amplitud, duraciones)

## Uso / verificación

1. Cargar el sketch, abrir monitor serie a 115200 baudios, terminador LF.
2. Probar cada motor por separado primero (`LF100`, `LS`, `RF100`, `RS`)
   para confirmar que el comportamiento individual no cambió frente a los
   sprints `linear_motor_test` y `rotation_motor_test`.
3. Probar el comando simultáneo, por ejemplo `M120,120` y luego `M120,-120`,
   dejando ambos motores corriendo varios segundos. Mientras corren, usar
   `D` y `E` para confirmar que las lecturas de distancia y encoder siguen
   siendo estables y no se congelan ni saltan de forma anómala.
4. Alternar direcciones rápido en ambos ejes a la vez (`M-150,150` seguido de
   `M150,-150`) para forzar el peor caso de conmutación simultánea en el
   STBY/VM compartido, y confirmar que ninguno de los dos motores se detiene
   inesperadamente ni el ESP32 se reinicia.
5. `S` o `B` en cualquier momento debe detener ambos motores de inmediato.

Si todo esto se comporta bien, el firmware de cascada (`main.ino`) puede
confiar en mover ambos ejes a la vez sin protecciones adicionales de
hardware más allá de las que ya tiene (watchdog, límites por eje).
