# Prueba dual: ambos motores simultaneos (driver TB6612FNG compartido)

Sketch de bring-up para validar que el carro (lineal) y la base (rotacion)
pueden actuarse **al mismo tiempo** desde la **misma** board TB6612FNG, antes
de integrar esto en el firmware de control en cascada (`lab3/fimware/main.ino`).

## Que se esta validando

Ambos motores (JGA25-370, 12V) se alimentan del mismo driver TB6612FNG:
canal B para el lineal, canal A para la rotacion, con **STBY compartido**
(GPIO4) y la misma alimentacion VM. Este sketch existe para confirmar, con
las dos cargas corriendo a la vez, que:

- El STBY compartido no causa que un motor frene/corte al otro al cambiar de
  direccion o arrancar/parar.
- La corriente combinada de ambos motores no produce brownout del ESP32 ni
  reset del sensor VL53L0X.
- Los dos canales PWM (LEDC) no interfieren entre si (timing, jitter) al
  correr en simultaneo con frecuencias distintas (ver tabla de pines).
- La lectura de distancia (VL53L0X, I2C) y el conteo de encoder (rotacion)
  siguen siendo correctos con ambos motores activos.

Si algo falla aqui (un motor se detiene al mover el otro, lecturas erraticas,
reinicios), es una senal de un problema de hardware compartido (rail VM,
STBY, ruido I2C) que hay que resolver antes de correr el firmware completo.

## Pines (iguales a los sprints individuales y a main.ino)

| Señal | Pin | Notas |
|---|---|---|
| BIN1 (lineal) | 18 | TB6612FNG canal B |
| BIN2 (lineal) | 19 | TB6612FNG canal B |
| AIN1 (rotacion) | 25 | TB6612FNG canal A |
| AIN2 (rotacion) | 26 | TB6612FNG canal A |
| PWMA (rotacion) | 27 | TB6612FNG canal A |
| STBY | 4 | **Compartido** entre ambos canales |
| ENC_A (rotacion) | 32 | Encoder cuadratura |
| ENC_B (rotacion) | 33 | Encoder cuadratura |
| SDA / SCL (VL53L0X) | 21 / 22 | I2C, distancia lineal |

PWM: canal lineal a 1000 Hz (igual que `linear_motor_test`), canal rotacion a
2000 Hz (igual que `rotation_motor_test`), ambos a 8 bits de resolucion — se
mantienen las mismas frecuencias que cada sprint individual para que el
comportamiento de cada motor no cambie al correr junto al otro.

## Comandos

### Lineal (prefijo `L`)
- `LF<pwm>` — avanza, pwm 0-255 (ej: `LF150`)
- `LR<pwm>` — retrocede, pwm 0-255 (ej: `LR150`)
- `LS` — stop lineal (coast)
- `LB` — brake lineal

### Rotacion (prefijo `R`)
- `RF<pwm>` — adelante, pwm 0-255 (ej: `RF150`)
- `RR<pwm>` — reversa, pwm 0-255 (ej: `RR150`)
- `RS` — stop rotacion (coast)
- `RB` — brake rotacion

### Simultaneo
- `M<pwmLin>,<pwmRot>` — aplica PWM con signo a ambos motores a la vez.
  Signo = direccion (positivo/negativo), igual convencion que F/R en los
  sprints individuales. Ejemplo: `M150,-100` avanza el carro al 150/255 y
  gira la base en reversa al 100/255.

### Globales
- `S` — stop global (coast, ambos motores)
- `B` — brake global (ambos motores)
- `D` — imprime distancia lineal actual (mm, VL53L0X)
- `E` — imprime conteo actual del encoder de rotacion
- `?` — imprime esta ayuda

## Uso / verificacion

1. Cargar el sketch, abrir monitor serie a 115200 baudios, terminador LF.
2. Probar cada motor por separado primero (`LF100`, `LS`, `RF100`, `RS`)
   para confirmar que el comportamiento individual no cambio frente a los
   sprints `linear_motor_test` y `rotation_motor_test`.
3. Probar el comando simultaneo, por ejemplo `M120,120` y luego `M120,-120`,
   dejando ambos motores corriendo varios segundos. Mientras corren, usar
   `D` y `E` para confirmar que las lecturas de distancia y encoder siguen
   siendo estables y no se congelan ni saltan de forma anomala.
4. Alternar direcciones rapido en ambos ejes a la vez (`M-150,150` seguido de
   `M150,-150`) para forzar el peor caso de conmutacion simultanea en el
   STBY/VM compartido, y confirmar que ninguno de los dos motores se detiene
   inesperadamente ni el ESP32 se reinicia.
5. `S` o `B` en cualquier momento debe detener ambos motores de inmediato.

Si todo esto se comporta bien, el firmware de cascada (`main.ino`) puede
confiar en mover ambos ejes a la vez sin protecciones adicionales de
hardware mas alla de las que ya tiene (watchdog, limites por eje).
