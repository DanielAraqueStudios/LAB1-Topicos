# Taller 2 - Metodos de diseno

Dos sistemas linealizados (Sistema 2: RLC con resistencia no lineal; Sistema 4:
proceso neumatico de dos tanques) analizados y controlados en tiempo discreto.
Modelos y herramientas de diseno en Python (`src/`), verificacion/simulacion
y generacion de modelos Simulink en MATLAB (`matlab/`).

## Modelos de planta (`src/system2_model.py`, `src/system4_model.py`)

Espacio de estados continuo `(A,B,C,D)` obtenido por linealizacion alrededor
de un punto de operacion (supuestos documentados en `ASSUMPTIONS` de cada
modulo). Verificacion simbolica de `G(s)=C(sI-A)^-1B+D` con `sympy`.

## Herramientas de diseno discreto (`src/discrete_tools.py`)

- **`c2d_zoh`**: discretizacion exacta ZOH via exponencial de matriz en bloque
  `expm([[A,B],[0,0]]*T)`.
- **`jury_stability`**: prueba de Jury (tabla analitica para grado<=3, raices
  numericas `|z|<1` como respaldo para grado>3).
- **`map_s_to_z_pole` / `zeta_wn_from_s_pole`**: mapeo de polos `s->z=e^{sT}` y
  extraccion de `(zeta,wn)` desde un polo en `s`.
- **`system_type_and_error_constants`**: tipo de sistema (polos en `z=1`/`s=0`)
  y constantes de error `Kp,Kv,Ka` por limites simbolicos.
- **`design_compensator` / `design_deadbeat`**: compensador digital por
  emparejamiento de coeficientes (denominador fijado por integradores/polos
  extra, numerador resuelto simbolicamente). Cuando el sistema de ecuaciones
  es sobredeterminado (`deg(denG)>1` con denominador fijo) cae a un ajuste por
  minimos cuadrados (`exact=False`) — limitacion estructural del metodo, no un
  error numerico. Deadbeat = mismo metodo con todos los polos deseados en `z=0`.
- **`design_lead_lag_angle`**: compensador lead/lag por **criterio de angulo**
  de lugar de raices (root locus) en el plano `z`. Ubica el polo dominante
  `z_d` desde la especificacion de 2do orden discreta `(zeta,wn,T)`, mide el
  deficit angular `theta` en `z_d`, lo reparte en `n_sections` secciones
  lead/lag identicas (cero en `Re(z_d)`, polo resuelto por la condicion de
  angulo, evitando que una sola seccion exceda `max_section_angle_deg`), y fija
  la ganancia por la condicion de magnitud `|C(z_d)G(z_d)|=1`. Metodo preferido
  para el compensador digital (reemplaza `design_compensator` en esa seccion).
- **`ackermann_place` / `_bass_gura_gain`**: realimentacion de estados SISO por
  formula de Ackermann, con Bass-Gura como validacion cruzada independiente.
- **`observer_poles_ts_rule`**: polos del observador `factor` veces mas rapidos
  que los de lazo cerrado (regla `tsdo=tsd/factor`, `factor=10` por defecto).
- **`luenberger_observer_gain`**: ganancia del observador via dualidad de
  Ackermann sobre `(A^T,C^T)`, tambien validada contra Bass-Gura (lanza error
  si ambos metodos difieren).
- **`augment_with_integrator`**: aumento clasico de 1 estado integral del error
  (`xi[k+1]=xi[k]+(r-y)`), forma legada solo para seguimiento de escalon.
- **`augment_for_tracking`**: forma general "-CG" (servo tipo Ogata) con
  `order` estados en cascada para seguir escalon (`order=1`), rampa
  (`order=2`) o parabola (`order=3`); la referencia se inyecta en el ultimo
  estado aumentado.
- **`simulate_discrete_closed_loop` / `simulate_compensator_loop`**
  (`src/discrete_tools_sim.py`): simulacion en lazo cerrado con/sin observador,
  y simulacion de compensador en serie con realizacion de forma directa.

## Scripts MATLAB (`matlab/sistema2_taller2.m`, `matlab/sistema4_taller2.m`)

Cada script, para su sistema:

1. Construye la planta continua (`ss`/`tf`) con las matrices numericas de
   Python y discretiza con `c2d(sys,T,'zoh')`.
2. Respuesta al escalon en lazo abierto (`step`, `stepinfo`).
3. Compensador digital: reconstruye el diseno lead/lag por criterio de angulo
   (coeficientes numericos de Python) y verifica los polos de lazo cerrado
   (`pole`).
4. Controlador deadbeat: idem con los coeficientes deadbeat.
5. Realimentacion de estados + observador + servo tipo Ogata (forma `-CG`,
   `augment_for_tracking`): las matrices aumentadas `Ghat/Hhat` se construyen
   en MATLAB a partir de `Ad,Bd,Cd` (no se hardcodean), y la ganancia `Khat`
   (control) y `L` (observador) se recalculan con `place()` sobre los polos
   deseados que entrega Python — mantiene el diseno robusto ante cualquier
   correccion posterior de esos polos. Simulacion manual del lazo (planta
   real + observador + integrador de servo) siguiendo referencia de escalon
   (Sistema 4) o rampa (Sistema 2).
6. Generacion automatica de un modelo Simulink (`Step->Sum->Controlador->
   Planta_Discreta->Scope` con realimentacion unitaria), envuelta en
   `try/catch` para degradar con una advertencia si Simulink no esta
   licenciado (el resto del script sigue funcionando via Control System
   Toolbox).

**Nota:** estos scripts no se ejecutaron localmente (no hay MATLAB instalado
en este entorno) — revisar la salida de `pole()`/`eig()` de verificacion antes
de dar el diseno por definitivo.
