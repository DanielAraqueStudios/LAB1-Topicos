# Laboratorio 1 — Tópicos Avanzados de Control

Informe IEEE y simulaciones en MATLAB para la práctica de laboratorio **"Control mediante retroalimentación de estados"** (Universidad Militar Nueva Granada, Ingeniería en Mecatrónica), aplicado a un péndulo con rueda de reacción (rueda inercial).

## Contenido del repositorio

```
LAB 1/
├── lab1.tex                  # Informe IEEE (LaTeX), listo para compilar en Overleaf
├── format/
│   └── format.tex            # Plantilla IEEEtran de referencia (estructura y estilo)
├── pdf/
│   └── Laboratorio 1 ...pdf  # Guía de laboratorio (enunciado original)
├── simulations/
│   ├── simulation.m          # Modelo, formas canónicas, diseño de K, Ki y L (3 métodos)
│   ├── post_process.m        # Simulación de respuestas en lazo cerrado y generación de figuras
│   ├── figures/               # Figuras PNG generadas a partir de los scripts anteriores
│   └── Simulaciones_lab_1sirve.mdl  # Modelo Simulink original
└── figures/                  # Espacio para imágenes/diagramas adicionales del informe
```

## Resumen técnico

El sistema modelado es un péndulo con rueda de reacción, reducido a tres estados medibles $\vec{x}=[\theta_1,\ \omega_1,\ \omega_r]^T$. A partir de sus matrices $A$, $B$, $C$ se calcularon:

- **Formas canónicas** controlable (FCC) y observable (FCO).
- **Controlador por retroalimentación de estados** con servosistema integrante, mediante tres metodologías equivalentes: asignación directa de coeficientes, compensación por matriz de pesos y el método de Ackermann.
- **Observador de estados**, diseñado con dinámica de error diez veces más rápida que el controlador, calculado igualmente con las tres metodologías.
- **Comparación de diseño** subamortiguado ($\zeta=0.6$) vs. críticamente amortiguado ($\zeta=1.0$).

Todos los valores numéricos y figuras del informe provienen de ejecutar `simulation.m` y `post_process.m` en MATLAB (R2024b) — no hay datos inventados.

## Cómo compilar el informe

1. Sube la carpeta `LAB 1/` completa a un proyecto de [Overleaf](https://www.overleaf.com/) (o compílala localmente con `pdflatex`), manteniendo la estructura de carpetas tal como está (`simulations/`, `simulations/figures/`, `figures/`).
2. Compilador: **pdfLaTeX**.
3. Archivo principal: `lab1.tex`.

### Placeholders pendientes

El informe incluye espacios reservados (`\imgplaceholder{...}`) para las imágenes conceptuales de la guía (representación de estados, observador, servosistema, péndulo con rueda inercial) y para dos fotos del modelado físico del sistema. Reemplázalos por `\includegraphics{...}` cuando tengas las imágenes finales.

## Requisitos para reproducir las simulaciones

- MATLAB (probado en R2024b), sin toolboxes adicionales más allá de Control System Toolbox y Symbolic Math Toolbox.
- Ejecutar `simulation.m` primero (matrices y ganancias), luego `post_process.m` (genera las figuras en `simulations/figures/`).
