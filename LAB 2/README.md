# Lab 2 - Identification and Discrete Control of a Jib Crane

Universidad Militar Nueva Granada (UMNG) - Advanced Control Topics (Tópicos Avanzados de Control).
Practice guide: *Modelamiento, Identificación y Simulación de un Sistema de Control en Tiempo Discreto* (`Laboratorio 2 Topicos 2026-2.pdf`).

## 1. Overview

The plant is a motor-driven column jib crane. The goal is to obtain a discrete-time model from measured data, compare candidate models objectively, and then use the chosen model to design and test a discrete controller.

| Item | Description |
|---|---|
| Input | Motor voltage, +/-12 V (applied as signed PWM duty through an H-bridge) |
| Outputs | Jib angle and load-cable angle (two potentiometers) |
| Controller/DAQ | ESP32, both potentiometers on ADC1 (0-3.3 V), H-bridge driven by PWM + DIR |
| Tools | Arduino/ESP32 firmware, MATLAB (System Identification Toolbox), LaTeX (IEEE report) |

The plant is treated as SIMO: one input, two outputs, each identified separately.

## 2. Objectives and deliverables

Objectives taken from the guide:

- Model the crane mechanically and propose a CAD design of the plant.
- Identify the plant graphically (FOPDT / second-order fit from the step response).
- Identify it with the System Identification Toolbox using ARX, ARMAX, OE and BJ structures, several orders, after `detrend`.
- Simulate every model with `lsim` against measured data and compare with the mean squared error (MSE).
- Select the model with an objective criterion (MSE, fit %, residual analysis).

Deliverables:

- [ ] CAD design of the crane
- [ ] Graphical identification (both outputs)
- [ ] Toolbox identification (ARX/ARMAX/OE/BJ, detrended data)
- [ ] `lsim` + MSE comparison table and objective model selection
- [ ] IEEE-format report
- [ ] Answers to the discussion questions (see section 8)

## 3. Work plan

Each phase must be validated before starting the next.

| Phase | Content | Status | Gate to move on |
|---|---|---|---|
| 1. Characterization | ESP32 firmware for open-loop step and PRBS tests with CSV logging; data collection on the real plant | Firmware done; data collection pending | Clean CSVs: sampling jitter < 1 %, outputs move clearly and do not clip, repeats overlap, response settles |
| 2. Model identification | Graphical FOPDT / second-order fit; Ident sweep (ARX/ARMAX/OE/BJ, orders, detrend); MSE, fit %, residual checks; validated sampling period Ts | Pending | Chosen model validated on data not used for fitting; Ts confirmed |
| 3. Discrete controller design | ZOH discretization at the validated Ts, discrete PID, predicted closed-loop response | Pending | Predicted response meets specifications with actuator limits respected |
| 4. Closed-loop firmware and tests | ESP32 timer-driven control loop, output saturation, anti-windup; tests on the plant compared with the prediction | Pending | Measured response agrees with prediction |

## 4. Repository structure

| Path | Purpose | Status |
|---|---|---|
| `Laboratorio 2 Topicos 2026-2.pdf` | Lab guide | Exists |
| `firmware/sprint1_characterization/` | Phase 1 characterization firmware and its README | Exists |
| `firmware/` (closed-loop sketch) | Phase 4 control firmware | Planned |
| `gui/` | Desktop tool (Python) for acquiring and inspecting the serial data: `core/`, `ui/`, `tests/` | In progress |
| `data/` | Recorded CSV files | Planned |
| `scripts/` | MATLAB scripts for identification and controller design | Planned |

## 5. Quick start (Phase 1)

1. Check the pin mapping at the top of the sketch (defaults: PWM GPIO 25, DIR GPIO 26, jib pot GPIO 34, cable pot GPIO 35).
2. Flash `firmware/sprint1_characterization/sprint1_characterization.ino` to the ESP32 (esp32 core 2.x or 3.x), serial at 115200 baud.
3. Start with a low amplitude (`U_PCT` 10-15), keep the crane away from end stops, and let it rest.
4. Send `s` for a step test or `p` for a PRBS test; `x` aborts; `i` prints the configuration.
5. Save the CSV to `data/` (see section 6).

Full details (wiring, constants, procedure, acceptance checks): [firmware/sprint1_characterization/README.md](firmware/sprint1_characterization/README.md).

## 6. Data conventions

CSV header and rows:

```
k,t_us,u_pct,y_jib_raw,y_cable_raw
0,0,0.0,2048,2051
```

- `k`: sample index. `t_us`: real timestamp in microseconds (used to verify jitter).
- `u_pct`: signed duty applied from that sample on. `y_*_raw`: 12-bit ADC counts (0-4095).
- Remove the `#` comment lines printed by the firmware before saving.
- File names in `LAB 2/data/`: `step_01.csv`, `step_02.csv`, ... and `prbs_01.csv`, `prbs_02.csv`, ...
- Plan: at least 3 step repeats and 2 PRBS runs at the same starting position; optionally a negative-amplitude step to check symmetry and dead zone.
- Convert counts to angle with the potentiometer calibration in the identification stage; keep raw files unmodified.

## 7. Open items to confirm

- [ ] Final GPIO pin mapping (PWM, DIR, both potentiometers)
- [ ] H-bridge model and its PWM/DIR logic
- [ ] Motor supply voltage and current limits
- [ ] Safe maximum duty (currently clamped at 50 %)
- [ ] Mechanical end stops and safe angular range for both axes
- [ ] Potentiometer supply (3.3 V) and angle calibration (counts to degrees)

## 8. Checklists

Discussion questions (from the guide, section "Preguntas para la discusión"):

- [ ] Discrete modelling: how does the sampling period affect the accuracy of the discrete model, and what are the risks of a value that is too large or too small?
- [ ] System identification: advantages and limitations of graphical methods versus the System Identification Toolbox, in reliability and practical applicability
- [ ] Model validation: why compute metrics such as the MSE, and how does it relate to the predictive capability of the model?
- [ ] Control applications: how does identifying and simulating the discrete system help design digital controllers (PID, predictive, robust), and what is the impact on microcontroller implementation?

Report (IEEE format):

- [ ] Abstract, introduction, plant description and CAD
- [ ] Experimental setup and data acquisition (Ts, signals, repeats)
- [ ] Graphical identification results
- [ ] Toolbox identification results and MSE/fit comparison table
- [ ] Model selection justification and residual analysis
- [ ] Controller design and closed-loop results (predicted vs measured)
- [ ] Discussion answers, conclusions, bibliography
