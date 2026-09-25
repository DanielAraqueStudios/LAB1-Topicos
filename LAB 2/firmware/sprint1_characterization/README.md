# Sprint 1 – Plant characterization firmware (ESP32)

Open-loop characterization of the motor-driven jib crane. The ESP32 applies a step or PRBS voltage to the motor through an H-bridge, samples both angle potentiometers at a fixed period, and prints the data as CSV for identification in MATLAB (Sprint 2).

## Hardware

| Signal | Default pin | Notes |
|---|---|---|
| H-bridge PWM / enable | GPIO 25 | 20 kHz, 10-bit |
| H-bridge direction | GPIO 26 | HIGH = positive |
| Jib angle potentiometer | GPIO 34 (ADC1) | wiper 0–3.3 V |
| Cable angle potentiometer | GPIO 35 (ADC1) | wiper 0–3.3 V |

Edit the pins at the top of `sprint1_characterization.ino` if your wiring differs. Use ADC1 pins only (ADC2 conflicts with WiFi). Power the potentiometers from 3.3 V, never 5 V. Share the ground between ESP32, H-bridge and motor supply.

## Requirements

- Arduino IDE or `arduino-cli` with the **esp32** board package (core 2.x or 3.x, both supported).
- Board: any ESP32 dev module. Serial monitor at **115200** baud, line ending "No line ending" or "Newline".

## Configuration

Constants at the top of the sketch:

| Constant | Default | Meaning |
|---|---|---|
| `TS_US` | 10000 | Sampling period (µs). Deliberately short; final Ts is chosen in Sprint 2 |
| `U_PCT` | 30 | Test amplitude, % duty. **Start lower (10–15) on first run** |
| `MAX_PCT` | 50 | Hard output clamp |
| `PRE_MS` / `STEP_MS` / `POST_MS` | 2000 / 8000 / 6000 | Rest, step on-time, free response |
| `PRBS_BIT_MS` / `PRBS_MS` | 300 / 20000 | PRBS bit time and duration |

Runs are limited to 2500 samples (25 s at 10 ms).

## Commands (serial)

| Key | Action |
|---|---|
| `s` | Step test |
| `p` | PRBS test |
| `x` | Abort (output forced to 0) |
| `i` | Print current configuration |

## Procedure

1. Flash the sketch. Serial prints `# Ready`.
2. Keep the crane clear of mechanical end stops and let it come fully to rest.
3. Send `s`. The output stays 0 for 2 s, steps to `+U_PCT` for 8 s, then returns to 0 for the free response. Wait for `# DONE`.
4. Save the CSV (copy from the monitor, or log with PuTTY / `arduino-cli monitor`). Name it e.g. `LAB 2/data/step_01.csv`. Remove the `#` lines.
5. Repeat the step 3 times, then run `p` twice (`prbs_01.csv`, `prbs_02.csv`).
6. Optionally repeat with negative amplitude (temporarily set `U_PCT` negative) to check symmetry and dead zone.

Tips for clean data: WiFi and Bluetooth off, decoupling capacitor (100 nF + 10 µF) on the potentiometer supply, fixed wiring, no one touching the crane during a run, identical starting position for each repeat.

## Output format

```
k,t_us,u_pct,y_jib_raw,y_cable_raw
0,0,0.0,2048,2051
```

`t_us` is the real timestamp of each sample, `u_pct` the signed duty applied from that sample on, `y_*_raw` the 12-bit ADC counts (0–4095). Each output sample is taken before the input at that step is applied.

## Checks before moving to Sprint 2

- `t_us` advances by ≈ 10000 every row (jitter well under 1 %).
- Both outputs are flat before the step and move clearly after it (a few hundred counts, not a handful).
- No clipping at 0 or 4095.
- Repeats overlap closely; if not, the plant is not returning to the same rest state.
- The response has settled before the run ends; if not, increase `STEP_MS` / `POST_MS`.

Send the CSV files to proceed to Sprint 2 (model identification).
