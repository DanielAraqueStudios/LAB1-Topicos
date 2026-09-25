# Jib crane data acquisition GUI

Desktop app (PyQt6 + pyqtgraph) for acquiring the open-loop characterization runs of Sprint 1 from the ESP32 over serial.
The firmware buffers the whole run on the device and dumps it as CSV afterwards, so the app shows a progress bar while the
test runs (step 16 s, PRBS 28 s, the firmware defaults) and then a "receiving data" state while the CSV is transferred.

## Install and run

```bash
cd "LAB 2/gui"
pip install -r requirements.txt
python main.py                # real device
python main.py --simulated    # preselect the built-in simulated device
python main.py --theme light
```

Pick the ESP32 port, keep 115200 baud, press **Connect**, then **Step** or **PRBS**. The simulated device (last entry of
the port list) needs no hardware: it generates a first-order-with-delay jib response and a lightly damped cable response
and runs at 4x speed.

## Features

- Connection panel with port refresh, baud selection and status indicator.
- Step / PRBS / Abort. Abort works while the test is running (sends `x`). Abort is not available during the data
  transfer, which the firmware cannot interrupt.
- Linked plots of input u, jib angle and cable angle versus time, in raw counts or volts (raw / 4095 x 3.3).
- Data quality panel with PASS / WARN / FAIL badges: sample count and missing samples, mean and max Ts, peak jitter,
  min/max per channel, clipping at 0 or 4095, flat or weak response, rest state before the input.
- Run table: automatic names (`step_01`, `prbs_01`, ...), quality per run, saved flag. Tick **Show** on several runs to
  overlay them and check repeatability. The selected run is drawn thicker and drives the quality panel.
- Save CSV to the chosen folder (default `LAB 2/data/`), exactly `k,t_us,u_pct,y_jib_raw,y_cable_raw` with no `#` lines.
  Existing files are never overwritten (the next free index is used). Load existing CSVs with **Load CSV**.
- Collapsible raw serial log (TX, RX and system lines).
- Dark and light themes.

### Shortcuts

| Key | Action | Key | Action |
|---|---|---|---|
| Ctrl+K | Connect / disconnect | Ctrl+S | Save selected run |
| F5 | Refresh ports | Ctrl+O | Load CSV |
| Ctrl+1 | Step test | Del | Remove selected run |
| Ctrl+2 | PRBS test | Ctrl+T | Toggle theme |
| Esc | Abort | Ctrl+L | Toggle serial log |
| Ctrl+0 | Fit plot view | | |

## Layout

```
main.py                    entry point
core/parser.py             protocol parsing (pure)
core/quality.py            quality metrics and checks (pure)
core/models.py             Run data class
core/storage.py            CSV save / load, auto naming
core/session.py            runs held in memory (Qt-free)
core/simulator.py          simulated device speaking the same protocol
core/serial_worker.py      all serial I/O, runs in a QThread, talks to the UI through signals
ui/theme.py                palettes and QSS
ui/messages.py             user-facing error text
ui/main_window.py          wiring
ui/widgets/                connection, test, plots, quality, runs, log panels
tests/                     pytest suite (no hardware needed)
```

## Tests

```bash
pip install -r requirements-dev.txt
QT_QPA_PLATFORM=offscreen python -m pytest -q      # PowerShell: $env:QT_QPA_PLATFORM="offscreen"
```

The suite covers the parser, quality checks, storage and naming, and an end-to-end GUI run against the simulated device
(step, abort while running, overlay, save, theme switch).

## Packaging with PyInstaller

```bash
pip install pyinstaller
pyinstaller --noconfirm --windowed --name JibCraneDAQ main.py
```

The result is in `dist/JibCraneDAQ/`. The default data folder is resolved relative to the source tree, so a packaged
build should use the **Folder...** button to choose where to save. Packaging has not been exercised in this repo; if a
module is reported missing at start-up add `--collect-submodules pyqtgraph`.

## Troubleshooting

- "Port is busy": close the Arduino IDE serial monitor or any terminal using the port.
- "Device not responding": confirm the characterization firmware is flashed, 115200 baud, and press EN on the board.
  Opening the port resets most ESP32 boards, so the first reply can take about two seconds.
- Timing warnings on a run come from the device, not the PC: the timestamps are taken by the firmware.
