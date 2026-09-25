"""Simulated ESP32 that speaks the same serial protocol (no hardware needed).

Plant: jib = first-order lag with dead time, cable = lightly damped
second-order response driven by the jib angle. Exposes the small subset of
the pyserial API the worker uses.
"""
from __future__ import annotations

import time

import numpy as np

TS_US = 10000
U_PCT = 30.0
MAX_PCT = 50.0
PRE_MS, STEP_MS, POST_MS = 2000, 8000, 6000
PRBS_BIT_MS, PRBS_MS = 300, 20000
N_MAX = 2500
DUMP_SECONDS = 1.2      # simulated time to transmit the buffered CSV


def run_duration_s(kind: str) -> float:
    """Nominal on-device duration (matches the firmware defaults)."""
    return (PRE_MS + (PRBS_MS if kind == "prbs" else STEP_MS) + POST_MS) / 1000.0


def synthesize(kind: str, rng: np.random.Generator) -> list[str]:
    """Generate the CSV lines (header + rows) the firmware would print."""
    n = min(int(run_duration_s(kind) * 1e6 / TS_US), N_MAX)
    ts = TS_US * 1e-6
    tms = np.arange(n) * TS_US / 1000.0
    if kind == "prbs":
        u = np.zeros(n)
        bits = rng.integers(0, 2, size=PRBS_MS // PRBS_BIT_MS + 1)
        on = (tms >= PRE_MS) & (tms < PRE_MS + PRBS_MS)
        idx = ((tms - PRE_MS) // PRBS_BIT_MS).astype(int).clip(0, len(bits) - 1)
        u[on] = np.where(bits[idx[on]] == 1, U_PCT, -U_PCT)
    else:
        u = np.where((tms >= PRE_MS) & (tms < PRE_MS + STEP_MS), U_PCT, 0.0)
    delay, tau, gain = 0.15, 1.2, 28.0            # counts per % duty
    y1 = np.zeros(n)
    y2 = np.zeros(n)
    base1, base2 = 1500.0 + rng.uniform(-30, 30), 2050.0 + rng.uniform(-30, 30)
    wn, zeta, kc = 4.5, 0.18, 0.35
    x, v = 0.0, 0.0
    jib = 0.0
    d = int(round(delay / ts))
    for i in range(n):
        ud = u[i - d] if i >= d else 0.0
        jib += ts / tau * (gain * ud - jib)
        acc = wn * wn * (kc * jib - x) - 2 * zeta * wn * v
        v += acc * ts
        x += v * ts
        y1[i] = base1 + jib
        y2[i] = base2 + x
    y1 += rng.normal(0, 2.0, n)
    y2 += rng.normal(0, 2.0, n)
    y1 = np.clip(np.rint(y1), 0, 4095).astype(int)
    y2 = np.clip(np.rint(y2), 0, 4095).astype(int)
    t = np.arange(n) * TS_US + rng.integers(-40, 41, n)
    t[0] = 0
    lines = ["k,t_us,u_pct,y_jib_raw,y_cable_raw"]
    lines += [f"{i},{int(t[i])},{u[i]:.1f},{y1[i]},{y2[i]}" for i in range(n)]
    return lines


class SimulatedSerial:
    is_open = True

    def __init__(self, time_scale: float = 4.0) -> None:
        self.time_scale = time_scale
        self._rng = np.random.default_rng()
        self._out = bytearray(b"# Ready. 's' step, 'p' PRBS, 'x' abort, 'i' info\n")
        self._run_kind: str | None = None
        self._run_start = 0.0
        self._pending: list[str] = []
        self._dump_start = 0.0
        self._dump_total = 0

    # -- device behaviour ---------------------------------------------
    def _advance(self) -> None:
        now = time.monotonic()
        if self._run_kind and now - self._run_start >= run_duration_s(self._run_kind) / self.time_scale:
            self._pending = synthesize(self._run_kind, self._rng) + ["# DONE"]
            self._dump_total = len(self._pending)
            self._dump_start = now
            self._run_kind = None
        if self._pending:
            frac = min(1.0, (now - self._dump_start) / (DUMP_SECONDS / self.time_scale))
            due = int(self._dump_total * frac) - (self._dump_total - len(self._pending))
            for _ in range(max(due, 0)):
                self._out += (self._pending.pop(0) + "\n").encode()

    def write(self, data: bytes) -> int:
        for c in data.decode(errors="ignore"):
            if c in "sp" and not self._run_kind and not self._pending:
                self._run_kind = "step" if c == "s" else "prbs"
                self._run_start = time.monotonic()
            elif c == "x" and self._run_kind:
                self._run_kind = None
                self._out += b"# ABORTED\n"
            elif c == "i":
                self._out += f"# Ts={TS_US} us, U={U_PCT:.1f}%, max={MAX_PCT:.1f}%\n".encode()
        return len(data)

    # -- pyserial-like reading -----------------------------------------
    @property
    def in_waiting(self) -> int:
        self._advance()
        return len(self._out)

    def read(self, n: int = 1) -> bytes:
        self._advance()
        chunk = bytes(self._out[:n])
        del self._out[:n]
        return chunk

    def close(self) -> None:
        self.is_open = False
