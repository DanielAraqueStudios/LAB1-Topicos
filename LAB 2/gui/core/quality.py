"""Data-quality analysis of a run (pure functions, no Qt)."""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .models import ADC_MAX, Run

PASS, WARN, FAIL = "pass", "warn", "fail"
_RANK = {PASS: 0, WARN: 1, FAIL: 2}

MIN_SAMPLES = 100
JITTER_WARN_PCT = 1.0
JITTER_FAIL_PCT = 5.0
FLAT_P2P = 50          # counts: below this the channel did not respond
WEAK_P2P = 150         # counts: below this the response is weak
REST_P2P = 40          # counts: pre-input activity considered "not at rest"
CLIP_FAIL_FRACTION = 0.01


@dataclass
class Check:
    key: str
    label: str
    status: str
    detail: str


@dataclass
class QualityReport:
    metrics: dict = field(default_factory=dict)
    checks: list = field(default_factory=list)

    @property
    def overall(self) -> str:
        return max((c.status for c in self.checks), key=_RANK.get, default=PASS)


def _worst(*statuses: str) -> str:
    return max(statuses, key=_RANK.get)


def analyze(run: Run) -> QualityReport:
    rep = QualityReport()
    m = rep.metrics
    n = run.n
    m["samples"] = n

    # --- sample count / completeness
    if n < MIN_SAMPLES:
        rep.checks.append(Check("samples", "Sample count", FAIL, f"{n} samples (need at least {MIN_SAMPLES})"))
    else:
        expected = int(run.k[-1]) + 1
        missing = expected - n
        if run.bad_rows or missing > 0:
            rep.checks.append(Check("samples", "Sample count", WARN,
                                    f"{n} samples, {max(missing, 0)} missing, {run.bad_rows} malformed rows dropped"))
        else:
            rep.checks.append(Check("samples", "Sample count", PASS, f"{n} samples, none missing"))
    if n < 2:
        return rep

    # --- timing
    dt = np.diff(run.t_us).astype(float)
    if np.any(dt <= 0):
        rep.checks.append(Check("timing", "Sampling period", FAIL, "timestamps are not strictly increasing"))
    else:
        mean_dt = float(dt.mean())
        med_dt = float(np.median(dt))
        jitter = float(np.max(np.abs(dt - med_dt)) / med_dt * 100.0)
        m.update(ts_mean_ms=mean_dt / 1000.0, ts_max_ms=float(dt.max()) / 1000.0, jitter_pct=jitter)
        st = PASS if jitter < JITTER_WARN_PCT else WARN if jitter < JITTER_FAIL_PCT else FAIL
        rep.checks.append(Check(
            "timing", "Sampling period", st,
            f"mean {mean_dt / 1000:.3f} ms, max {dt.max() / 1000:.3f} ms, peak jitter {jitter:.2f} %"))

    # --- channel ranges + clipping
    for key, label, y in (("jib", "Jib", run.jib), ("cable", "Cable", run.cable)):
        lo, hi = int(y.min()), int(y.max())
        m[f"{key}_min"], m[f"{key}_max"] = lo, hi
        clipped = int(np.count_nonzero((y <= 0) | (y >= ADC_MAX)))
        m[f"{key}_clipped"] = clipped
        if clipped == 0:
            st, txt = PASS, f"range {lo}..{hi}, no clipping"
        elif clipped / n >= CLIP_FAIL_FRACTION:
            st, txt = FAIL, f"range {lo}..{hi}, {clipped} samples at 0/4095 (saturated)"
        else:
            st, txt = WARN, f"range {lo}..{hi}, {clipped} samples at 0/4095"
        rep.checks.append(Check(f"clip_{key}", f"{label} clipping", st, txt))

    # --- response and rest state
    moving = np.flatnonzero(run.u_pct != 0)
    for key, label, y in (("jib", "Jib", run.jib), ("cable", "Cable", run.cable)):
        p2p = int(y.max() - y.min())
        m[f"{key}_p2p"] = p2p
        if p2p < FLAT_P2P:
            st, txt = FAIL, f"only {p2p} counts of movement: no response (check wiring and amplitude)"
        elif p2p < WEAK_P2P:
            st, txt = WARN, f"{p2p} counts of movement: weak response"
        else:
            st, txt = PASS, f"{p2p} counts of movement"
        if moving.size and moving[0] >= 10 and st != FAIL:
            pre = y[: moving[0]]
            pre_p2p = int(pre.max() - pre.min())
            m[f"{key}_rest_p2p"] = pre_p2p
            if pre_p2p > REST_P2P:
                st = _worst(st, WARN)
                txt += f"; not at rest before input ({pre_p2p} counts)"
        rep.checks.append(Check(f"resp_{key}", f"{label} response", st, txt))
    if not moving.size:
        rep.checks.append(Check("input", "Input signal", WARN, "u is zero for the whole run"))
    return rep
