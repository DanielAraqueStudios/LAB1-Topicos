"""Pure parsing helpers for the ESP32 characterization protocol (no Qt, no serial)."""
from __future__ import annotations

import math
import re

HEADER = "k,t_us,u_pct,y_jib_raw,y_cable_raw"
ADC_MAX = 4095

_INFO_RE = re.compile(r"Ts=(\d+)\s*us,\s*U=([\d.]+)%,\s*max=([\d.]+)%")


class DataFormatError(ValueError):
    """Raised when a text blob does not contain a valid run."""


def parse_row(line: str):
    """Return (k, t_us, u_pct, jib, cable) or None when the row is malformed."""
    parts = line.strip().split(",")
    if len(parts) != 5:
        return None
    try:
        k, t_us = int(parts[0]), int(parts[1])
        u = float(parts[2])
        jib, cable = int(parts[3]), int(parts[4])
    except ValueError:
        return None
    if k < 0 or t_us < 0 or not math.isfinite(u):
        return None
    if not (0 <= jib <= ADC_MAX and 0 <= cable <= ADC_MAX):
        return None
    return (k, t_us, u, jib, cable)


def parse_info(line: str):
    """Parse '# Ts=10000 us, U=30.0%, max=50.0%' into a dict, else None."""
    m = _INFO_RE.search(line)
    if not m:
        return None
    return {"ts_us": int(m.group(1)), "u_pct": float(m.group(2)), "max_pct": float(m.group(3))}


class StreamParser:
    """Line-oriented state machine. Feed decoded lines, get (event, payload) tuples.

    Events: info, comment, header, row, bad, stray, done, aborted.
    """

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.rows: list[tuple] = []
        self.bad = 0
        self.in_data = False

    def feed_line(self, line: str):
        s = line.strip()
        if not s:
            return None
        if s.startswith("#"):
            body = s[1:].strip()
            if body.startswith("DONE"):
                return ("done", None)
            if body.startswith("ABORTED"):
                return ("aborted", None)
            info = parse_info(s)
            if info:
                return ("info", info)
            return ("comment", body)
        if s == HEADER:
            self.rows = []
            self.bad = 0
            self.in_data = True
            return ("header", None)
        if not self.in_data:
            return ("stray", s)
        row = parse_row(s)
        if row is None:
            self.bad += 1
            return ("bad", s)
        self.rows.append(row)
        return ("row", row)


def parse_csv_text(text: str):
    """Parse a saved CSV (or raw capture). Returns (rows, bad_count)."""
    p = StreamParser()
    for line in text.splitlines():
        p.feed_line(line)
    if not p.in_data:
        raise DataFormatError("missing header")
    if not p.rows:
        raise DataFormatError("no valid rows")
    return p.rows, p.bad
