"""Data model for one acquired run."""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

ADC_VREF = 3.3
ADC_MAX = 4095


@dataclass
class Run:
    kind: str
    k: np.ndarray
    t_us: np.ndarray
    u_pct: np.ndarray
    jib: np.ndarray
    cable: np.ndarray
    name: str = ""
    bad_rows: int = 0
    path: str | None = None
    saved: bool = False
    source: str = "device"
    quality: object = field(default=None, repr=False)

    @classmethod
    def from_rows(cls, kind: str, rows, bad_rows: int = 0, **kw) -> "Run":
        a = np.array(rows, dtype=float).reshape(-1, 5)
        return cls(
            kind=kind,
            k=a[:, 0].astype(np.int64),
            t_us=a[:, 1].astype(np.int64),
            u_pct=a[:, 2],
            jib=a[:, 3].astype(np.int64),
            cable=a[:, 4].astype(np.int64),
            bad_rows=bad_rows,
            **kw,
        )

    @property
    def n(self) -> int:
        return int(self.k.size)

    @property
    def t_s(self) -> np.ndarray:
        return (self.t_us - self.t_us[0]) * 1e-6 if self.n else np.zeros(0)


def counts_to_volts(counts) -> np.ndarray:
    return np.asarray(counts, dtype=float) / ADC_MAX * ADC_VREF
