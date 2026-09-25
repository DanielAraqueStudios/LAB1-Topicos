"""CSV persistence for runs."""
from __future__ import annotations

import re
from pathlib import Path

from .models import Run
from .parser import HEADER, DataFormatError, parse_csv_text

_NAME_RE = re.compile(r"^(step|prbs)_(\d+)$")


def default_data_dir() -> Path:
    """`LAB 2/data` next to the gui folder."""
    return Path(__file__).resolve().parents[2] / "data"


def next_name(kind: str, folder: Path, taken=()) -> str:
    """step_01, step_02, ... skipping names used in `folder` or in `taken`."""
    used = {n for n in taken}
    if folder.is_dir():
        used |= {p.stem for p in folder.glob(f"{kind}_*.csv")}
    idx = 1
    while f"{kind}_{idx:02d}" in used:
        idx += 1
    return f"{kind}_{idx:02d}"


def save_run(run: Run, folder: Path, taken=()) -> Path:
    """Write the run as CSV (header + rows only). Never overwrites: renames on collision."""
    folder.mkdir(parents=True, exist_ok=True)
    name = run.name or next_name(run.kind, folder, taken)
    path = folder / f"{name}.csv"
    if path.exists():
        kind = run.kind if run.kind in ("step", "prbs") else "run"
        name = next_name(kind, folder, taken)
        path = folder / f"{name}.csv"
    lines = [HEADER]
    for k, t, u, a, b in zip(run.k, run.t_us, run.u_pct, run.jib, run.cable):
        lines.append(f"{int(k)},{int(t)},{float(u):.1f},{int(a)},{int(b)}")
    path.write_text("\n".join(lines) + "\n", encoding="ascii", newline="\n")
    run.name, run.path, run.saved = name, str(path), True
    return path


def load_run(path: Path) -> Run:
    """Load a CSV. Raises DataFormatError / OSError."""
    text = Path(path).read_text(encoding="utf-8", errors="replace")
    rows, bad = parse_csv_text(text)
    stem = Path(path).stem
    m = _NAME_RE.match(stem)
    kind = m.group(1) if m else ("prbs" if "prbs" in stem.lower() else "step" if "step" in stem.lower() else "file")
    return Run.from_rows(kind, rows, bad, name=stem, path=str(path), saved=True, source="file")


__all__ = ["default_data_dir", "next_name", "save_run", "load_run", "DataFormatError"]
