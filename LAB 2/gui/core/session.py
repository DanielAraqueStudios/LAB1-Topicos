"""Session state: the runs acquired or loaded so far (Qt-free)."""
from __future__ import annotations

from pathlib import Path

from . import storage
from .models import Run
from .quality import analyze


class Session:
    def __init__(self, folder: Path | None = None) -> None:
        self.runs: list[Run] = []
        self.folder = folder or storage.default_data_dir()

    def names(self) -> list[str]:
        return [r.name for r in self.runs]

    def add(self, run: Run) -> int:
        """Analyse, auto-name (step_01, prbs_01, ...) and append. Returns the row index."""
        if not run.name:
            run.name = storage.next_name(run.kind, self.folder, self.names())
        run.quality = analyze(run)
        self.runs.append(run)
        return len(self.runs) - 1

    def save(self, index: int) -> Path:
        run = self.runs[index]
        others = [r.name for i, r in enumerate(self.runs) if i != index]
        return storage.save_run(run, self.folder, others)

    def load(self, path: Path) -> int:
        return self.add(storage.load_run(path))

    def remove(self, index: int) -> None:
        del self.runs[index]
