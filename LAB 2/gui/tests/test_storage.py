import numpy as np

from core.models import Run
from core.parser import HEADER
from core.session import Session
from core.storage import load_run, next_name, save_run


def sample_run(kind="step"):
    rows = [(i, i * 10000, 30.0 if i > 2 else 0.0, 100 + i, 200 + i) for i in range(5)]
    return Run.from_rows(kind, rows)


def test_save_format_and_load(tmp_path):
    run = sample_run()
    run.name = "step_01"
    p = save_run(run, tmp_path)
    lines = p.read_text().splitlines()
    assert lines[0] == HEADER and lines[1] == "0,0,0.0,100,200" and lines[-1] == "4,40000,30.0,104,204"
    assert not any(ln.startswith("#") for ln in lines)
    back = load_run(p)
    assert back.kind == "step" and back.n == 5 and np.array_equal(back.jib, run.jib)


def test_auto_names(tmp_path):
    (tmp_path / "step_01.csv").write_text("x")
    assert next_name("step", tmp_path) == "step_02"
    assert next_name("prbs", tmp_path) == "prbs_01"
    assert next_name("prbs", tmp_path, taken=["prbs_01"]) == "prbs_02"


def test_save_never_overwrites(tmp_path):
    a, b = sample_run(), sample_run()
    a.name = b.name = "step_01"
    save_run(a, tmp_path)
    p = save_run(b, tmp_path)
    assert p.name == "step_02.csv" and b.name == "step_02"


def test_session_naming(tmp_path):
    s = Session(tmp_path)
    for r in (sample_run(), sample_run(), sample_run("prbs")):
        s.add(r)
    assert s.names() == ["step_01", "step_02", "prbs_01"]
    s.save(1)
    assert (tmp_path / "step_02.csv").exists()
