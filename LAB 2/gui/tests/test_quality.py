import numpy as np

from core.models import Run
from core.quality import FAIL, PASS, WARN, analyze


def make(n=1000, ts=10000, jib=None, cable=None, u=None, bad=0):
    t = np.arange(n) * ts
    if u is None:
        u = np.where(np.arange(n) >= 200, 30.0, 0.0)
    ramp = np.clip(np.arange(n) - 200, 0, None) * (800 / (n - 200))   # flat until the input starts
    jib = 1000 + ramp if jib is None else jib
    cable = 2000 + ramp if cable is None else cable
    rows = list(zip(range(n), t, u, np.asarray(jib, int), np.asarray(cable, int)))
    return Run.from_rows("step", rows, bad)


def status(rep, key):
    return next(c.status for c in rep.checks if c.key == key)


def test_good_run_passes():
    rep = analyze(make())
    assert rep.overall == PASS
    assert abs(rep.metrics["ts_mean_ms"] - 10.0) < 1e-6
    assert rep.metrics["jitter_pct"] < 1e-6


def test_jitter_levels():
    run = make()
    run.t_us[500:] += 300          # one 3 % long interval
    assert status(analyze(run), "timing") == WARN
    run.t_us[500:] += 4000
    assert status(analyze(run), "timing") == FAIL


def test_non_monotonic_time_fails():
    run = make()
    run.t_us[10] = run.t_us[9]
    assert status(analyze(run), "timing") == FAIL


def test_clipping_detected():
    y = np.full(1000, 4095)
    y[:100] = 2000
    assert status(analyze(make(jib=y)), "clip_jib") == FAIL
    y = np.linspace(0, 800, 1000).astype(int) + 1000
    y[500] = 4095
    assert status(analyze(make(jib=y)), "clip_jib") == WARN


def test_flat_and_weak_response():
    rep = analyze(make(jib=np.full(1000, 1500)))
    assert status(rep, "resp_jib") == FAIL and rep.overall == FAIL
    weak = 1500 + np.linspace(0, 100, 1000).astype(int)
    assert status(analyze(make(jib=weak)), "resp_jib") == WARN


def test_not_at_rest_warns():
    y = 1000 + np.clip(np.arange(1000) - 200, 0, None)
    y[:200] += np.tile([0, 100], 100)
    assert status(analyze(make(jib=y)), "resp_jib") == WARN


def test_too_few_samples_and_bad_rows():
    assert status(analyze(make(n=20)), "samples") == FAIL
    assert status(analyze(make(bad=2)), "samples") == WARN
    run = make()
    run.k[500:] += 3                # missing samples
    assert status(analyze(run), "samples") == WARN
