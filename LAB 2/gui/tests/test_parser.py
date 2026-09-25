import numpy as np
import pytest

from core.parser import HEADER, DataFormatError, StreamParser, parse_csv_text, parse_info, parse_row
from core.simulator import SimulatedSerial, synthesize


def test_parse_row_ok():
    assert parse_row("3,30000,30.0,2048,2051") == (3, 30000, 30.0, 2048, 2051)


@pytest.mark.parametrize("line", [
    "", "1,2,3", "a,1,0.0,1,1", "1,100,0.0,4096,10", "1,100,0.0,-1,10", "1,100,nan,10,10",
    "-1,100,0.0,10,10", "1,100,0.0,10,10,extra",
])
def test_parse_row_rejects(line):
    assert parse_row(line) is None


def test_parse_info():
    assert parse_info("# Ts=10000 us, U=30.0%, max=50.0%") == {"ts_us": 10000, "u_pct": 30.0, "max_pct": 50.0}
    assert parse_info("# Ready") is None


def test_stream_events():
    p = StreamParser()
    assert p.feed_line("# Ready. ok") == ("comment", "Ready. ok")
    assert p.feed_line("0,0,0.0,1,1") == ("stray", "0,0,0.0,1,1")
    assert p.feed_line(HEADER) == ("header", None)
    assert p.feed_line("0,0,0.0,10,20")[0] == "row"
    assert p.feed_line("1,10000,0.0,garbage")[0] == "bad"
    assert p.feed_line("   ") is None
    assert p.feed_line("# DONE") == ("done", None)
    assert len(p.rows) == 1 and p.bad == 1


def test_aborted():
    assert StreamParser().feed_line("# ABORTED") == ("aborted", None)


def test_parse_csv_text_strips_comments():
    text = f"# hello\n{HEADER}\n0,0,0.0,1,2\n1,10000,0.0,3,4\n# DONE\n"
    rows, bad = parse_csv_text(text)
    assert len(rows) == 2 and bad == 0


def test_parse_csv_text_errors():
    with pytest.raises(DataFormatError):
        parse_csv_text("0,0,0.0,1,2\n")
    with pytest.raises(DataFormatError):
        parse_csv_text(HEADER + "\n")


def test_fake_serial_stream_roundtrip():
    """Feed the simulated device's stream through the parser."""
    lines = synthesize("step", np.random.default_rng(1))
    p = StreamParser()
    for ln in lines + ["# DONE"]:
        p.feed_line(ln)
    assert len(p.rows) == 1600 and p.bad == 0
    ser = SimulatedSerial(time_scale=1e6)
    ser.write(b"i")
    assert b"Ts=10000" in ser.read(ser.in_waiting)
