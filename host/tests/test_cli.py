from __future__ import annotations

import io
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import pytest

from nrflink import cli
from nrflink.messages import Battery, Command, KeyValue, KvEntry, KvType, encode_message
from nrflink.serial_frame import GatewayStats, SerialKind, encode_gateway_stats, encode_wire
from nrflink.svg import Series, line_chart


def capture() -> bytes:
    """A gateway output stream in the real wire format."""
    out = bytearray()
    out += encode_wire(SerialKind.LOG, 0, b"gateway ready")
    body = bytes((1, 1, 1)) + encode_message(Battery(7400, 0, 0, 0, 0xFFFF, 0x80))
    out += encode_wire(SerialKind.MESSAGE, 1, body)
    kv = KeyValue((KvEntry("q", KvType.INT, 97),))
    out += encode_wire(SerialKind.MESSAGE, 2, bytes((1, 1, 0)) + encode_message(kv))
    out += encode_wire(SerialKind.MESSAGE, 3, bytes((1, 1, 0)) + b"\x02\x01")
    stats = GatewayStats(uptime_ms=5000, frames_received=100, hop_locked=1, channel=44)
    out += encode_wire(SerialKind.LINK_STATS, 4, encode_gateway_stats(stats))
    out += encode_wire(SerialKind.COMMAND_RESULT, 6, bytes((9, 0)))
    out += encode_wire(0x55, 7, b"")
    out += encode_wire(SerialKind.LINK_STATS, 8, b"\x00")
    out += b"\x01\x02\x00"  # a corrupt frame
    return bytes(out)


def test_decode_capture(tmp_path: Path) -> None:
    f = tmp_path / "gw.bin"
    f.write_bytes(capture())
    csv_path = tmp_path / "t.csv"
    out = io.StringIO()
    assert cli.main(["decode", str(f), "--csv", str(csv_path)], out=out) == 0
    lines = out.getvalue().splitlines()
    assert lines[0] == "[  0] log: gateway ready"
    assert lines[1].startswith("[  1] tel pipe1 rpd battery 7.400 V")
    assert lines[2] == "[  2] tel pipe1 kv q=97"
    assert "undecodable message" in lines[3]
    assert "stats up=5.0s rx=100" in lines[4] and "hop=0@ch44 locked" in lines[4]
    assert lines[5] == "[  6] command seq=9 FAILED"
    assert lines[6].startswith("[  7] unknown kind 0x55")
    assert "bad link_stats body" in lines[7]
    assert lines[8] == "# 8 packets, rejected: cobs=1, serial seq gaps: 1"
    rows = csv_path.read_text(encoding="utf-8").splitlines()
    assert rows[0] == "host_time,serial_seq,channel,type,fields"
    assert len(rows) == 3 and ",battery," in rows[1] and ",key_value," in rows[2]
    out2 = io.StringIO()
    cli.main(["decode", str(f)], out=out2)
    assert out2.getvalue() == out.getvalue()


class FakePort:
    def __init__(self, incoming: bytes) -> None:
        self.incoming = io.BytesIO(incoming)
        self.written = bytearray()
        self.closed = False

    def read(self, n: int = 1) -> bytes:
        return self.incoming.read(n)

    def write(self, data: bytes) -> int:
        self.written += data
        return len(data)

    def close(self) -> None:
        self.closed = True


def test_send_waits_for_result(monkeypatch: pytest.MonkeyPatch) -> None:
    port = FakePort(encode_wire(SerialKind.COMMAND_RESULT, 0, bytes((3, 1))))
    monkeypatch.setattr(cli, "_open_serial", lambda p, b: port)
    out = io.StringIO()
    assert cli.main(["send", "--port", "X", "period", "100"], out=out) == 0
    assert port.written == encode_wire(SerialKind.SEND_COMMAND, 0, encode_message(Command(1, 100)))
    assert "delivered" in out.getvalue() and port.closed


def test_send_timeout_and_validation(monkeypatch: pytest.MonkeyPatch) -> None:
    port = FakePort(b"")
    monkeypatch.setattr(cli, "_open_serial", lambda p, b: port)
    out = io.StringIO()
    assert cli.main(["send", "--port", "X", "status", "--timeout", "0.05"], out=out) == 1
    assert "no command result" in out.getvalue()
    assert cli.main(["send", "--port", "X", "pa", "9"], out=io.StringIO()) == 2
    assert cli.main(["send", "--port", "X", "period"], out=io.StringIO()) == 2
    assert cli.build_command("identify", 500) == encode_message(Command(4, 500))


def test_monitor_with_duration(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    port = FakePort(capture())
    monkeypatch.setattr(cli, "_open_serial", lambda p, b: port)
    out = io.StringIO()
    csv_path = tmp_path / "m.csv"
    assert (
        cli.main(["monitor", "--port", "X", "--duration", "0.05", "--csv", str(csv_path)], out=out)
        == 0
    )
    assert "8 packets" in out.getvalue()
    assert csv_path.exists() and port.closed


def test_simulate_command() -> None:
    out = io.StringIO()
    args = ["simulate", "--loss", "0.2", "--burst", "5", "--duration", "4", "--wifi", "1"]
    assert cli.main(args, out=out) == 0
    text = out.getvalue()
    assert text.startswith("SIMULATED (not measured): burst loss 20%")
    assert "telemetry" in text and "gateway first locked" in text
    out = io.StringIO()
    cli.main(["simulate", "--no-hop", "--duration", "3", "--rate", "250", "--loss", "0.1"], out=out)
    assert "250 kbit/s" in out.getvalue() and "fixed channel 76" in out.getvalue()


def test_sweep_writes_svg(tmp_path: Path) -> None:
    svg = tmp_path / "s.svg"
    out = io.StringIO()
    assert (
        cli.main(["sweep", "--duration", "3", "--losses", "0", "0.4", "--svg", str(svg)], out=out)
        == 0
    )
    text = out.getvalue()
    assert text.count("| independent, ARC=15 |") == 2
    root = ET.fromstring(svg.read_text(encoding="utf-8"))
    assert root.tag.endswith("svg")
    content = svg.read_text(encoding="utf-8")
    assert "SIMULATED" in content and "Not measured on hardware" in content


def test_vectors_command(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    out = io.StringIO()
    assert cli.main(["vectors"], out=out) in (0, 2)
    target = tmp_path / "v.json"
    assert cli.main(["vectors", "--write", "--path", str(target)], out=io.StringIO()) == 0
    assert cli.main(["vectors", "--path", str(target)], out=io.StringIO()) == 0
    target.write_text("{}", encoding="utf-8")
    assert cli.main(["vectors", "--path", str(target)], out=io.StringIO()) == 1
    monkeypatch.chdir(tmp_path)
    assert cli.main(["vectors"], out=io.StringIO()) == 2


def test_repository_vectors_up_to_date(vectors: dict[str, Any]) -> None:
    from nrflink.vectors import build_vectors

    assert build_vectors() == vectors


def test_line_chart_is_valid_svg() -> None:
    svg = line_chart(
        [Series("a", [0, 1, 2], [0.0, 50.0, 100.0]), Series("b & c", [0, 2], [10.0, float("nan")])],
        title="t <x>",
        subtitle="sub",
        x_label="x",
        y_label="y",
    )
    root = ET.fromstring(svg)
    texts = [t.text for t in root.iter("{http://www.w3.org/2000/svg}text")]
    assert "t <x>" in texts and "b & c" in texts
