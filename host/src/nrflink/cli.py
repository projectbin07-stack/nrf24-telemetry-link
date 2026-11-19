"""Command-line interface: ``nrflink <command> ...``."""

from __future__ import annotations

import argparse
import csv
import sys
import time
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import IO, Any, Protocol

from . import __version__
from .messages import (
    Command,
    CommandOp,
    MessageError,
    decode_message,
    describe,
    encode_message,
    message_name,
    message_to_dict,
)
from .serial_frame import (
    FrameError,
    SerialKind,
    SerialPacket,
    StreamDecoder,
    decode_command_result,
    decode_gateway_stats,
    decode_message_body,
    encode_wire,
)
from .sim import (
    Bernoulli,
    GilbertElliott,
    Interference,
    LossModel,
    NoLoss,
    SimConfig,
    describe_config,
    simulate,
    summary_line,
    sweep,
)
from .svg import Series, line_chart
from .vectors import VECTORS_RELATIVE_PATH, build_vectors, find_repo_file, render


class ByteSource(Protocol):
    def read(self, size: int = 1) -> bytes: ...


def describe_packet(pkt: SerialPacket) -> str:
    """One line per gateway packet, as printed by ``nrflink monitor``."""
    try:
        kind = SerialKind(pkt.kind)
    except ValueError:
        return f"[{pkt.seq:3d}] unknown kind 0x{pkt.kind:02x} body={pkt.body.hex()}"
    try:
        if kind == SerialKind.MESSAGE:
            ev = decode_message_body(pkt.body)
            chan = "rel" if ev.channel == 0 else "tel"
            rpd = " rpd" if ev.rpd else ""
            try:
                text = describe(decode_message(ev.message))
            except MessageError as exc:
                text = f"undecodable message ({exc}): {ev.message.hex()}"
            return f"[{pkt.seq:3d}] {chan} pipe{ev.pipe}{rpd} {text}"
        if kind == SerialKind.LINK_STATS:
            s = decode_gateway_stats(pkt.body)
            lock = "locked" if s.hop_locked else "scanning"
            return (
                f"[{pkt.seq:3d}] stats up={s.uptime_ms / 1000:.1f}s rx={s.frames_received} "
                f"tel={s.delivered_unreliable} lost={s.unreliable_lost} dup={s.duplicates} "
                f"crc={s.crc_errors} bad={s.malformed} cmd={s.reliable_acked}/{s.reliable_sent} "
                f"failed={s.reliable_failed} retries={s.reliable_retries} "
                f"hop={s.hop_index}@ch{s.channel} {lock} resyncs={s.resyncs} rpd={s.rpd_percent}%"
            )
        if kind == SerialKind.COMMAND_RESULT:
            seq, ok = decode_command_result(pkt.body)
            return f"[{pkt.seq:3d}] command seq={seq} {'delivered' if ok else 'FAILED'}"
        if kind == SerialKind.LOG:
            return f"[{pkt.seq:3d}] log: {pkt.body.decode('ascii', errors='replace')}"
    except FrameError as exc:
        return f"[{pkt.seq:3d}] bad {kind.name.lower()} body: {exc}"
    return f"[{pkt.seq:3d}] {kind.name.lower()} body={pkt.body.hex()}"


class CsvLog:
    """Writes decoded telemetry messages as CSV rows (one column set per row type)."""

    def __init__(self, stream: IO[str]) -> None:
        self.writer = csv.writer(stream)
        self.writer.writerow(["host_time", "serial_seq", "channel", "type", "fields"])

    def write(self, pkt: SerialPacket) -> None:
        if pkt.kind != SerialKind.MESSAGE:
            return
        try:
            ev = decode_message_body(pkt.body)
            msg = decode_message(ev.message)
        except (FrameError, MessageError):
            return
        fields = message_to_dict(msg)
        flat = " ".join(f"{k}={v}" for k, v in fields.items())
        self.writer.writerow([f"{time.time():.3f}", pkt.seq, ev.channel, message_name(msg), flat])


def run_monitor(
    source: ByteSource,
    out: IO[str],
    *,
    duration_s: float | None = None,
    csv_log: CsvLog | None = None,
    clock: Callable[[], float] = time.monotonic,
) -> StreamDecoder:
    decoder = StreamDecoder()
    start = clock()
    while duration_s is None or clock() - start < duration_s:
        chunk = source.read(256)
        if not chunk:
            if duration_s is None:
                break  # end of file
            continue
        for pkt in decoder.feed(chunk):
            print(describe_packet(pkt), file=out)
            if csv_log is not None:
                csv_log.write(pkt)
    errors = ", ".join(f"{k}={v}" for k, v in sorted(decoder.errors.items())) or "none"
    print(
        f"# {decoder.packets} packets, rejected: {errors}, serial seq gaps: {decoder.lost}",
        file=out,
    )
    return decoder


def build_command(name: str, arg: int | None) -> bytes:
    ops = {
        "period": CommandOp.SET_TELEMETRY_PERIOD_MS,
        "pa": CommandOp.SET_PA_LEVEL,
        "status": CommandOp.REQUEST_STATUS,
        "identify": CommandOp.IDENTIFY,
    }
    op = ops[name]
    if op in (CommandOp.SET_TELEMETRY_PERIOD_MS, CommandOp.SET_PA_LEVEL) and arg is None:
        raise ValueError(f"'{name}' needs a value")
    if op == CommandOp.SET_TELEMETRY_PERIOD_MS and not 20 <= (arg or 0) <= 60000:
        raise ValueError("period must be 20..60000 ms")
    if op == CommandOp.SET_PA_LEVEL and not 0 <= (arg or 0) <= 3:
        raise ValueError("pa level must be 0..3")
    return encode_message(Command(int(op), arg or 0))


def _open_serial(port: str, baud: int) -> Any:
    import serial  # pyserial

    return serial.Serial(port, baud, timeout=0.1)


def _loss_model(args: argparse.Namespace) -> LossModel:
    if args.loss <= 0:
        return NoLoss()
    if args.burst is not None:
        return GilbertElliott(args.loss, args.burst)
    return Bernoulli(args.loss)


def cmd_simulate(args: argparse.Namespace, out: IO[str]) -> int:
    cfg = SimConfig(
        duration_s=args.duration,
        seed=args.seed,
        loss=_loss_model(args),
        hopping=not args.no_hop,
        fixed_channel=args.channel,
        telemetry_period_ms=args.period,
        command_period_ms=args.command_period,
        gateway_boot_ms=args.gateway_boot_ms,
    )
    cfg.radio.arc = args.arc
    cfg.radio.ard_us = args.ard
    cfg.radio.data_rate_bps = {250: 250_000, 1000: 1_000_000, 2000: 2_000_000}[args.rate]
    if args.wifi is not None:
        cfg.interference = Interference.wifi(args.wifi, args.wifi_loss)
    r = simulate(cfg)
    print("SIMULATED (not measured): " + describe_config(cfg), file=out)
    print(summary_line(r), file=out)
    if r.first_lock_ms is not None:
        print(f"gateway first locked at {r.first_lock_ms:.1f} ms", file=out)
    return 0


DEFAULT_SWEEP = (0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7)


def cmd_sweep(args: argparse.Namespace, out: IO[str]) -> int:
    losses = tuple(args.losses) if args.losses else DEFAULT_SWEEP
    runs = [
        sweep(losses, burst=None, arc=15, duration_s=args.duration, seed=args.seed),
        sweep(losses, burst=10.0, arc=15, duration_s=args.duration, seed=args.seed),
        sweep(losses, burst=None, arc=3, duration_s=args.duration, seed=args.seed),
    ]
    print(
        f"SIMULATED (not measured): {args.duration:g} s per point, seed {args.seed}, "
        "1 Mbit/s, ARD 500 us, telemetry every 50 ms, a command every 503 ms, "
        "8-channel hopping",
        file=out,
    )
    print(
        "| loss model | per-packet loss | telemetry delivered | goodput kbit/s "
        "| commands delivered | command latency p50 / p95 ms | ESB attempts per frame |",
        file=out,
    )
    print("|---|---|---|---|---|---|---|", file=out)
    for s in runs:
        for loss, r in zip(s.losses, s.results, strict=True):
            print(
                f"| {s.label} | {loss:.0%} | {100 * r.telemetry_delivery:.1f}% "
                f"| {r.goodput_bps / 1000:.2f} | {100 * r.command_delivery:.1f}% "
                f"| {r.latency_percentile(0.5):.1f} / {r.latency_percentile(0.95):.1f} "
                f"| {r.mean_attempts:.2f} |",
                file=out,
            )
    if args.svg:
        series = [
            Series(
                f"telemetry, {s.label}",
                [100 * x for x in s.losses],
                [100 * r.telemetry_delivery for r in s.results],
            )
            for s in runs
        ]
        series.append(
            Series(
                f"commands, {runs[1].label}",
                [100 * x for x in runs[1].losses],
                [100 * r.command_delivery for r in runs[1].results],
            )
        )
        svg = line_chart(
            series,
            title="SIMULATED delivery ratio vs per-packet loss (nrflink link simulator)",
            subtitle=(
                f"Not measured on hardware. {args.duration:g} s simulated per point, seed "
                f"{args.seed}, 1 Mbit/s, ARD 500 us, 8-channel hopping."
            ),
            x_label="per-packet loss probability (data and ACK packets), %",
            y_label="messages delivered, %",
            x_range=(0, 100 * max(losses)),
            y_range=(0, 100),
            x_format="{:g}%",
            y_format="{:g}%",
        )
        Path(args.svg).write_text(svg, encoding="utf-8")
        print(f"wrote {args.svg}", file=out)
    return 0


def cmd_vectors(args: argparse.Namespace, out: IO[str]) -> int:
    text = render(build_vectors())
    path = Path(args.path) if args.path else find_repo_file(VECTORS_RELATIVE_PATH)
    if args.write:
        target = path or Path(VECTORS_RELATIVE_PATH)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(text, encoding="utf-8", newline="\n")
        print(f"wrote {target}", file=out)
        return 0
    if path is None:
        print(f"{VECTORS_RELATIVE_PATH} not found (run from the repository)", file=out)
        return 2
    if path.read_text(encoding="utf-8") != text:
        print(f"{path} is out of date: run `nrflink vectors --write`", file=out)
        return 1
    print(f"{path} is up to date", file=out)
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="nrflink",
        description="Gateway decoder, command sender and link simulator for nrf24-telemetry-link.",
    )
    p.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    sub = p.add_subparsers(dest="command", required=True)

    m = sub.add_parser("monitor", help="decode and print gateway output from a serial port")
    m.add_argument("--port", required=True, help="serial port, e.g. COM5 or /dev/ttyUSB0")
    m.add_argument("--baud", type=int, default=115200)
    m.add_argument("--duration", type=float, default=None, help="stop after N seconds")
    m.add_argument("--csv", help="also write telemetry messages to this CSV file")

    d = sub.add_parser("decode", help="decode a binary capture of gateway output")
    d.add_argument("file")
    d.add_argument("--csv", help="also write telemetry messages to this CSV file")

    s = sub.add_parser("send", help="send a command to the node through the gateway")
    s.add_argument("--port", required=True)
    s.add_argument("--baud", type=int, default=115200)
    s.add_argument("name", choices=["period", "pa", "status", "identify"])
    s.add_argument("value", type=int, nargs="?")
    s.add_argument("--timeout", type=float, default=3.0, help="seconds to wait for the result")

    sim = sub.add_parser("simulate", help="run the link simulator once (SIMULATED results)")
    sim.add_argument("--loss", type=float, default=0.0, help="per-packet loss probability")
    sim.add_argument("--burst", type=float, default=None, help="Gilbert-Elliott mean burst length")
    sim.add_argument("--arc", type=int, default=15, help="ESB auto-retransmit count (0..15)")
    sim.add_argument("--ard", type=int, default=500, help="ESB auto-retransmit delay, us")
    sim.add_argument("--rate", type=int, choices=[250, 1000, 2000], default=1000, help="kbit/s")
    sim.add_argument("--duration", type=float, default=30.0, help="simulated seconds")
    sim.add_argument("--seed", type=int, default=1)
    sim.add_argument("--period", type=int, default=50, help="telemetry period, ms")
    sim.add_argument("--command-period", type=int, default=503, help="ms between commands")
    sim.add_argument("--no-hop", action="store_true", help="stay on --channel")
    sim.add_argument("--channel", type=int, default=76)
    sim.add_argument("--wifi", type=int, default=None, help="Wi-Fi channel (1-13) interfering")
    sim.add_argument("--wifi-loss", type=float, default=1.0, help="loss on Wi-Fi-covered channels")
    sim.add_argument("--gateway-boot-ms", type=int, default=0, help="gateway starts late")

    sw = sub.add_parser("sweep", help="delivery vs loss table and SVG plot (SIMULATED)")
    sw.add_argument("--duration", type=float, default=30.0, help="simulated seconds per point")
    sw.add_argument("--seed", type=int, default=1)
    sw.add_argument("--losses", type=float, nargs="*", help="loss probabilities to simulate")
    sw.add_argument("--svg", help="write the plot to this file")

    v = sub.add_parser("vectors", help="check (or --write) test/vectors/link_vectors.json")
    v.add_argument("--write", action="store_true")
    v.add_argument("--path", help="vector file (default: found from the working directory)")
    return p


def cmd_send(args: argparse.Namespace, out: IO[str]) -> int:
    try:
        msg = build_command(args.name, args.value)
    except ValueError as exc:
        print(f"error: {exc}", file=out)
        return 2
    port = _open_serial(args.port, args.baud)
    try:
        port.write(encode_wire(SerialKind.SEND_COMMAND, 0, msg))
        decoder = StreamDecoder()
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            for pkt in decoder.feed(port.read(256)):
                print(describe_packet(pkt), file=out)
                if pkt.kind == SerialKind.COMMAND_RESULT:
                    _, ok = decode_command_result(pkt.body)
                    return 0 if ok else 1
        print("no command result before the timeout", file=out)
        return 1
    finally:
        port.close()


def main(argv: Sequence[str] | None = None, out: IO[str] | None = None) -> int:
    out = out or sys.stdout
    args = build_parser().parse_args(argv)
    if args.command == "monitor":
        port = _open_serial(args.port, args.baud)
        try:
            if args.csv:
                with open(args.csv, "w", newline="", encoding="utf-8") as fh:
                    run_monitor(port, out, duration_s=args.duration, csv_log=CsvLog(fh))
            else:
                run_monitor(port, out, duration_s=args.duration)
        except KeyboardInterrupt:
            pass
        finally:
            port.close()
        return 0
    if args.command == "decode":
        with open(args.file, "rb") as src:
            if args.csv:
                with open(args.csv, "w", newline="", encoding="utf-8") as fh:
                    run_monitor(src, out, csv_log=CsvLog(fh))
            else:
                run_monitor(src, out)
        return 0
    if args.command == "send":
        return cmd_send(args, out)
    if args.command == "simulate":
        return cmd_simulate(args, out)
    if args.command == "sweep":
        return cmd_sweep(args, out)
    return cmd_vectors(args, out)
