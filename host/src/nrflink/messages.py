"""Application message schema, mirror of ``lib/linkcore/src/linkcore/messages.h``.

``message = version:u8 (=1) type:u8 body``; little-endian fixed-point fields.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Any

SCHEMA_VERSION = 1

KV_MAX_KEY = 15
KV_MAX_STRING = 32
KV_MAX_ENTRIES = 15

BATTERY_FLAG_OVERFLOW = 0x01
BATTERY_FLAG_NO_CURRENT_SENSOR = 0x80
SOC_UNKNOWN = 0xFFFF


class MessageError(ValueError):
    """Raised for bytes that are not a valid message of this schema."""


class MsgType(IntEnum):
    BATTERY = 0x01
    ODOMETRY = 0x02
    IMU = 0x03
    KEY_VALUE = 0x04
    COMMAND = 0x10


class CommandOp(IntEnum):
    SET_TELEMETRY_PERIOD_MS = 1
    SET_PA_LEVEL = 2
    REQUEST_STATUS = 3
    IDENTIFY = 4


class KvType(IntEnum):
    INT = 0
    FLOAT = 1
    BOOL = 2
    STRING = 3


@dataclass(frozen=True)
class Battery:
    bus_mv: int
    current_ua: int = 0
    power_uw: int = 0
    charge_uah: int = 0
    soc_permille: int = 0
    flags: int = 0


@dataclass(frozen=True)
class Odometry:
    t_ms: int
    x_mm: int
    y_mm: int
    theta_mrad: int
    v_mm_s: int
    w_mrad_s: int


@dataclass(frozen=True)
class Imu:
    t_ms: int
    accel_mg: tuple[int, int, int]
    gyro_cdps: tuple[int, int, int]
    temp_cc: int


@dataclass(frozen=True)
class Command:
    opcode: int
    arg: int = 0


KvValue = int | float | bool | str


@dataclass(frozen=True)
class KvEntry:
    key: str
    type: KvType
    value: KvValue


@dataclass(frozen=True)
class KeyValue:
    entries: tuple[KvEntry, ...] = field(default_factory=tuple)

    def get(self, key: str) -> KvValue | None:
        for e in self.entries:
            if e.key == key:
                return e.value
        return None


Message = Battery | Odometry | Imu | Command | KeyValue

_BODY: dict[MsgType, str] = {
    MsgType.BATTERY: "<HiiiHB",
    MsgType.ODOMETRY: "<Iiihhh",
    MsgType.IMU: "<I3h3hh",
    MsgType.COMMAND: "<Bi",
}


def _header(t: MsgType) -> bytes:
    return bytes((SCHEMA_VERSION, int(t)))


def _valid_key(key: str) -> bytes:
    raw = key.encode("ascii", errors="strict")
    if not 1 <= len(raw) <= KV_MAX_KEY or any(c < 0x21 or c > 0x7E for c in raw):
        raise MessageError(f"invalid key {key!r}")
    return raw


def encode_message(m: Message) -> bytes:
    if isinstance(m, Battery):
        return _header(MsgType.BATTERY) + struct.pack(
            _BODY[MsgType.BATTERY],
            m.bus_mv,
            m.current_ua,
            m.power_uw,
            m.charge_uah,
            m.soc_permille,
            m.flags,
        )
    if isinstance(m, Odometry):
        return _header(MsgType.ODOMETRY) + struct.pack(
            _BODY[MsgType.ODOMETRY], m.t_ms, m.x_mm, m.y_mm, m.theta_mrad, m.v_mm_s, m.w_mrad_s
        )
    if isinstance(m, Imu):
        return _header(MsgType.IMU) + struct.pack(
            _BODY[MsgType.IMU], m.t_ms, *m.accel_mg, *m.gyro_cdps, m.temp_cc
        )
    if isinstance(m, Command):
        return _header(MsgType.COMMAND) + struct.pack(_BODY[MsgType.COMMAND], m.opcode, m.arg)
    if len(m.entries) > KV_MAX_ENTRIES:
        raise MessageError("too many key/value entries")
    out = bytearray(_header(MsgType.KEY_VALUE))
    out.append(len(m.entries))
    for e in m.entries:
        key = _valid_key(e.key)
        out.append(len(key))
        out += key
        out.append(int(e.type))
        if e.type == KvType.INT:
            out += struct.pack("<i", int(e.value))
        elif e.type == KvType.FLOAT:
            out += struct.pack("<f", float(e.value))
        elif e.type == KvType.BOOL:
            out.append(1 if e.value else 0)
        else:
            raw = str(e.value).encode("utf-8")
            if len(raw) > KV_MAX_STRING:
                raise MessageError("string value longer than 32 bytes")
            out.append(len(raw))
            out += raw
    return bytes(out)


def message_type(data: bytes) -> MsgType:
    if len(data) < 2:
        raise MessageError("message shorter than its 2-byte header")
    if data[0] != SCHEMA_VERSION:
        raise MessageError(f"unsupported schema version {data[0]}")
    try:
        return MsgType(data[1])
    except ValueError:
        raise MessageError(f"unknown message type 0x{data[1]:02x}") from None


def _decode_kv(data: bytes) -> KeyValue:
    if len(data) < 3 or data[2] > KV_MAX_ENTRIES:
        raise MessageError("bad key/value count")
    count = data[2]
    pos = 3
    entries: list[KvEntry] = []
    n = len(data)
    for _ in range(count):
        if pos >= n:
            raise MessageError("key/value message truncated")
        klen = data[pos]
        pos += 1
        if not 1 <= klen <= KV_MAX_KEY or pos + klen + 1 > n:
            raise MessageError("bad key length")
        raw = data[pos : pos + klen]
        if any(c < 0x21 or c > 0x7E for c in raw):
            raise MessageError("non-printable key")
        key = raw.decode("ascii")
        pos += klen
        tag = data[pos]
        pos += 1
        value: KvValue
        if tag in (KvType.INT, KvType.FLOAT):
            if pos + 4 > n:
                raise MessageError("value truncated")
            if tag == KvType.INT:
                value = struct.unpack_from("<i", data, pos)[0]
            else:
                value = struct.unpack_from("<f", data, pos)[0]
            pos += 4
        elif tag == KvType.BOOL:
            if pos + 1 > n or data[pos] > 1:
                raise MessageError("bad bool value")
            value = data[pos] == 1
            pos += 1
        elif tag == KvType.STRING:
            if pos + 1 > n:
                raise MessageError("value truncated")
            slen = data[pos]
            pos += 1
            if slen > KV_MAX_STRING or pos + slen > n:
                raise MessageError("bad string length")
            value = data[pos : pos + slen].decode("utf-8", errors="replace")
            pos += slen
        else:
            raise MessageError(f"unknown value tag {tag}")
        entries.append(KvEntry(key, KvType(tag), value))
    if pos != n:
        raise MessageError("trailing bytes after key/value entries")
    return KeyValue(tuple(entries))


def decode_message(data: bytes) -> Message:
    data = bytes(data)
    t = message_type(data)
    if t == MsgType.KEY_VALUE:
        return _decode_kv(data)
    fmt = _BODY[t]
    if len(data) != 2 + struct.calcsize(fmt):
        raise MessageError(f"{t.name.lower()} must be {2 + struct.calcsize(fmt)} bytes")
    v = struct.unpack_from(fmt, data, 2)
    if t == MsgType.BATTERY:
        return Battery(*v)
    if t == MsgType.ODOMETRY:
        return Odometry(*v)
    if t == MsgType.IMU:
        return Imu(v[0], (v[1], v[2], v[3]), (v[4], v[5], v[6]), v[7])
    return Command(*v)


def message_name(m: Message) -> str:
    return {
        Battery: "battery",
        Odometry: "odometry",
        Imu: "imu",
        Command: "command",
        KeyValue: "key_value",
    }[type(m)]


def message_to_dict(m: Message) -> dict[str, Any]:
    """Plain representation used by the CLI and by the shared vectors."""
    if isinstance(m, KeyValue):
        return {
            "entries": [
                {"key": e.key, "type": e.type.name.lower(), "value": e.value} for e in m.entries
            ]
        }
    if isinstance(m, Imu):
        return {
            "t_ms": m.t_ms,
            "accel_mg": list(m.accel_mg),
            "gyro_cdps": list(m.gyro_cdps),
            "temp_cc": m.temp_cc,
        }
    return dict(m.__dict__)


def message_from_dict(name: str, d: dict[str, Any]) -> Message:
    if name == "battery":
        return Battery(**d)
    if name == "odometry":
        return Odometry(**d)
    if name == "imu":
        a, g = d["accel_mg"], d["gyro_cdps"]
        return Imu(d["t_ms"], (a[0], a[1], a[2]), (g[0], g[1], g[2]), d["temp_cc"])
    if name == "command":
        return Command(**d)
    if name == "key_value":
        return KeyValue(
            tuple(KvEntry(e["key"], KvType[e["type"].upper()], e["value"]) for e in d["entries"])
        )
    raise MessageError(f"unknown message name {name!r}")


def describe(m: Message) -> str:
    """One-line human-readable rendering."""
    if isinstance(m, Battery):
        soc = "?" if m.soc_permille == SOC_UNKNOWN else f"{m.soc_permille / 10:.1f}%"
        return (
            f"battery {m.bus_mv / 1000:.3f} V {m.current_ua / 1000:.1f} mA "
            f"{m.power_uw / 1000:.1f} mW soc={soc} flags=0x{m.flags:02x}"
        )
    if isinstance(m, Odometry):
        return (
            f"odometry t={m.t_ms} ms x={m.x_mm} mm y={m.y_mm} mm "
            f"theta={m.theta_mrad / 1000:.3f} rad v={m.v_mm_s} mm/s w={m.w_mrad_s} mrad/s"
        )
    if isinstance(m, Imu):
        return (
            f"imu t={m.t_ms} ms accel_mg={list(m.accel_mg)} "
            f"gyro_dps={[g / 100 for g in m.gyro_cdps]} temp={m.temp_cc / 100:.2f} C"
        )
    if isinstance(m, Command):
        try:
            op = CommandOp(m.opcode).name.lower()
        except ValueError:
            op = f"op{m.opcode}"
        return f"command {op} arg={m.arg}"
    parts = []
    for e in m.entries:
        v = e.value
        if isinstance(v, float) and math.isfinite(v):
            parts.append(f"{e.key}={v:g}")
        else:
            parts.append(f"{e.key}={v}")
    return "kv " + " ".join(parts)
