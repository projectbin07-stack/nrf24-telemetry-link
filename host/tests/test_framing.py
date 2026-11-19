from __future__ import annotations

import random
from typing import Any

import pytest

from nrflink.framing import CobsError, cobs_decode, cobs_encode, crc16_ccitt


def test_crc_check_value() -> None:
    # Published check value of CRC-16/CCITT-FALSE.
    assert crc16_ccitt(b"123456789") == 0x29B1
    assert crc16_ccitt(b"") == 0xFFFF


def test_crc_chaining() -> None:
    assert crc16_ccitt(b"6789", crc16_ccitt(b"12345")) == crc16_ccitt(b"123456789")


@pytest.mark.parametrize(
    ("raw", "encoded"),
    [
        (b"", b"\x01"),
        (b"\x00", b"\x01\x01"),
        (b"\x00\x00", b"\x01\x01\x01"),
        (b"\x11\x22\x00\x33", b"\x03\x11\x22\x02\x33"),
        (b"\x11\x22\x33\x44", b"\x05\x11\x22\x33\x44"),
        (b"\x11\x00\x00\x00", b"\x02\x11\x01\x01\x01"),
    ],
)
def test_cobs_reference_examples(raw: bytes, encoded: bytes) -> None:
    # Worked examples from Cheshire & Baker / the COBS Wikipedia article.
    assert cobs_encode(raw) == encoded
    assert cobs_decode(encoded) == raw


def test_cobs_long_blocks() -> None:
    data = bytes(range(1, 256))
    enc = cobs_encode(data)
    assert enc[0] == 0xFF and enc[255] == 0x02
    assert cobs_decode(enc) == data
    assert cobs_encode(bytes(range(1, 255)))[0] == 0xFF
    assert len(cobs_encode(bytes(range(1, 255)))) == 255


def test_cobs_random_round_trip() -> None:
    rng = random.Random(7)
    for _ in range(300):
        data = bytes(rng.choice((0, rng.randrange(256))) for _ in range(rng.randrange(600)))
        enc = cobs_encode(data)
        assert 0 not in enc
        assert cobs_decode(enc) == data


@pytest.mark.parametrize("bad", [b"", b"\x00", b"\x05\x01\x02", b"\x03\x01\x00"])
def test_cobs_rejects_malformed(bad: bytes) -> None:
    with pytest.raises(CobsError):
        cobs_decode(bad)


def test_shared_vectors(vectors: dict[str, Any]) -> None:
    for case in vectors["crc16"]:
        assert crc16_ccitt(case["ascii"].encode()) == int(case["crc"], 16)
    for case in vectors["cobs"]:
        assert cobs_encode(bytes.fromhex(case["raw"])).hex() == case["encoded"]
