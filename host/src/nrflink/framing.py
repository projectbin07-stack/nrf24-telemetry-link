"""CRC-16/CCITT-FALSE and COBS, identical to ``lib/linkcore`` (crc16.h, cobs.h)."""

from __future__ import annotations


class CobsError(ValueError):
    """Raised for input that is not valid COBS."""


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final XOR.

    ``crc16_ccitt(b"123456789") == 0x29B1``.
    """
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    """Consistent Overhead Byte Stuffing. The result contains no zero bytes."""
    out = bytearray()
    block = bytearray()
    after_full_block = False
    for byte in data:
        after_full_block = False
        if byte == 0:
            out.append(len(block) + 1)
            out += block
            block.clear()
        else:
            block.append(byte)
            if len(block) == 254:
                out.append(255)
                out += block
                block.clear()
                after_full_block = True
    if not after_full_block:
        out.append(len(block) + 1)
        out += block
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    """Inverse of :func:`cobs_encode`. Raises :class:`CobsError` on malformed input."""
    if not data:
        raise CobsError("empty frame")
    out = bytearray()
    i = 0
    n = len(data)
    while i < n:
        code = data[i]
        if code == 0:
            raise CobsError(f"zero byte at offset {i}")
        end = i + code
        if end > n:
            raise CobsError(f"block at offset {i} runs past the end of the frame")
        block = data[i + 1 : end]
        if 0 in block:
            raise CobsError("zero byte inside a block")
        out += block
        i = end
        if code != 255 and i < n:
            out.append(0)
    return bytes(out)
