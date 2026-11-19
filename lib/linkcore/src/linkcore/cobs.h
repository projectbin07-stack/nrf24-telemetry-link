// Consistent Overhead Byte Stuffing (Cheshire & Baker, 1999).
//
// COBS removes every 0x00 from a packet so that 0x00 can delimit frames on
// the gateway's USB serial line. Overhead is one byte per 254 bytes plus one.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace linkcore {

/// Worst-case encoded size (without the 0x00 delimiter).
constexpr size_t cobs_max_encoded_size(size_t length) { return length + length / 254u + 1u; }

/// Encode `length` bytes into `out`. Returns the encoded size, or 0 if
/// `capacity` is too small (a valid encoding is never empty).
size_t cobs_encode(const uint8_t* in, size_t length, uint8_t* out, size_t capacity);

/// Decode one COBS block (delimiter already removed). Returns false on
/// malformed input (a 0x00 byte, a code byte pointing past the end) or if
/// `capacity` is too small; otherwise stores the decoded size.
bool cobs_decode(const uint8_t* in, size_t length, uint8_t* out, size_t capacity,
                 size_t* out_length);

}  // namespace linkcore
