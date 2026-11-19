// Little-endian load/store helpers (independent of the host byte order).
#pragma once

#include <stdint.h>
#include <string.h>

namespace linkcore {

inline void put_u16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>(v >> 8);
}

inline void put_u32(uint8_t* p, uint32_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
  p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}

inline uint16_t get_u16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8));
}

inline uint32_t get_u32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

inline void put_f32(uint8_t* p, float v) {
  static_assert(sizeof(float) == 4, "float must be IEEE-754 binary32");
  uint32_t bits;
  memcpy(&bits, &v, 4);
  put_u32(p, bits);
}

inline float get_f32(const uint8_t* p) {
  const uint32_t bits = get_u32(p);
  float v;
  memcpy(&v, &bits, 4);
  return v;
}

}  // namespace linkcore
