#include "linkcore/cobs.h"

namespace linkcore {

size_t cobs_encode(const uint8_t* in, size_t length, uint8_t* out, size_t capacity) {
  if (capacity == 0) {
    return 0;
  }
  size_t code_pos = 0;
  size_t write = 1;
  uint8_t code = 1;
  bool after_full_block = false;
  for (size_t i = 0; i < length; ++i) {
    after_full_block = false;
    if (in[i] == 0) {
      out[code_pos] = code;
      code = 1;
      code_pos = write++;
      if (code_pos >= capacity) {
        return 0;
      }
    } else {
      if (write >= capacity) {
        return 0;
      }
      out[write++] = in[i];
      ++code;
      if (code == 0xFF) {
        out[code_pos] = code;
        code = 1;
        after_full_block = true;
        if (i + 1 < length) {
          code_pos = write++;
          if (code_pos >= capacity) {
            return 0;
          }
        }
      }
    }
  }
  // A maximal block that ends the data carries no implicit zero, so no
  // further code byte follows it.
  if (!after_full_block) {
    out[code_pos] = code;
  }
  return write;
}

bool cobs_decode(const uint8_t* in, size_t length, uint8_t* out, size_t capacity,
                 size_t* out_length) {
  if (length == 0) {
    return false;
  }
  size_t read = 0;
  size_t write = 0;
  while (read < length) {
    const uint8_t code = in[read];
    if (code == 0 || read + code > length) {
      return false;
    }
    ++read;
    for (uint8_t i = 1; i < code; ++i) {
      if (in[read] == 0 || write >= capacity) {
        return false;
      }
      out[write++] = in[read++];
    }
    if (code != 0xFF && read < length) {
      if (write >= capacity) {
        return false;
      }
      out[write++] = 0;
    }
  }
  *out_length = write;
  return true;
}

}  // namespace linkcore
