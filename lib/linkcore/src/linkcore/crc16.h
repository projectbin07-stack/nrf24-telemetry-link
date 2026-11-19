// CRC-16/CCITT-FALSE: polynomial 0x1021, initial value 0xFFFF, no reflection,
// no final XOR. Check value: crc16_ccitt("123456789") == 0x29B1.
//
// Used twice in this project: end-to-end over every reassembled link-layer
// message, and over every gateway serial packet.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace linkcore {

uint16_t crc16_ccitt(const uint8_t* data, size_t length, uint16_t crc = 0xFFFFu);

}  // namespace linkcore
