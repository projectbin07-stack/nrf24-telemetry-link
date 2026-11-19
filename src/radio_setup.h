// Radio and link configuration shared by node_main.cpp and gateway_main.cpp,
// built from the NTL_* macros in include/board_config.h.
#pragma once

#include <board_config.h>
#include <linkcore/hopping.h>
#include <linkcore/link.h>
#include <linkcore/nrf24.h>

inline linkcore::Nrf24Config make_radio_config(const linkcore::HopSequence& hops) {
  linkcore::Nrf24Config c;
  c.channel = NTL_HOPPING ? hops.channel(0) : NTL_FIXED_CHANNEL;
  c.data_rate = static_cast<linkcore::DataRate>(NTL_DATA_RATE);
  c.pa_level = static_cast<linkcore::PaLevel>(NTL_PA_LEVEL);
  c.crc = linkcore::CrcLength::k16Bit;
  c.address_width = 5;
  c.retransmit_delay = NTL_ARD_CODE;
  c.retransmit_count = NTL_ARC;
  c.dynamic_payloads = true;
  c.ack_payloads = true;
  c.dynamic_ack = true;
  return c;
}

inline linkcore::LinkConfig make_link_config() {
  linkcore::LinkConfig c;
  c.reliable_timeout_ms = NTL_RELIABLE_TIMEOUT_MS;
  c.reliable_max_attempts = NTL_RELIABLE_ATTEMPTS;
  return c;
}

inline void make_hop_sequence(linkcore::HopSequence* hops) {
  hops->generate(NTL_HOP_SEED, NTL_HOP_FIRST, NTL_HOP_LAST, NTL_HOP_SPACING, NTL_HOP_COUNT);
}
