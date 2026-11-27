/* Cross-path capture: one chain transmits a probe frame while the other chain
 * listens through the logic-analyzer capture. Types and helpers shared by
 * callers; only the 8822E implements the session (jaguar3/CrossPath8822e.h). */
#ifndef CROSS_PATH_CAPTURE_H
#define CROSS_PATH_CAPTURE_H

#include <cstdint>
#include <vector>

#include "SelectedChannel.h" // ChannelWidth_t

namespace devourer {

struct IqSample {
  int16_t i = 0, q = 0;
  bool bad = false;
};

/* One capture word of the path-B tap: I = bits 55:44, Q = bits 43:32, 12-bit
 * two's complement; bit 61 clear marks a bad I sample. */
IqSample cross_path_sample(uint64_t la_word);

constexpr int kCrossPathGains = 4;
/* Listener RX gain words (RF 0x00 of the listening path), weakest first by
 * measured level. */
constexpr uint32_t kCrossPathGainWord[kCrossPathGains] = {0x3000e, 0x30800,
                                                          0x300c0, 0x30c00};

/* Capture length: 8192 samples at 20 MHz, 16384 at 40. */
uint32_t cross_path_samples(ChannelWidth_t bw);

/* Radiotap (HT MCS1, one stream, at bw) + 24-byte data header + a
 * deterministic pseudo-random body (150 bytes at 20 MHz, 320 at 40). */
std::vector<uint8_t> cross_path_probe_frame(ChannelWidth_t bw);

} // namespace devourer

#endif
