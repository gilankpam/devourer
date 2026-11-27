#include "CrossPathCapture.h"

#include "RadiotapBuilder.h"

namespace devourer {

IqSample cross_path_sample(uint64_t w) {
  auto s12 = [](uint32_t v) {
    return static_cast<int16_t>((v & 0x800) ? int32_t(v) - 0x1000 : int32_t(v));
  };
  IqSample s;
  s.i = s12(static_cast<uint32_t>(w >> 44) & 0xfff);
  s.q = s12(static_cast<uint32_t>(w >> 32) & 0xfff);
  s.bad = !((w >> 61) & 1);
  return s;
}

uint32_t cross_path_samples(ChannelWidth_t bw) {
  return bw == CHANNEL_WIDTH_40 ? 16384 : 8192;
}

std::vector<uint8_t> cross_path_probe_frame(ChannelWidth_t bw) {
  TxMode m;
  m.mode = TxMode::Mode::HT;
  m.ht_mcs = 1;
  m.bw_mhz = bw == CHANNEL_WIDTH_40 ? 40 : 20;
  std::vector<uint8_t> f = build_stream_radiotap(m);
  static const uint8_t hdr[24] = {0x08, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff,
                                  0xff, 0xff, 0x57, 0x42, 0x75, 0x05, 0xd6, 0x00,
                                  0x57, 0x42, 0x75, 0x05, 0xd6, 0x00, 0x00, 0x00};
  f.insert(f.end(), hdr, hdr + sizeof(hdr));
  uint32_t x = 0x12345678; // full-constellation pseudo-random body
  const int n = bw == CHANNEL_WIDTH_40 ? 320 : 150;
  for (int i = 0; i < n; ++i) {
    x = x * 1664525u + 1013904223u;
    f.push_back(static_cast<uint8_t>(x >> 24));
  }
  return f;
}

} // namespace devourer
