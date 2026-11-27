/* Headless guard for the cross-path types and the 8822E chain-B listener over
 * a recording fake transport that keeps register state. */
#include <cstdio>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "CrossPathCapture.h"
#include "RadiotapBuilder.h"
#include "Transport.h"
#include "jaguar3/CrossPath8822e.h"

namespace {

struct Fake : devourer::ITransport {
  std::vector<std::pair<uint16_t, uint32_t>> writes;
  std::map<uint16_t, uint32_t> regs;
  std::map<uint16_t, uint32_t> stuck; // bits that ignore writes

  uint32_t read32(uint16_t a) override { return regs[a]; }
  uint8_t read8(uint16_t a) override { return read32(a) & 0xff; }
  uint16_t read16(uint16_t a) override { return read32(a) & 0xffff; }
  bool write32(uint16_t a, uint32_t v) override {
    writes.push_back({a, v});
    const uint32_t m = stuck[a];
    regs[a] = (regs[a] & m) | (v & ~m);
    return true;
  }
  bool write8(uint16_t a, uint8_t v) override { return write32(a, v); }
  bool write16(uint16_t a, uint16_t v) override { return write32(a, v); }

  std::vector<uint32_t> to(uint16_t a) const {
    std::vector<uint32_t> r;
    for (auto &w : writes) if (w.first == a) r.push_back(w.second);
    return r;
  }

  bool is_usb() const override { return false; }
  bool write_bytes(uint16_t, const uint8_t *, size_t) override { return false; }
  bool tx_async(uint8_t, uint8_t *, size_t, unsigned) override { return false; }
  int tx_sync(uint8_t, uint8_t *, size_t, int) override { return -1; }
  void rx_loop(int, int, const std::function<void(const uint8_t *, int)> &,
               const std::function<bool()> &) override {}
};

int fails = 0;
#define CHECK(c)                                                               \
  do {                                                                         \
    if (!(c)) {                                                                \
      std::fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #c);         \
      ++fails;                                                                 \
    }                                                                          \
  } while (0)

} // namespace

int main() {
  using namespace devourer;
  auto s = cross_path_sample((uint64_t{1} << 61) | (uint64_t{0x7ff} << 44) | (uint64_t{0x800} << 32));
  CHECK(s.i == 2047 && s.q == -2048 && !s.bad);
  CHECK(cross_path_sample(uint64_t{0xfff} << 44).i == -1);
  CHECK(cross_path_sample(0).bad);

  TxMode m20, m40;
  m20.mode = m40.mode = TxMode::Mode::HT;
  m20.ht_mcs = m40.ht_mcs = 1;
  m40.bw_mhz = 40;
  CHECK(cross_path_probe_frame(CHANNEL_WIDTH_20) == cross_path_probe_frame(CHANNEL_WIDTH_20));
  CHECK(cross_path_probe_frame(CHANNEL_WIDTH_20).size() == build_stream_radiotap(m20).size() + 24 + 150);
  CHECK(cross_path_probe_frame(CHANNEL_WIDTH_40).size() == build_stream_radiotap(m40).size() + 24 + 320);
  CHECK(cross_path_samples(CHANNEL_WIDTH_20) == 8192);
  CHECK(cross_path_samples(CHANNEL_WIDTH_40) == 16384);

  auto fake = std::make_shared<Fake>();
  fake->regs[0x820] = 0x12345632;
  fake->regs[0x410c] = 0xabcd0003;
  fake->regs[0x4c] = 0x00a00000;
  fake->regs[0x3c] = 0x04000011;
  const uint32_t s4c = 0x00a00000, s3c = 0x04000011;
  jaguar3::CrossPathListener8822e l(RtlAdapter(fake, std::make_shared<Logger>()));

  l.undo();
  CHECK(fake->writes.empty() && !l.active());
  CHECK(!l.park(true) && fake->writes.empty());

  l.setup();
  std::vector<uint32_t> want;
  for (uint32_t n = 0; n < 16; ++n) want.push_back(0x70000000u | (n << 16) | 0xF0001u);
  CHECK(fake->to(0x4130) == want);
  CHECK((fake->to(0x1c38) == std::vector<uint32_t>{0, 0xffffffff}));
  CHECK((fake->regs[0x410c] & 3) == 0 && fake->regs[0x410c] >> 2 == 0xabcd0003u >> 2);
  CHECK((fake->regs[0x820] & 0xff) == 0x31 && (fake->regs[0x820] >> 8) == 0x123456);
  CHECK(l.active());

  CHECK(l.park(true));
  CHECK((fake->regs[0x4c] & (1u << 22)) && !(fake->regs[0x3c] & (1u << 26)));
  l.set_gain(2);
  CHECK(fake->to(0x4108).back() == 0x300c0);
  fake->writes.clear();
  l.undo();
  auto w = fake->to(0x4130);
  CHECK(w.size() == 18 && w.back() == 0x70fb0001);
  CHECK(fake->regs[0x1c38] == 0xffa1005e && (fake->regs[0x410c] & 3) == 3);
  CHECK(fake->regs[0x410c] >> 2 == 0xabcd0003u >> 2);
  CHECK(fake->regs[0x820] == 0x12345632);
  CHECK(fake->regs[0x4c] == s4c && fake->regs[0x3c] == s3c);
  CHECK(!l.active());
  fake->writes.clear();
  l.undo();
  CHECK(fake->writes.empty());

  // a park that does not take
  fake->stuck[0x3c] = 1u << 26;
  l.setup();
  CHECK(!l.park(true));
  CHECK(fake->regs[0x4c] == s4c && fake->regs[0x3c] == s3c);
  l.undo();
  CHECK(fake->regs[0x4c] == s4c);

  if (fails) return 1;
  std::puts("cross_path: ok");
  return 0;
}
