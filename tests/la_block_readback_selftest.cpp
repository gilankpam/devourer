/* Headless guard for LaCapture's block readback (LaParams::block_len > 0)
 * against a fake transport that models the TX buffer behind the 0x0140 page
 * select + 0x8000 window. The window holds words whose value is their own
 * buffer address, so a wrong page, a block crossing a 4 KB page or a wrong
 * order at the ring wrap shows up as a sample mismatch. The capture is
 * scripted to finish mid-window with the ring wrapped, so the newest-2048
 * readback starts at a non-page-aligned address, crosses several pages and
 * wraps from the window end back to its start. */
#include <cstdio>
#include <cstring>
#include <map>
#include <memory>
#include <vector>

#include "LaCapture.h"
#include "Transport.h"

namespace {

constexpr uint32_t kFinish = 0x4000 + 0x100; /* unit 8 B: 0x20800 */

struct Fake : devourer::ITransport {
  std::map<uint16_t, uint32_t> regs;
  std::vector<uint8_t> txbuf = std::vector<uint8_t>(0x40000);
  uint32_t max_block_end = 0; /* max of (window offset & 0xfff) + len */
  bool short_reads = false;

  Fake() {
    for (uint32_t a = 0; a < txbuf.size(); a += 8) {
      const uint64_t w = a;
      std::memcpy(&txbuf[a], &w, 8);
    }
    regs[0x00F0] = 0x1234; /* liveness canary */
  }
  uint32_t base() { return (regs[0x0140] - 0x780) << 12; }

  uint32_t read32(uint16_t a) override {
    if (a == 0x07C0) /* finished, ring wrapped, poll bit clear */
      return 0x80000001u | (kFinish << 16);
    if (a >= 0x8000) {
      uint32_t v;
      std::memcpy(&v, &txbuf[base() + (a - 0x8000)], 4);
      return v;
    }
    return regs[a];
  }
  uint8_t read8(uint16_t a) override { return read32(a) & 0xff; }
  uint16_t read16(uint16_t a) override { return read32(a) & 0xffff; }
  int read_block(uint16_t a, uint8_t *buf, int len) override {
    max_block_end = std::max(max_block_end, uint32_t((a & 0xfff) + len));
    std::memcpy(buf, &txbuf[base() + (a - 0x8000)], len);
    return short_reads ? len - 8 : len;
  }
  bool write8(uint16_t a, uint8_t v) override { regs[a] = v; return true; }
  bool write16(uint16_t a, uint16_t v) override { regs[a] = v; return true; }
  bool write32(uint16_t a, uint32_t v) override { regs[a] = v; return true; }

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
  auto fake = std::make_shared<Fake>();
  auto logger = std::make_shared<Logger>();
  devourer::LaCapture la(RtlAdapter(fake, logger), logger,
                         devourer::la_regs_jgr3());

  devourer::LaParams p;
  p.max_samples = 2048;
  auto a = la.run(p); /* per-word path */
  p.block_len = 248;
  auto b = la.run(p);
  CHECK(a.ok && b.ok);
  CHECK(a.samples == b.samples);
  CHECK(b.samples.size() == 2048);
  /* oldest of the newest 2048 sits at 0x3c810; the ring wraps after 0x40000 */
  CHECK(b.samples.size() == 2048 && b.samples.front() == 0x3c810 &&
        b.samples.back() == 0x20808);
  CHECK(fake->max_block_end > 0 && fake->max_block_end <= 0x1000);

  fake->short_reads = true;
  auto c = la.run(p);
  CHECK(!c.ok && c.samples.empty());

  return fails ? 1 : 0;
}
