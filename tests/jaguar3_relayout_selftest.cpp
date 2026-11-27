/* Headless guard for HalmacJaguar3MacInit::relayout_tx_buffer against a
 * recording fake transport. */
#include <cstdio>
#include <memory>
#include <utility>
#include <vector>

#include "Transport.h"
#include "jaguar3/HalmacJaguar3MacInit.h"

namespace {

using W = std::pair<uint16_t, uint32_t>;

struct Fake : devourer::ITransport {
  std::vector<W> writes;
  uint32_t read32(uint16_t a) override { return a == 0x0100 ? 0xff : 0; }
  uint8_t read8(uint16_t a) override { return read32(a) & 0xff; }
  uint16_t read16(uint16_t a) override { return read32(a) & 0xffff; }
  bool write8(uint16_t a, uint8_t v) override { writes.push_back({a, v}); return true; }
  bool write16(uint16_t a, uint16_t v) override { writes.push_back({a, v}); return true; }
  bool write32(uint16_t a, uint32_t v) override { writes.push_back({a, v}); return true; }

  int count_to(uint16_t a) const {
    int n = 0;
    for (auto &w : writes) n += w.first == a;
    return n;
  }
  int count_in(uint16_t lo, uint16_t hi) const {
    int n = 0;
    for (auto &w : writes) n += w.first >= lo && w.first <= hi;
    return n;
  }
  int first_to(uint16_t a) const {
    for (size_t i = 0; i < writes.size(); i++)
      if (writes[i].first == a) return static_cast<int>(i);
    return -1;
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
  auto fake = std::make_shared<Fake>();
  auto logger = std::make_shared<Logger>();
  jaguar3::HalmacJaguar3MacInit mac(RtlAdapter(fake, logger), logger);

  CHECK(mac.relayout_tx_buffer(true));
  /* fires if the pulse is dropped or reordered after the page config */
  CHECK(fake->writes.size() > 2 && fake->writes[0] == W(0x0100, 0xfb));
  CHECK(fake->writes.size() > 2 && fake->writes[1] == W(0x0100, 0xff));
  CHECK(fake->first_to(0x0230) > 1);
  /* fires if the bit is held off across the config (extra write) or the pulse is skipped */
  CHECK(fake->count_to(0x0100) == 2);
  /* fires if the half layout is not applied (boundary stays at full) */
  const uint16_t half_bndy = mac.rsvd_boundary();
  CHECK(half_bndy != 0);
  /* fires if the full layout is not restored */
  CHECK(mac.relayout_tx_buffer(false));
  CHECK(mac.rsvd_boundary() == half_bndy + 1024);
  /* fires if the whole TRX init (EDCA/SIFS) is re-run */
  CHECK(fake->count_in(0x0500, 0x0574) == 0);
  return fails ? 1 : 0;
}
