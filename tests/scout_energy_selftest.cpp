#include <cstdio>
#include <cstdlib>
#include "jaguar3/ScoutEnergyMath.h"
static int fails = 0;
#define CHECK(c) do { if (!(c)) { std::fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #c); ++fails; } } while (0)
int main() {
  using namespace devourer::jgr3;
  // Vendor sum: parity(2d04 hi) + rate-illegal(2d08 lo) + crc8(2d08 hi) +
  // mcs(2d10 lo) + fast-fsync(2d20 lo) + sb-search(2d20 hi) + mcs-vht(2d10 hi)
  // + crc8-vhta(2d0c lo). Same formula as GetRxEnergy.
  CHECK(fa_ofdm_sum(0x00010000, 0x00030002, 0x00070004, 0x00060005, 0x00000008) ==
        1 + 2 + 3 + 4 + 5 + 6 + 7 + 8);
  CHECK(fa_ofdm_sum(0, 0, 0, 0, 0) == 0);
  CHECK(fa_ofdm_sum(0xffff0000, 0xffffffff, 0xffffffff, 0xffffffff, 0x0000ffff) == 8u * 0xffff);
  // Reset dwords: 0x1d2c[31] off, 0x1eb4[25] on, 0x1eb4[25] off, 0x1d2c[31] on,
  // every other bit of the shadow preserved.
  const ResetDwords r = compose_reset(0x80001234u, 0x00005678u);
  CHECK(r.d1d2c_off == 0x00001234u);
  CHECK(r.d1eb4_on == (0x00005678u | (1u << 25)));
  CHECK(r.d1eb4_off == 0x00005678u);
  CHECK(r.d1d2c_on == 0x80001234u);
  if (fails) return EXIT_FAILURE;
  std::puts("scout_energy_selftest: ok");
  return 0;
}
