/* Headless guard for the active idle-noise-floor math helpers
 * (src/NoiseFloorMath.h): the 10-bit sign conversion of the 0x0FA0 debug-port
 * I/Q and the 3*log2 pwdb approximation. Pure math; no hardware. */
#include "NhmReader.h"
#include "NoiseFloorMath.h"

#include <cstdio>
#include <map>

using devourer::nf::pwdb_conversion;
using devourer::nf::sign_conversion;

static int g_fail = 0;

static void check(const char *what, long got, long want) {
  if (got != want) {
    std::printf("FAIL %s: got %ld want %ld\n", what, got, want);
    ++g_fail;
  }
}

int main() {
  /* 10-bit two's-complement (rxi/rxq packing). */
  check("sign 0x000", sign_conversion(0x000, 10), 0);
  check("sign 0x1ff", sign_conversion(0x1ff, 10), 511);   /* largest positive */
  check("sign 0x200", sign_conversion(0x200, 10), -512);  /* sign bit only */
  check("sign 0x3ff", sign_conversion(0x3ff, 10), -1);    /* all ones = -1 */

  /* pwdb = 3*log2(X) - 3*decimal_bit + coarse-decimal, called as (X,20,18). */
  check("pwdb 0",       pwdb_conversion(0, 20, 18), -54);       /* X==0 -> 1 */
  check("pwdb 1",       pwdb_conversion(1, 20, 18), -54);       /* only bit0 */
  check("pwdb 4",       pwdb_conversion(4, 20, 18), -48);       /* bit2, no dec */
  check("pwdb 6",       pwdb_conversion(6, 20, 18), -46);       /* bit2 + bit1 dec */
  check("pwdb 1<<18",   pwdb_conversion(1 << 18, 20, 18), 0);   /* full scale */
  check("pwdb 1<<19",   pwdb_conversion(1 << 19, 20, 18), 3);
  /* A representative accepted idle sample: I=Q=20 -> val=800, leading bit 9. */
  check("pwdb 800",     pwdb_conversion(800, 20, 18), -25);     /* in [-27,0) */

  /* --- read_nhm threshold writes against a fake BB (JGR3 map). The masked
   * write helper shifts the value to the mask itself, so th[8..10] must go in
   * unshifted: pre-shifting pushed them past the mask and the registers read 0
   * (buckets 9-11 meaningless). IGI 0x1e -> th[0] = (30-14)*2 = 32, +4 each. */
  {
    std::map<uint16_t, uint32_t> regs;
    const auto r = devourer::nhm_regs_jgr3();
    regs[r.ready] = (1u << 16) | 123;
    auto read32 = [&](uint16_t a) { return regs[a]; };
    auto set_bb = [&](uint16_t a, uint32_t m, uint32_t v) {
      int sh = 0; while (sh < 32 && !((m >> sh) & 1u)) sh++;
      regs[a] = (regs[a] & ~m) | ((v << sh) & m);
    };
    RxEnergy e;
    devourer::read_nhm(r, 0x1e, read32, set_bb, e);
    check("bg valid", e.valid_nhm ? 1 : 0, 1);
    check("bg th0", regs[r.th0_3] & 0xff, 32);
    check("bg th8", (regs[r.th8] >> r.th8_shift) & 0xff, 32 + 4 * 8);
    check("bg th9", (regs[r.ctrl] >> 16) & 0xff, 32 + 4 * 9);
    check("bg th10", (regs[r.ctrl] >> 24) & 0xff, 32 + 4 * 10);
    check("bg cfg includes cca", (regs[r.ctrl] >> 8) & 0xf, 0x3);
    check("bg period", (regs[r.period] >> 16) & 0xffff, 500);
  }

  if (g_fail) {
    std::printf("noise_floor_math: %d failure(s)\n", g_fail);
    return 1;
  }
  std::printf("noise_floor_math: all checks passed\n");
  return 0;
}
