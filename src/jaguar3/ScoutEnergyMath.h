/* Pure register arithmetic for RtlJaguar3Device::GetRxEnergyScout: the OFDM
 * false-alarm vendor sum (phydm_fa_cnt_statistics_jgr3) and the full-dword
 * composition of the BB counter reset (phydm_reset_bb_hw_cnt jgr3:
 * 0x1eb4[25] 1->0 wrapped by 0x1d2c[31] rx-clk-gate off/on). Composed from
 * cached shadows so the scout path never pays a read-modify-write. */
#ifndef DEVOURER_JAGUAR3_SCOUT_ENERGY_MATH_H
#define DEVOURER_JAGUAR3_SCOUT_ENERGY_MATH_H
#include <cstdint>
namespace devourer { namespace jgr3 {
inline uint32_t fa_ofdm_sum(uint32_t r2d04, uint32_t r2d08, uint32_t r2d10,
                            uint32_t r2d20, uint32_t r2d0c) {
  return ((r2d04 >> 16) & 0xffff) + (r2d08 & 0xffff) + ((r2d08 >> 16) & 0xffff) +
         (r2d10 & 0xffff) + (r2d20 & 0xffff) + ((r2d20 >> 16) & 0xffff) +
         ((r2d10 >> 16) & 0xffff) + (r2d0c & 0xffff);
}
struct ResetDwords { uint32_t d1d2c_off, d1eb4_on, d1eb4_off, d1d2c_on; };
inline ResetDwords compose_reset(uint32_t shadow_1d2c, uint32_t shadow_1eb4) {
  ResetDwords r;
  r.d1d2c_off = shadow_1d2c & ~(1u << 31);
  r.d1eb4_on = shadow_1eb4 | (1u << 25);
  r.d1eb4_off = shadow_1eb4 & ~(1u << 25);
  r.d1d2c_on = shadow_1d2c | (1u << 31);
  return r;
}
} }  // namespace devourer::jgr3
#endif
