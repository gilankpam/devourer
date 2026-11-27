/* AFE sequences on the 8822E that more than one component replays. */
#ifndef JAGUAR3_AFE_8822E_H
#define JAGUAR3_AFE_8822E_H

#include <cstdint>

namespace jaguar3 {

/* Path-B AFEDIG (0x4130) restore after IQK, in write order: ends the IQK AFE
 * setting. Bring-up runs TXGAPK after IQK, so this is not the table a
 * finished bring-up leaves in 0x4130 (Halrf8822e records which one is). */
inline constexpr uint32_t kAfeRestoreSeqB[18] = {
    0x700f0001, 0x700f0001, 0x70144001, 0x70244001, 0x70344001, 0x70444001,
    0x705b0001, 0x70644001, 0x707b0001, 0x708f0001, 0x709f0001, 0x70af0001,
    0x70bf0001, 0x70cb0001, 0x70db0001, 0x70eb0001, 0x70fb0001, 0x70fb0001};

/* AFEDIG restore after TXGAPK, in write order; the same values go to path A
 * (0x1830) and path B (0x4130). The last table a bring-up writes, unless
 * TXGAPK did not run. */
inline constexpr uint32_t kAfeTxgapkRestoreSeq[16] = {
    0x700b8041, 0x70144041, 0x70244041, 0x70344041, 0x70444041, 0x705b8041,
    0x70644041, 0x707b8041, 0x708b8041, 0x709b8041, 0x70ab8041, 0x70bb8041,
    0x70cb8041, 0x70db8041, 0x70eb8041, 0x70fb8041};

} // namespace jaguar3
#endif
