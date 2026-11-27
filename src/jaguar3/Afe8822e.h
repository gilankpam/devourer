/* AFE sequences on the 8822E that more than one component replays. */
#ifndef JAGUAR3_AFE_8822E_H
#define JAGUAR3_AFE_8822E_H

#include <cstdint>

namespace jaguar3 {

/* Path-B AFEDIG (0x4130) restore, in write order: ends the IQK AFE setting and
 * returns chain B's ADC/DAC path to the normal-operation state. */
inline constexpr uint32_t kAfeRestoreSeqB[18] = {
    0x700f0001, 0x700f0001, 0x70144001, 0x70244001, 0x70344001, 0x70444001,
    0x705b0001, 0x70644001, 0x707b0001, 0x708f0001, 0x709f0001, 0x70af0001,
    0x70bf0001, 0x70cb0001, 0x70db0001, 0x70eb0001, 0x70fb0001, 0x70fb0001};

} // namespace jaguar3
#endif
