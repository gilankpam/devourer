/* Chain-B listener for the 8822E cross-path capture: chain A transmits, chain
 * B receives into the logic-analyzer capture. setup() applies the listener
 * recipe, undo() puts back exactly what it changed. */
#ifndef JAGUAR3_CROSS_PATH_8822E_H
#define JAGUAR3_CROSS_PATH_8822E_H

#include <cstdint>

#include "RtlAdapter.h"

namespace jaguar3 {

class CrossPathListener8822e {
public:
  explicit CrossPathListener8822e(RtlAdapter adapter) : _a(adapter) {}

  /* Save 0x820/0x410c/0x4c/0x3c and apply the listener recipe. No-op while
   * already active. */
  void setup();
  /* Park (true): 0x4c[22]=1, 0x3c[26]=0, read back. Returns false, with both
   * bits restored, if either did not take or no session is active. Park
   * (false) puts both bits back to their saved values. */
  bool park(bool parked);
  /* Listener RX gain: kCrossPathGainWord[gain] to RF 0x00 of path B. */
  void set_gain(int gain);
  /* Unpark if parked, then restore the listener's changes. No-op when not
   * active. */
  void undo();
  bool active() const { return _active; }

private:
  RtlAdapter _a;
  bool _active = false, _parked = false;
  uint32_t _s820 = 0, _s410c = 0, _s4c = 0, _s3c = 0, _sRfB0 = 0;
};

} // namespace jaguar3
#endif
