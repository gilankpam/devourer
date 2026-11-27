#include "CrossPath8822e.h"

#include "Afe8822e.h"
#include "CrossPathCapture.h"

namespace jaguar3 {

namespace {
constexpr uint16_t kRegDpdt4c = 0x4c;  // [22] set while parked
constexpr uint16_t kRegDpdt3c = 0x3c;  // [26] cleared while parked
constexpr uint32_t kBit22 = 1u << 22, kBit26 = 1u << 26;
}

void CrossPathListener8822e::setup() {
  if (_active)
    return;
  _s820 = _a.rtw_read32(0x820);
  _s410c = _a.rtw_read32(0x410c);
  _s4c = _a.rtw_read32(kRegDpdt4c);
  _s3c = _a.rtw_read32(kRegDpdt3c);
  _sRfB0 = _a.rtw_read32(0x4c00) & 0xfffff; // path-B RF 0x00 via the direct window
  for (uint32_t n = 0; n < 16; ++n)
    _a.rtw_write32(0x4130, 0x70000000u | (n << 16) | 0xF0001u);
  _a.rtw_write32(0x1c38, 0);
  _a.rtw_write32(0x1c38, 0xffffffff);
  _a.rtw_write32(0x410c, _s410c & ~3u);
  _a.rtw_write32(0x820, (_s820 & ~0xffu) | 0x31);
  _active = true;
  _parked = false;
}

bool CrossPathListener8822e::park(bool parked) {
  if (!_active)
    return false;
  if (!parked) {
    _a.rtw_write32(kRegDpdt4c, (_a.rtw_read32(kRegDpdt4c) & ~kBit22) | (_s4c & kBit22));
    _a.rtw_write32(kRegDpdt3c, (_a.rtw_read32(kRegDpdt3c) & ~kBit26) | (_s3c & kBit26));
    _parked = false;
    return true;
  }
  _a.rtw_write32(kRegDpdt4c, _a.rtw_read32(kRegDpdt4c) | kBit22);
  _a.rtw_write32(kRegDpdt3c, _a.rtw_read32(kRegDpdt3c) & ~kBit26);
  if ((_a.rtw_read32(kRegDpdt4c) & kBit22) && !(_a.rtw_read32(kRegDpdt3c) & kBit26)) {
    _parked = true;
    return true;
  }
  park(false);
  return false;
}

void CrossPathListener8822e::set_gain(int gain) {
  if (gain < 0 || gain >= devourer::kCrossPathGains)
    return;
  // RF reg 0x00 has address bits [27:20] = 0, so the word is the data itself.
  _a.rtw_write32(0x4108, devourer::kCrossPathGainWord[gain] & 0xfffff);
}

void CrossPathListener8822e::undo() {
  if (!_active)
    return;
  if (_parked)
    park(false);
  for (uint32_t v : kAfeRestoreSeqB)
    _a.rtw_write32(0x4130, v);
  _a.rtw_write32(0x1c38, 0xffa1005e);
  _a.rtw_write32(0x410c, (_a.rtw_read32(0x410c) & ~3u) | 3u);
  _a.rtw_write32(0x820, (_a.rtw_read32(0x820) & ~0xffu) | (_s820 & 0xffu));
  // After 3-wire is back on, the way the driver writes RF 0x00 (via 0x4108).
  _a.rtw_write32(0x4108, _sRfB0);
  _active = false;
}

} // namespace jaguar3
