#include "CrossPath8822e.h"

#include <chrono>

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
    _a.rtw_write32(0x4130, 0x70000000u | (n << 20) | 0xF0001u); // 0x70<n>F0001
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

namespace {
void sleep_ms(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
}

bool CrossPathSession::begin(ChannelWidth_t bw) {
  if (_active || (bw != CHANNEL_WIDTH_20 && bw != CHANNEL_WIDTH_40))
    return false;
  _saved_index = _last_index = _io.get_index();
  bool ok = false;
  try {
    if (_io.relayout(true)) {
      {
        auto lk = regs();
        _l.setup();
      }
      // The train starts after the listener, so every probe is heard.
      _train_stop = false;
      _train = std::thread([this, f = devourer::cross_path_probe_frame(bw)] {
        while (!_train_stop)
          if (!_io.send(f.data(), f.size()))
            sleep_ms(1); // a refusing send path must not spin a core
      });
      ok = true;
    }
  } catch (...) {
  }
  if (!ok) {
    try {
      {
        auto lk = regs();
        _l.undo(); // no-op if setup never ran
      }
      _io.relayout(false); // a half-applied layout goes back to full
    } catch (...) {
    }
    return false;
  }
  _bw = bw;
  _active = true;
  return true;
}

bool CrossPathSession::park(bool parked) {
  if (!_active)
    return false;
  auto lk = regs();
  return _l.park(parked);
}

bool CrossPathSession::capture(int tx_index, int gain,
                               std::vector<devourer::IqSample> &out) {
  out.clear();
  if (!_active || tx_index < 0 || tx_index > 127 || gain < 0 ||
      gain >= devourer::kCrossPathGains)
    return false;
  if (tx_index != _last_index) {
    _io.set_index(tx_index);
    _last_index = tx_index;
    sleep_ms(15); // TXAGC settle
  }
  {
    auto lk = regs();
    _l.set_gain(gain);
  }
  sleep_ms(3); // RX gain settle
  devourer::LaParams p;
  p.trig_mode = devourer::LaTrigMode::AdcMacTrig;
  p.mac_sig = devourer::LaMacSig::Manual;
  p.dma_type = 1;
  p.dbg_port = 0xa80;
  p.trigger_time_us = 300;
  p.poll_ms = 2;
  p.poll_tries = 2000;
  p.block_len = 248;
  p.smp_rate = _bw == CHANNEL_WIDTH_40 ? 1 : 2;
  p.max_samples = devourer::cross_path_samples(_bw);
  const devourer::LaResult r = _io.capture(p);
  if (!r.ok)
    return false;
  out.reserve(r.samples.size());
  for (uint64_t w : r.samples)
    out.push_back(devourer::cross_path_sample(w));
  return true;
}

void CrossPathSession::end() {
  if (!_active)
    return;
  _train_stop = true;
  if (_train.joinable())
    _train.join();
  sleep_ms(30); // the chip's queue empties before the buffer moves
  {
    auto lk = regs();
    _l.undo();
  }
  _io.relayout(false);
  _io.set_index(_saved_index);
  _active = false;
}

} // namespace jaguar3
