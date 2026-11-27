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
  _txgapk_table = _afe_b_txgapk && _afe_b_txgapk();
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
  _parked = true; // set first: a throw mid-park still gets unparked by undo()
  _a.rtw_write32(kRegDpdt4c, _a.rtw_read32(kRegDpdt4c) | kBit22);
  _a.rtw_write32(kRegDpdt3c, _a.rtw_read32(kRegDpdt3c) & ~kBit26);
  if ((_a.rtw_read32(kRegDpdt4c) & kBit22) && !(_a.rtw_read32(kRegDpdt3c) & kBit26))
    return true;
  park(false);
  return false;
}

void CrossPathListener8822e::set_gain(int gain) {
  if (gain < 0 || gain >= devourer::kCrossPathGains)
    return;
  // RF reg 0x00 has address bits [27:20] = 0, so the word is the data itself.
  _a.rtw_write32(0x4108, devourer::kCrossPathGainWord[gain] & 0xfffff);
}

bool CrossPathListener8822e::undo() {
  if (!_active)
    return true;
  if (_parked)
    park(false);
  // Leave 0x4130 on the table bring-up left there, not merely a valid one.
  if (_txgapk_table)
    for (uint32_t v : kAfeTxgapkRestoreSeq)
      _a.rtw_write32(0x4130, v);
  else
    for (uint32_t v : kAfeRestoreSeqB)
      _a.rtw_write32(0x4130, v);
  _a.rtw_write32(0x1c38, 0xffa1005e);
  _a.rtw_write32(0x410c, (_a.rtw_read32(0x410c) & ~3u) | (_s410c & 3u));
  _a.rtw_write32(0x820, (_a.rtw_read32(0x820) & ~0xffu) | (_s820 & 0xffu));
  // After 3-wire is back on, the way the driver writes RF 0x00 (via 0x4108).
  _a.rtw_write32(0x4108, _sRfB0);
  const uint32_t r820 = _a.rtw_read32(0x820), r410c = _a.rtw_read32(0x410c),
                 r4c = _a.rtw_read32(kRegDpdt4c), r3c = _a.rtw_read32(kRegDpdt3c);
  _active = false;
  if ((r820 & 0xff) == (_s820 & 0xff) && (r410c & 3) == (_s410c & 3) &&
      (r4c & kBit22) == (_s4c & kBit22) && (r3c & kBit26) == (_s3c & kBit26))
    return true;
  _a.logger()->error("CrossPath: listener undo read back 0x820=0x{:08x} "
                     "0x410c=0x{:08x} 0x4c=0x{:08x} 0x3c=0x{:08x}, saved "
                     "0x{:08x} 0x{:08x} 0x{:08x} 0x{:08x}",
                     r820, r410c, r4c, r3c, _s820, _s410c, _s4c, _s3c);
  return false;
}

namespace {
void sleep_ms(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
}

bool CrossPathSession::wait_drained() {
  if (!_io.drained)
    return true;
  for (int t = 0; t < kDrainMs; t += 2) {
    if (_io.drained())
      return true;
    sleep_ms(2);
  }
  return _io.drained();
}

bool CrossPathSession::begin(ChannelWidth_t bw) {
  if (_active || (bw != CHANNEL_WIDTH_20 && bw != CHANNEL_WIDTH_40))
    return false;
  bool relaid = false, ok = false;
  try {
    if (!wait_drained())
      return false; // nothing written yet
    _saved_index = _last_index = _io.get_index();
    relaid = true; // from here a failure re-lays to full
    if (_io.relayout(true)) {
      {
        auto lk = regs();
        _l.setup();
      }
      // The train starts after the listener, so every probe is heard.
      _train_stop = false;
      _train = std::thread([this, f = devourer::cross_path_probe_frame(bw)] {
        while (!_train_stop) {
          bool sent = false;
          try {
            sent = _io.send(f.data(), f.size());
          } catch (...) { // a dead transfer counts as a refused send
          }
          if (!sent)
            sleep_ms(1); // a refusing send path must not spin a core
        }
      });
      ok = true;
    }
  } catch (...) {
    _l.logger()->error("CrossPath: begin failed on register I/O");
  }
  if (ok) {
    _bw = bw;
    _active = true;
    return true;
  }
  if (!relaid)
    return false;
  try {
    auto lk = regs();
    _l.undo(); // no-op if setup never ran
  } catch (...) {
    _l.logger()->error("CrossPath: begin failed and the listener undo threw");
  }
  try {
    if (!_io.relayout(false)) // a half-applied layout goes back to full
      _l.logger()->error("CrossPath: begin failed and the full re-layout failed");
  } catch (...) {
    _l.logger()->error("CrossPath: begin failed and the full re-layout threw");
  }
  return false;
}

bool CrossPathSession::park(bool parked) {
  if (!_active)
    return false;
  try {
    auto lk = regs();
    return _l.park(parked);
  } catch (...) {
    return false;
  }
}

bool CrossPathSession::capture(int tx_index, int gain,
                               std::vector<devourer::IqSample> &out) {
  out.clear();
  if (!_active || tx_index < 0 || tx_index > 127 || gain < 0 ||
      gain >= devourer::kCrossPathGains)
    return false;
  try {
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
  } catch (...) {
    out.clear();
    return false;
  }
}

bool CrossPathSession::end() {
  if (!_active)
    return true;
  _train_stop = true;
  if (_train.joinable())
    _train.join();
  const Logger_t &log = _l.logger();
  // Re-laying a buffer that still holds frames corrupts the page accounting
  // and wedges TX. Past the bound the layout goes back to full regardless:
  // leaving TX confined to half is not an option either.
  bool drained = false;
  try {
    drained = wait_drained();
  } catch (...) {
  }
  if (!drained)
    log->error("CrossPath: TX queue not drained within {} ms; re-laying the "
               "buffer to full anyway (TX may wedge)",
               kDrainMs);
  bool done = true, matched = true;
  try {
    auto lk = regs();
    matched = _l.undo();
  } catch (...) {
    done = false;
    log->error("CrossPath: end: listener undo threw");
  }
  try {
    if (!_io.relayout(false)) {
      done = false;
      log->error("CrossPath: end: full re-layout failed");
    }
  } catch (...) {
    done = false;
    log->error("CrossPath: end: full re-layout threw");
  }
  try {
    _io.set_index(_saved_index);
  } catch (...) {
    done = false;
    log->error("CrossPath: end: index override restore threw");
  }
  if (done)
    _active = false;
  return done && matched && drained;
}

} // namespace jaguar3
