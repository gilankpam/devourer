/* Chain-B listener for the 8822E cross-path capture: chain A transmits, chain
 * B receives into the logic-analyzer capture. setup() applies the listener
 * recipe, undo() puts back exactly what it changed. */
#ifndef JAGUAR3_CROSS_PATH_8822E_H
#define JAGUAR3_CROSS_PATH_8822E_H

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include "CrossPathCapture.h"
#include "LaCapture.h"
#include "RtlAdapter.h"

namespace jaguar3 {

class CrossPathListener8822e {
public:
  /* afe_b_txgapk: true when the last path-B AFE table bring-up wrote to the
   * write-only 0x4130 was the TXGAPK restore, false for the IQK restore
   * (Jaguar3Calibration::afe_b_txgapk_last). Unset reads as the IQK one. */
  explicit CrossPathListener8822e(RtlAdapter adapter,
                                  std::function<bool()> afe_b_txgapk = {})
      : _a(adapter), _afe_b_txgapk(std::move(afe_b_txgapk)) {}

  /* Save 0x820/0x410c/0x4c/0x3c, note which AFE table 0x4130 holds, and
   * apply the listener recipe. No-op while already active. */
  void setup();
  /* Park (true): 0x4c[22]=1, 0x3c[26]=0, read back. Returns false, with both
   * bits restored, if either did not take or no session is active. Park
   * (false) puts both bits back to their saved values. */
  bool park(bool parked);
  /* Listener RX gain: kCrossPathGainWord[gain] to RF 0x00 of path B. */
  void set_gain(int gain);
  /* Unpark if parked, then restore the listener's changes: 0x4130 gets the
   * AFE table noted at setup. Reads back the bits it restored in 0x820,
   * 0x410c, 0x4c and 0x3c; false (logged) when one differs from the saved
   * value. No-op returning true when not active. A throw leaves it active,
   * so a later undo() retries. */
  bool undo();
  bool active() const { return _active; }
  const Logger_t &logger() const { return _a.logger(); }

private:
  RtlAdapter _a;
  std::function<bool()> _afe_b_txgapk;
  bool _active = false, _parked = false, _txgapk_table = false;
  uint32_t _s820 = 0, _s410c = 0, _s4c = 0, _s3c = 0, _sRfB0 = 0;
};

/* True when every TX queue is empty: each REG_FIFOPAGE_INFO word (HQ, LQ,
 * NQ, PUB: 0x0230/0x0234/0x0238/0x0240) reads free pages [27:16] equal to its
 * configured pages [11:0]. PUB alone is not enough: the probe train rides
 * LQ, and on the bench the first poll after the train stopped read PUB
 * 721/721 with LQ at 20, 54 and 62 of 64 free. An idle queue always reads
 * free == configured, so this never refuses an idle chip. */
inline bool cross_path_queues_drained(const uint32_t (&info)[4]) {
  for (uint32_t v : info)
    if (((v >> 16) & 0xfff) != (v & 0xfff))
      return false;
  return true;
}

/* One cross-path session: TX buffer re-laid to half, listener set up and a
 * probe train running from begin() to end(), captures in between. The device
 * supplies the register work through Io; the session owns the order. Not
 * thread-safe: one control thread drives it. No member throws: a throwing Io
 * callable or register access counts as a failed step. */
class CrossPathSession {
public:
  struct Io {
    std::function<bool(bool half)> relayout;
    std::function<int()> get_index; // the flat index override now (-1 = none)
    std::function<void(int)> set_index;
    std::function<devourer::LaResult(const devourer::LaParams &)> capture;
    std::function<bool(const uint8_t *, size_t)> send;
    // True once the chip's TX queues are empty (cross_path_queues_drained).
    // Unset counts as drained.
    std::function<bool()> drained;
    // Held around the listener's register work when set; the other callables
    // serialize themselves.
    std::mutex *reg_mu = nullptr;
  };

  CrossPathSession(Io io, CrossPathListener8822e &listener)
      : _io(std::move(io)), _l(listener) {}
  ~CrossPathSession() { end(); }
  CrossPathSession(const CrossPathSession &) = delete;
  CrossPathSession &operator=(const CrossPathSession &) = delete;

  /* bw is 20 or 40 MHz: the probe's and the capture's width; the caller has
   * the chip tuned at it. False if a session is open, bw is another width or a
   * step fails (a throw included); a failure undoes the steps already done.
   * Refuses, writing nothing, when the TX queue does not drain within
   * kDrainMs: the buffer only moves when empty. */
  bool begin(ChannelWidth_t bw);
  bool park(bool parked);
  /* tx_index 0..127, gain 0..kCrossPathGains-1. `out` is cleared first and
   * filled only on a completed capture. */
  bool capture(int tx_index, int gain, std::vector<devourer::IqSample> &out);
  /* Stop the train, wait (up to kDrainMs) for the queue to empty, undo the
   * listener, re-lay the buffer to full, restore the saved index override.
   * Every step runs even when an earlier one failed. True when every step
   * succeeded, the listener read back as saved and the queue drained before
   * the re-layout; a drain timeout still undoes and re-lays (logged, false).
   * The session stays open, so a later end() retries, only when a step threw
   * or the re-layout failed. True with no session open. */
  bool end();
  bool active() const { return _active; }
  ChannelWidth_t width() const { return _bw; }
  static constexpr int kDrainMs = 500;

private:
  bool wait_drained();
  std::unique_lock<std::mutex> regs() {
    return _io.reg_mu ? std::unique_lock<std::mutex>(*_io.reg_mu)
                      : std::unique_lock<std::mutex>();
  }

  Io _io;
  CrossPathListener8822e &_l;
  bool _active = false;
  ChannelWidth_t _bw = CHANNEL_WIDTH_20;
  int _saved_index = -1, _last_index = -1;
  std::atomic<bool> _train_stop{false};
  std::thread _train;
};

} // namespace jaguar3
#endif
