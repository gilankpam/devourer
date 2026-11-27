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

/* One cross-path session: TX buffer re-laid to half, listener set up and a
 * probe train running from begin() to end(), captures in between. The device
 * supplies the register work through Io; the session owns the order. Not
 * thread-safe: one control thread drives it. */
class CrossPathSession {
public:
  struct Io {
    std::function<bool(bool half)> relayout;
    std::function<int()> get_index; // the flat index override now (-1 = none)
    std::function<void(int)> set_index;
    std::function<devourer::LaResult(const devourer::LaParams &)> capture;
    std::function<bool(const uint8_t *, size_t)> send;
    // Held around the listener's register work when set; the other callables
    // serialize themselves.
    std::mutex *reg_mu = nullptr;
  };

  CrossPathSession(Io io, CrossPathListener8822e &listener)
      : _io(std::move(io)), _l(listener) {}
  ~CrossPathSession() {
    try {
      end();
    } catch (...) { // register I/O throws on a dead transfer
    }
  }
  CrossPathSession(const CrossPathSession &) = delete;
  CrossPathSession &operator=(const CrossPathSession &) = delete;

  /* bw is 20 or 40 MHz: the probe's and the capture's width; the caller has
   * the chip tuned at it. False if a session is open, bw is another width or a
   * step fails (a throw included); a failure
   * undoes the steps already done. */
  bool begin(ChannelWidth_t bw);
  bool park(bool parked);
  /* tx_index 0..127, gain 0..kCrossPathGains-1. `out` is cleared first and
   * filled only on a completed capture. */
  bool capture(int tx_index, int gain, std::vector<devourer::IqSample> &out);
  /* Stop the train, let the queue empty, undo the listener, re-lay the buffer
   * to full, restore the saved index override. No-op when not open. */
  void end();
  bool active() const { return _active; }
  ChannelWidth_t width() const { return _bw; }

private:
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
