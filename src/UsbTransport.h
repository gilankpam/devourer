#pragma once

/* UsbTransport — the libusb implementation of IRtlTransport. Everything
 * USB-wire-specific that used to live inside the adapter is here: vendor
 * control transfers for the register plane, sync/async bulk-OUT TX with the
 * wedge (mid-stream stall) recovery and TX submission counters, the
 * kernel-rtw88-style async RX URB queue, and the interface-descriptor walk
 * that discovers the bulk endpoints. The exclusive per-adapter UsbDeviceLock
 * rides here too — its lifetime is the transport's. */

#include "UsbXferCount.h"
#include <atomic>
#include <memory>
#include <mutex>
#include <vector>

#include <libusb.h>

#include "DeviceConfig.h"
#include "RtlTransport.h"
#include "logger.h"

namespace devourer {
class UsbDeviceLock;
}

#define REALTEK_USB_VENQT_READ 0xC0
#define REALTEK_USB_VENQT_WRITE 0x40
#define USB_TIMEOUT 500

namespace devourer {

class UsbTransport final : public IRtlTransport {
public:
  UsbTransport(libusb_device_handle *dev_handle, Logger_t logger,
               libusb_context *ctx = nullptr,
               std::shared_ptr<devourer::UsbDeviceLock> usb_lock = nullptr,
               bool rx_zerocopy = false, RxMode rx_mode = RxMode::Async,
               int pool_spare = 0, int ring_ms = 0,
               PoolExhaust pool_exhaust = PoolExhaust::Backpressure,
               bool ctrl_batch_enabled = true);
  ~UsbTransport() override;

  bool is_usb() const override { return true; }

  uint8_t read8(uint16_t reg) override { return ctrl_read<uint8_t>(reg); }
  uint16_t read16(uint16_t reg) override { return ctrl_read<uint16_t>(reg); }
  uint32_t read32(uint16_t reg) override { return ctrl_read<uint32_t>(reg); }
  bool write8(uint16_t reg, uint8_t v) override { return ctrl_write(reg, v); }
  bool write16(uint16_t reg, uint16_t v) override { return ctrl_write(reg, v); }
  bool write32(uint16_t reg, uint32_t v) override { return ctrl_write(reg, v); }
  bool write32_wide(uint32_t addr, uint32_t v) override {
    /* Realtek USB register addressing: wValue = addr[15:0], wIndex =
     * addr[31:16]. Lets the BB/RF window (addr + 0x10000) reach wIndex=1
     * instead of colliding with the MAC/system space at wIndex=0. */
    usb_ctrl_xfers().fetch_add(1, std::memory_order_relaxed);
    if (_batch)
      return async_write(static_cast<uint16_t>(addr & 0xFFFF),
                         static_cast<uint16_t>(addr >> 16), &v, sizeof(v));
    return libusb_control_transfer(
               _dev_handle, REALTEK_USB_VENQT_WRITE, 5,
               static_cast<uint16_t>(addr & 0xFFFF),
               static_cast<uint16_t>(addr >> 16), (uint8_t *)&v, sizeof(v),
               USB_TIMEOUT) == static_cast<int>(sizeof(v));
  }
  uint32_t read32_wide(uint32_t addr) override {
    uint32_t data = 0;
    usb_ctrl_xfers().fetch_add(1, std::memory_order_relaxed);
    if (_batch) {
      if (async_read(static_cast<uint16_t>(addr & 0xFFFF),
                     static_cast<uint16_t>(addr >> 16), &data, sizeof(data)))
        return data;
      return 0xFFFFFFFFu;
    }
    if (libusb_control_transfer(_dev_handle, REALTEK_USB_VENQT_READ, 5,
                                static_cast<uint16_t>(addr & 0xFFFF),
                                static_cast<uint16_t>(addr >> 16),
                                (uint8_t *)&data, sizeof(data),
                                USB_TIMEOUT) == static_cast<int>(sizeof(data)))
      return data;
    return 0xFFFFFFFFu; /* INVALID_RF_DATA-style sentinel on a failed read */
  }
  bool write_bytes(uint16_t reg, const uint8_t *p, size_t n) override;
  void write_batch_begin() override;
  void write_batch_end() override;
  void flush_writes() override;
  /* Batched EP0 register transfers. See IRtlTransport::ctrl_batch for the
   * contract; the implementation notes (chunking at kAsyncWriteDepth, the
   * cross-thread event-pump hazard, why the read payload is copied inside the
   * wait loop) are on the definition in UsbTransport.cpp. */
  bool ctrl_batch(std::vector<CtrlOp> &ops) override;

  bool tx_async(uint8_t ep, uint8_t *buf, size_t len,
                unsigned timeout_ms) override;
  int tx_sync(uint8_t ep, uint8_t *buf, size_t len, int timeout_ms) override;
  void rx_loop(int buf_size, int n_urbs,
               const std::function<void(const uint8_t *, int)> &on_data,
               const std::function<bool()> &should_stop) override;
  int rx_raw(uint8_t *buf, int len, int timeout_ms) override;
  void clear_halt(uint8_t ep) override { libusb_clear_halt(_dev_handle, ep); }
  void quiesce_tx() override;

  UsbLinkInfo usb_info() const override { return _info; }
  TxStats tx_stats() const override;

private:
  template <typename T> T ctrl_read(uint16_t reg);
  template <typename T> bool ctrl_write(uint16_t reg, T value);
  /* Pipelined-write machinery (see IRtlTransport::write_batch_begin). */
  /* Register transfers are 1/2/4 bytes; async_write/async_read refuse a
   * larger payload rather than overrun the inline setup buffer. */
  static constexpr size_t kAsyncMaxPayload = 4;
  /* CROSS-THREAD as of the ctrl_batch work (2026-09-14). Before it, the slot
   * pool was only ever driven by the single-threaded bring-up write batch, so
   * plain fields were safe. A runtime ctrl_batch pumps libusb events from the
   * CALLING thread while the RX bulk pump pumps the SAME context on its own
   * thread, and libusb lets either thread reap either thread's completions —
   * so async_write_cb can now run on a thread that is not the submitter.
   * `done` is the handshake (release on the callback side, acquire on the
   * waiter's) and publishes `status`/`actual`/the payload bytes written
   * before it. */
  struct AsyncWrite {
    libusb_transfer *t;
    uint8_t buf[LIBUSB_CONTROL_SETUP_SIZE + kAsyncMaxPayload];
    UsbTransport *self;
    std::atomic<bool> done;
    /* Submitted and not yet reaped: libusb owns `t` and `buf` while set, so
     * the slot must not be reused, freed, or handed back to the free list. */
    std::atomic<bool> inflight;
    int status;
    int actual;
  };
  /* 8 is a FLOOR, not an optimum: the InitWrite measurement this came from
   * reads "depth >= 8". It also caps a ctrl_batch chunk, and on the RK3566
   * ground station the cost model is ~290 us per completion WAIT against only
   * ~7-16 us per op — so the 12-op scout read pays that ~290 us twice for
   * nothing, and a depth >= 12 would fold it into one wait (measured basis
   * and the expected ~590 -> ~390 us in tests/scout_read_bench.cpp's header).
   * NOT raised here: it is on InitWrite's proven ~14k-write bring-up path and
   * needs bring-up-time validation on hardware first. */
  static constexpr int kAsyncWriteDepth = 8;
  /* Extra event-loop turns spent reaping cancellations after a drain times
   * out. A live loop reports each cancellation promptly; a dead one fails
   * every turn immediately, so this costs nothing in the case that matters. */
  static constexpr int kFlushCancelTurns = 8;
  bool async_write(uint16_t wvalue, uint16_t windex, const void *data,
                   size_t n);
  /* Read queued behind the pending writes (EP0 order) and waited for on its
   * own completion only: a read-modify-write pair costs one wakeup, not two.
   * Returns false on failure (data untouched). */
  bool async_read(uint16_t wvalue, uint16_t windex, void *data, size_t n);
  AsyncWrite *async_take_slot();
  /* Non-blocking take: a free slot, or nullptr if the pool is momentarily
   * empty. ctrl_batch sizes each chunk by what this yields, which is what
   * makes it impossible for a chunk to cannibalise its own in-flight slots —
   * see the definition of ctrl_batch. */
  AsyncWrite *async_try_take_slot();
  /* Allocate the async slot pool on first use. Extracted from
   * write_batch_begin so ctrl_batch can arm the pool at runtime on a session
   * that never ran a bring-up write batch (and so there is exactly ONE copy
   * of the allocation). Idempotent; never shrinks the pool. */
  void ensure_async_slots();
  /* Hand a slot back to the free list (mutex-guarded: the completion callback
   * does the same from whichever thread reaped it). */
  void async_release_slot(AsyncWrite *w);
  bool async_wait_progress(); /* one event-loop turn; false on timeout/error */
  static void LIBUSB_CALL async_write_cb(libusb_transfer *t);
  bool _batch = false;
  /* Guards _aw_free only. Held for the push/pop, never across a libusb call:
   * async_write_cb pushes from the reaping thread (see AsyncWrite above), and
   * an unguarded vector push racing a pop corrupts the pool outright. */
  std::mutex _aw_mu;
  std::vector<AsyncWrite *> _aw_free;
  /* Allocated once, never resized after: iterated without _aw_mu. */
  std::vector<AsyncWrite *> _aw_all;
  std::atomic<int> _aw_inflight{0};
  std::atomic<uint64_t> _aw_completed{0};
  std::atomic<int> _aw_errors{0};
  /* libusb "completed" flag for handle_events_timeout_completed: set by
   * async_write_cb after it bumps _aw_completed, so a waiter blocked in
   * libusb's event wait is woken by OUR completion even when another thread
   * owns the event lock. Without it a completion reaped by the RX thread
   * between the waiter's check and its pump costs a full 250 ms timeout —
   * exactly the latency this feature exists to remove. Plain int by libusb's
   * documented contract (it reads it under its own lock). */
  int _aw_wait_flag = 0;
  /* Held for the whole of ctrl_batch and across the _batch flip in
   * write_batch_begin/end, so a bring-up write batch and a runtime control
   * batch can never interleave over the same slot pool. */
  std::mutex _batch_mu;
  /* DeviceConfig::usb.ctrl_batch — false makes ctrl_batch fall back to the
   * synchronous default (the A/B knob, DEVOURER_CTRL_BATCH=0). */
  bool _cfg_ctrl_batch = true;
  /* Set when a drain gave up with transfers still submitted: the destructor
   * then leaks those slots instead of freeing a transfer libusb still owns.
   * ATOMIC because flush_writes() can set it from whichever thread reaped
   * (or failed to reap) the queue, while ctrl_batch re-reads it between
   * chunks — the pool is dead from that point and every further chunk would
   * otherwise submit into it and pay a ~2 s drain EACH, with the caller's
   * _reg_mu held the whole time. */
  std::atomic<bool> _aw_abandoned{false};
  void discover_endpoints(); /* was InitDvObj */
  const char *speed_str() const;
  static void transfer_callback(struct libusb_transfer *transfer);

  libusb_device_handle *_dev_handle;
  libusb_context *_ctx = nullptr;
  Logger_t _logger;
  UsbLinkInfo _info;

  /* Set by transfer_callback when an async TX bulk-OUT completes non-OK
   * (TIMED_OUT / stall). Consumed at the top of the next tx_async on the TX
   * thread to re-clear_halt the endpoint — a mid-stream stall (e.g. hardware
   * NDP generation on some xhci hosts) would otherwise stay wedged, since the
   * first-send clear_halt only runs once. */
  std::atomic<bool> _tx_wedged{false};

  /* TX submission counters (the driver-drop / congestion signal, TxStats.h).
   * The async transfer_callback increments them from the libusb event thread. */
  std::atomic<uint64_t> _tx_submitted{0};
  std::atomic<uint64_t> _tx_failed{0};
  std::atomic<int> _tx_last_rc{0};
  std::atomic<bool> _tx_last_timeout{false};

  /* Async-TX completions must be reaped by libusb_handle_events or the kernel
   * URB queue fills, submits start failing, and TX throughput collapses (the
   * Jaguar1 issue #240: its send path is tx_async and a TX-only session has no
   * other event pump). We reap in the CALLER's thread — each tx_async drains
   * completed transfers before submitting the next — rather than a background
   * pump thread, which would race the caller-owned libusb teardown (an earlier
   * attempt crashed on a usbi_mutex assertion). _tx_inflight tracks
   * submitted-but-not-yet-reaped transfers so the destructor can drain them
   * before the device handle / context go away, and so a soft cap can throttle
   * over-submission. */
  std::atomic<int> _tx_inflight{0};

  /* Submitted-but-not-yet-completed transfers, so quiesce_tx can cancel them
   * by handle. transfer_callback removes its own entry, and it runs on
   * whichever thread pumped the event — the submitting one, or another
   * tx_async caller under DEVOURER_TX_THREADS. Never hold _tx_mu across a
   * libusb_handle_events call: the callback re-enters and takes it. */
  std::mutex _tx_mu;
  std::vector<libusb_transfer *> _tx_live;

  /* Latched by quiesce_tx. Refuses further submissions (and further event
   * pumping) so nothing re-enters libusb once teardown has begun, and makes
   * quiesce idempotent for the Stop()-then-destructor path. */
  std::atomic<bool> _tx_shutdown{false};

  /* Allocate the async RX ring from kernel DMA memory (dev_mem_alloc) for a
   * zerocopy bulk-IN path; falls back to heap buffers per-URB when the alloc is
   * unsupported. See rx_loop and DeviceConfig::Usb::rx_zerocopy. */
  bool _rx_zerocopy = false;

  /* RX-ring servicing strategy + buffer-pool depth + diagnostic telemetry
   * cadence, from DeviceConfig::Rx. rx_loop reads these; the defaults preserve
   * the historic inline async ring with no extra buffers and no telemetry. */
  RxMode _rx_mode = RxMode::Async;
  int _pool_spare = 0;
  int _ring_ms = 0;
  PoolExhaust _pool_exhaust = PoolExhaust::Backpressure;

  /* rx_loop helpers for the servicing strategies dispatched off _rx_mode. */
  void rx_loop_sync(int buf_size,
                    const std::function<void(const uint8_t *, int)> &on_data,
                    const std::function<bool()> &should_stop);

  /* Exclusive per-adapter USB lock (UsbDeviceLock.h), held for the transport
   * lifetime; released when the device (and thus the transport) dies. */
  std::shared_ptr<devourer::UsbDeviceLock> _usb_lock;
};

template <typename T> T UsbTransport::ctrl_read(uint16_t reg_num) {
  T data = 0;
  usb_ctrl_xfers().fetch_add(1, std::memory_order_relaxed);
  if (_batch) {
    if (async_read(reg_num, 0, &data, sizeof(T)))
      return data;
    _logger->error("rtw_read({:04x}) pipelined, sizeof(T) = {}", reg_num, sizeof(T));
    throw std::ios_base::failure("rtw_read");
  }
  if (libusb_control_transfer(_dev_handle, REALTEK_USB_VENQT_READ, 5, reg_num,
                              0, (uint8_t *)&data, sizeof(T),
                              USB_TIMEOUT) == sizeof(T)) {
    return data;
  }
  _logger->error("rtw_read({:04x}), sizeof(T) = {}", reg_num, sizeof(T));
  throw std::ios_base::failure("rtw_read");
  return 0;
}

template <typename T> bool UsbTransport::ctrl_write(uint16_t reg_num, T value) {
  usb_ctrl_xfers().fetch_add(1, std::memory_order_relaxed);
  if (_batch)
    return async_write(reg_num, 0, &value, sizeof(T));
  return libusb_control_transfer(_dev_handle, REALTEK_USB_VENQT_WRITE, 5,
                                 reg_num, 0, (uint8_t *)&value, sizeof(T),
                                 USB_TIMEOUT) == sizeof(T);
}

} /* namespace devourer */
