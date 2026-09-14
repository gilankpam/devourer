// scout_read_bench.cpp — transfer-count and latency bench for
// IRtlDevice::GetRxEnergyScout() vs the full GetRxEnergy(false), the
// evidence for the "~6 reads instead of ~24" channel-scout claim (mabur
// 2026-09-14 spec §6). Same open sequence as tests/reglat.cpp (USB vid/pid,
// DEVOURER_VID/DEVOURER_PID env), but GetRxEnergyScout is an IRtlDevice
// method, not a bare register op, so bring-up runs InitWrite first via
// WiFiDriver::CreateRtlDevice — the doctor demo's open path, trimmed to a
// single chip with no CLI.
//
//   DEVOURER_PID=0xa81a ./build/scout_read_bench  (DEVOURER_VID/PID pick the
//                                                   adapter; DEVOURER_CHANNEL
//                                                   picks the bring-up
//                                                   channel, default 6; no
//                                                   sudo needed when the USB
//                                                   device node is world-rw
//                                                   and no kernel driver is
//                                                   bound)
//
// Measured 2026-09-14 (spare 8822EU, USB bus 5-1, 200 calls each, after
// InitWrite ch6): GetRxEnergyScout 10.0 xfers/call (6 reads + 4 composed
// full-dword writes — no first-call prime spike visible at this N), us
// min/med/mean/max 3553/3750/3763/4444; GetRxEnergy(false) 24.0 xfers/call
// (8 reads + 8 MASKED writes, each a read-modify-write = 2 transfers), us
// 8793/9000/9020/9352. Implied per-transfer cost is self-consistent between
// the two rows (~370-375 us/xfer on this rig's USB path), which is the
// sanity check that usb_ctrl_xfers() is really moving and not reporting a
// constant. The transfer-count DIFFERS from the plan's ~14/~24 estimate —
// the plan under-credited how much the composed-shadow write buys: every
// phy_set_bb_reg in GetRxEnergy's reset is a masked read-modify-write (2
// transfers), while the scout's rtw_write32 of a pre-composed dword is a
// single transfer, so the scout undercuts the full read by more than
// predicted (10 vs 24, not 14 vs 24) — report the measurement, not the
// estimate.
#ifdef _WIN32
#define NOMINMAX
#endif

#if defined(__ANDROID__) || defined(_MSC_VER) || defined(__APPLE__)
#include <libusb.h>
#else
#include <libusb-1.0/libusb.h>
#endif

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <vector>

#include "DeviceSession.h"
#include "RxSense.h"
#include "UsbOpen.h"
#include "UsbXferCount.h"
#include "WiFiDriver.h"
#include "logger.h"

namespace {

struct Stats { double min, med, mean, max; };

Stats summarize(std::vector<double> &us) {
  std::sort(us.begin(), us.end());
  double sum = 0;
  for (double x : us) sum += x;
  return Stats{us.front(), us[us.size() / 2], sum / us.size(), us.back()};
}

/* Runs `n` calls of `fn`, returns per-call transfer count and latency
 * stats. `fn` is one of the two RxEnergy reads under test. */
template <typename Fn>
void bench(const char *label, IRtlDevice *dev, Fn fn, int n) {
  const uint64_t x0 = devourer::usb_ctrl_xfers().load();
  std::vector<double> us;
  us.reserve(n);
  for (int i = 0; i < n; ++i) {
    const auto a = std::chrono::steady_clock::now();
    (void)fn(dev);
    us.push_back(
        std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - a)
            .count());
  }
  const uint64_t x1 = devourer::usb_ctrl_xfers().load();
  const Stats s = summarize(us);
  std::printf("%s: %.1f xfers/call, us min/med/mean/max %.0f/%.0f/%.0f/%.0f\n",
             label, double(x1 - x0) / n, s.min, s.med, s.mean, s.max);
}

} // namespace

int main() {
  auto logger = std::make_shared<Logger>();

  libusb_context *ctx = nullptr;
  if (libusb_init(&ctx) < 0) {
    std::fprintf(stderr, "libusb_init failed\n");
    return 1;
  }
  devourer::DeviceSession session{logger};
  session.adopt_context(ctx);

  uint16_t vid = 0x0bda, pid = 0;
  if (const char *v = std::getenv("DEVOURER_VID")) vid = (uint16_t)strtoul(v, 0, 0);
  if (const char *p = std::getenv("DEVOURER_PID")) pid = (uint16_t)strtoul(p, 0, 0);
  libusb_device_handle *handle = libusb_open_device_with_vid_pid(ctx, vid, pid);
  if (!handle) {
    std::fprintf(stderr, "usb open %04x:%04x failed (set DEVOURER_PID)\n", vid, pid);
    return 1;
  }

  std::shared_ptr<devourer::UsbDeviceLock> lock;
  if (devourer::claim_interface_then_reset(
          handle, devourer::find_wifi_interface(handle), logger, true, lock) != 0) {
    session.adopt_handle(handle);
    return 1;
  }
  session.adopt_handle(handle);
  session.adopt_lock(lock);

  devourer::DeviceConfig cfg;
  WiFiDriver driver(logger);
  std::unique_ptr<IRtlDevice> owned = driver.CreateRtlDevice(handle, ctx, lock, cfg);
  if (!owned) {
    std::fprintf(stderr, "CreateRtlDevice failed (not a Jaguar3 card?)\n");
    return 1;
  }
  session.adopt_device(std::move(owned));
  IRtlDevice *const dev = session.device();

  uint8_t channel = 6;
  if (const char *c = std::getenv("DEVOURER_CHANNEL"))
    channel = static_cast<uint8_t>(strtoul(c, nullptr, 0));
  try {
    dev->InitWrite(SelectedChannel{.Channel = channel,
                                   .ChannelOffset = 0,
                                   .ChannelWidth = CHANNEL_WIDTH_20});
  } catch (const std::exception &e) {
    std::fprintf(stderr, "InitWrite failed: %s\n", e.what());
    return 1;
  }

  constexpr int kCalls = 200;
  bench("GetRxEnergyScout", dev,
       [](IRtlDevice *d) { return d->GetRxEnergyScout(); }, kCalls);
  bench("GetRxEnergy(false)", dev,
       [](IRtlDevice *d) { return d->GetRxEnergy(/*with_nhm=*/false); }, kCalls);

  return 0;
}
