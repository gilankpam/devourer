// scout_read_bench.cpp — transfer-count and latency bench for
// IRtlDevice::GetRxEnergyScout() vs the full GetRxEnergy(false), the
// evidence for the "~6 reads instead of ~24" channel-scout claim (mabur
// 2026-09-14 spec §6). Same open sequence as tests/reglat.cpp (USB vid/pid,
// DEVOURER_VID/DEVOURER_PID env), but GetRxEnergyScout is an IRtlDevice
// method, not a bare register op, so bring-up runs InitWrite first via
// WiFiDriver::CreateRtlDevice — the doctor demo's open path, trimmed to a
// single chip with no CLI.
//
//   sudo ./build/scout_read_bench                 (DEVOURER_PID/VID pick the
//                                                   adapter; DEVOURER_CHANNEL
//                                                   picks the bring-up
//                                                   channel, default 6)
//
// Expected (UNVERIFIED on this host — no Realtek card attached; run on a
// rig with an 8822C/E and record the numbers): GetRxEnergyScout ~14
// transfers/call (16 on the first, un-primed call) vs GetRxEnergy(false)
// ~24, and a proportional latency drop.
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
