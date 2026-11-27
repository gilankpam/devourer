/* crosspath — bench tool for the 8822E cross-path capture session.
 *
 *   crosspath --channel N --bw 20|40 [--gain G] [--out DIR] [--cycles K]
 *             [--vid 0xNNNN --pid 0xNNNN]
 *
 * Per cycle: free TX pages, CrossPathBegin, one unparked capture at index 28
 * gain 0, CrossPathPark(true), one capture per index 28..124 step 4 at --gain
 * (default 0), CrossPathEnd, free TX pages, then 2000 MCS7 frames and the
 * send-failure count. Each capture prints its mean power in dB; the unparked
 * and parked index-28 lines are labelled so the park rise reads directly.
 *
 * With --out, captures are written as DVLA files (32-byte header, sample
 * count at byte 12 LE, then one 8-byte word per sample, low dword zero):
 *   i<idx 3 digits>_0x<gain word hex>_<cycle>.bin   parked captures
 *   unparked_i028_0x<gain word hex>_<cycle>.bin     the unparked capture
 * The unparked name does not start with "i" so an `i*.bin` glob reads only
 * the parked set. Name the directory with "_40_" for a 40 MHz run. */
#include <libusb.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "CrossPathCapture.h"
#include "DeviceSession.h"
#include "IRtlRadio.h"
#include "RadiotapBuilder.h"
#include "SignalStop.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "env_config.h"
#include "logger.h"

namespace {

constexpr int kFrames = 2000;
constexpr uint8_t kSmpRate20 = 2, kSmpRate40 = 1, kDmaType = 1;
constexpr uint32_t kTriggerUs = 300;

/* One capture word: low dword zero, high dword rebuilt from the sample. */
bool write_capture(const std::string &path, const std::vector<devourer::IqSample> &s,
                   int bw) {
  FILE *f = std::fopen(path.c_str(), "wb");
  if (!f)
    return false;
  uint8_t hdr[32] = {'D', 'V', 'L', 'A', 1};
  hdr[5] = 1; /* LaTrigMode::AdcMacTrig */
  hdr[6] = 3; /* LaMacSig::Manual */
  hdr[7] = bw == 40 ? kSmpRate40 : kSmpRate20;
  hdr[8] = kDmaType;
  const uint32_t n = static_cast<uint32_t>(s.size());
  std::memcpy(hdr + 12, &n, 4);
  std::memcpy(hdr + 20, &kTriggerUs, 4);
  std::fwrite(hdr, 1, sizeof(hdr), f);
  for (const auto &x : s) {
    const uint32_t hi = (x.bad ? 0u : 1u << 29) |
                        (static_cast<uint32_t>(x.i) & 0xfff) << 12 |
                        (static_cast<uint32_t>(x.q) & 0xfff);
    const uint32_t w[2] = {0, hi}; /* little-endian host assumed (x86, ARM) */
    std::fwrite(w, 4, 2, f);
  }
  return std::fclose(f) == 0;
}

double mean_power_db(const std::vector<devourer::IqSample> &s) {
  double acc = 0;
  for (const auto &x : s)
    acc += double(x.i) * x.i + double(x.q) * x.q;
  return s.empty() ? 0 : 10 * std::log10(acc / s.size());
}

bool pages(IRtlRadio *r, const char *tag, unsigned &free_out) {
  uint16_t fr = 0, tot = 0;
  try {
    if (!r->GetTxQueuePages(fr, tot))
      return false;
  } catch (...) {
    return false;
  }
  free_out = fr;
  std::printf("pages %s free=%u total=%u\n", tag, fr, tot);
  return true;
}

} // namespace

int main(int argc, char **argv) {
  int channel = 144, bw = 20, gain = 0, cycles = 1, vid = 0x0bda, pid = 0xa81a;
  std::string out;
  for (int i = 1; i < argc; ++i) {
    const std::string k = argv[i];
    const bool more = i + 1 < argc;
    if (k == "--channel" && more) channel = std::atoi(argv[++i]);
    else if (k == "--bw" && more) bw = std::atoi(argv[++i]);
    else if (k == "--gain" && more) gain = std::atoi(argv[++i]);
    else if (k == "--cycles" && more) cycles = std::atoi(argv[++i]);
    else if (k == "--out" && more) out = argv[++i];
    else if (k == "--vid" && more) vid = static_cast<int>(std::strtol(argv[++i], nullptr, 0));
    else if (k == "--pid" && more) pid = static_cast<int>(std::strtol(argv[++i], nullptr, 0));
    else {
      std::fprintf(stderr, "bad arg: %s\n", k.c_str());
      return 2;
    }
  }
  if ((bw != 20 && bw != 40) || gain < 0 || gain >= devourer::kCrossPathGains ||
      cycles < 1) {
    std::fprintf(stderr, "need --bw 20|40, --gain 0..%d, --cycles >= 1\n",
                 devourer::kCrossPathGains - 1);
    return 2;
  }

  auto logger = std::make_shared<Logger>();
  apply_logging_env(*logger);
  install_devourer_signal_handlers();
  devourer::DeviceSession session{logger};
  libusb_context *ctx = nullptr;
  if (libusb_init(&ctx) < 0)
    return 1;
  session.adopt_context(ctx);
  libusb_device_handle *handle = libusb_open_device_with_vid_pid(
      ctx, static_cast<uint16_t>(vid), static_cast<uint16_t>(pid));
  if (!handle) {
    logger->error("no adapter {:04x}:{:04x}", vid, pid);
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
  WiFiDriver driver(logger);
  auto owned = driver.CreateRadio(handle, ctx, lock, devourer_config_from_env());
  if (!owned)
    return 1;
  session.adopt_device(std::move(owned));
  IRtlRadio *const dev = dynamic_cast<IRtlRadio *>(session.device());
  if (!dev)
    return 1;
  const ChannelWidth_t bwe = bw == 40 ? CHANNEL_WIDTH_40 : CHANNEL_WIDTH_20;
  /* 40 MHz on 5 GHz: the standard pairing (36/40, 44/48, ..., 157/161), so
   * the upper channel of a pair is HT40- (offset 2) and the lower HT40+. */
  const bool ht40_minus = bw == 40 && channel >= 36 && ((channel - 36) / 4) % 2;
  dev->InitWrite(SelectedChannel{.Channel = static_cast<uint8_t>(channel),
                                 .ChannelOffset = static_cast<uint8_t>(ht40_minus ? 2 : 0),
                                 .ChannelWidth = bwe});
  if (!dev->CrossPathSupported()) {
    std::fprintf(stderr, "cross-path capture unsupported on this adapter\n");
    return 1;
  }

  std::vector<uint8_t> frame =
      devourer::build_stream_radiotap(devourer::parse_tx_mode_str("MCS7"));
  const uint8_t mac[24] = {0x08, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff,
                           0xff, 0xff, 0x57, 0x42, 0x75, 0x05, 0xd6, 0x00,
                           0x57, 0x42, 0x75, 0x05, 0xd6, 0x00, 0x00, 0x00};
  frame.insert(frame.end(), mac, mac + 24);
  uint32_t x = 0x12345678;
  for (int i = 0; i < 1400; ++i) {
    x = x * 1664525u + 1013904223u;
    frame.push_back(static_cast<uint8_t>(x >> 24));
  }

  int bad_cycles = 0;
  for (int c = 0; c < cycles && !g_devourer_should_stop; ++c) {
    bool ok = true;
    unsigned before = 0, after = 0;
    std::printf("cycle %d\n", c);
    ok &= pages(dev, "before", before);
    if (!dev->CrossPathBegin(bwe)) {
      std::printf("begin FAILED\n");
      ++bad_cycles;
      continue;
    }
    std::vector<devourer::IqSample> s;
    auto cap = [&](int idx, int g, bool parked) {
      const bool r = dev->CrossPathCapture(idx, g, s);
      char gw[16];
      std::snprintf(gw, sizeof gw, "0x%x", devourer::kCrossPathGainWord[g]);
      if (!r) {
        std::printf("capture idx=%d gain=%s %s FAILED\n", idx, gw,
                    parked ? "parked" : "unparked");
        ok = false;
        return;
      }
      std::printf("capture idx=%d gain=%s %s n=%zu power=%.2f dB\n", idx, gw,
                  parked ? "parked" : "unparked", s.size(), mean_power_db(s));
      if (!out.empty()) {
        char name[96];
        std::snprintf(name, sizeof name, "%si%03d_%s_%d.bin",
                      parked ? "" : "unparked_", idx, gw, c);
        /* parked name is i028_..., unparked is unparked_i028_... */
        if (!write_capture(out + "/" + name, s, bw)) {
          std::printf("write %s FAILED\n", name);
          ok = false;
        }
      }
    };
    cap(28, 0, false);
    if (!dev->CrossPathPark(true)) {
      std::printf("park FAILED\n");
      ok = false;
    } else {
      for (int idx = 28; idx <= 124; idx += 4)
        cap(idx, gain, true);
    }
    dev->CrossPathEnd();
    ok &= pages(dev, "after", after);
    if (before != after) {
      std::printf("pages MISMATCH before=%u after=%u\n", before, after);
      ok = false;
    }
    int failed = 0;
    for (int i = 0; i < kFrames && !g_devourer_should_stop; ++i)
      if (!dev->send_packet(frame.data(), frame.size()))
        ++failed;
    std::printf("send frames=%d failed=%d\n", kFrames, failed);
    /* Let the burst leave the queue so the next cycle starts on a drained one. */
    for (int t = 0; t < 500 && !g_devourer_should_stop; t += 2) {
      uint16_t fr = 0, tot = 0;
      if (dev->GetTxQueuePages(fr, tot) && fr == tot)
        break;
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (failed)
      ok = false;
    std::printf("cycle %d %s\n", c, ok ? "OK" : "BAD");
    if (!ok)
      ++bad_cycles;
  }
  std::printf("done cycles=%d bad=%d\n", cycles, bad_cycles);
  return bad_cycles ? 1 : 0;
}
