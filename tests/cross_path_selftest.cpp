/* Headless guard for the cross-path types and the 8822E chain-B listener over
 * a recording fake transport that keeps register state. */
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>
#include <ios>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "CrossPathCapture.h"
#include "RadiotapBuilder.h"
#include "Transport.h"
#include "jaguar3/Afe8822e.h"
#include "jaguar3/CrossPath8822e.h"

namespace {

struct Fake : devourer::ITransport {
  std::vector<std::pair<uint16_t, uint32_t>> writes;
  std::map<uint16_t, uint32_t> regs;
  std::map<uint16_t, uint32_t> stuck; // bits that ignore writes
  int write_delay_us = 0;              // a USB-like write cost, so thread order shows

  uint32_t read32(uint16_t a) override { return regs[a]; }
  uint8_t read8(uint16_t a) override { return read32(a) & 0xff; }
  uint16_t read16(uint16_t a) override { return read32(a) & 0xffff; }
  bool write32(uint16_t a, uint32_t v) override {
    if (write_delay_us) std::this_thread::sleep_for(std::chrono::microseconds(write_delay_us));
    writes.push_back({a, v});
    if (a == 0x4108 && !(v >> 20)) regs[0x4c00] = v & 0xfffff; // RF 0x00, path B
    const uint32_t m = stuck[a];
    regs[a] = (regs[a] & m) | (v & ~m);
    return true;
  }
  bool write8(uint16_t a, uint8_t v) override { return write32(a, v); }
  bool write16(uint16_t a, uint16_t v) override { return write32(a, v); }

  std::vector<uint32_t> to(uint16_t a) const {
    std::vector<uint32_t> r;
    for (auto &w : writes) if (w.first == a) r.push_back(w.second);
    return r;
  }

  bool is_usb() const override { return false; }
  bool write_bytes(uint16_t, const uint8_t *, size_t) override { return false; }
  bool tx_async(uint8_t, uint8_t *, size_t, unsigned) override { return false; }
  int tx_sync(uint8_t, uint8_t *, size_t, int) override { return -1; }
  void rx_loop(int, int, const std::function<void(const uint8_t *, int)> &,
               const std::function<bool()> &) override {}
};

int fails = 0;
#define CHECK(c)                                                               \
  do {                                                                         \
    if (!(c)) {                                                                \
      std::fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #c);         \
      ++fails;                                                                 \
    }                                                                          \
  } while (0)

} // namespace

int main() {
  using namespace devourer;
  auto s = cross_path_sample((uint64_t{1} << 61) | (uint64_t{0x7ff} << 44) | (uint64_t{0x800} << 32));
  CHECK(s.i == 2047 && s.q == -2048 && !s.bad);
  CHECK(cross_path_sample(uint64_t{0xfff} << 44).i == -1);
  CHECK(cross_path_sample(0).bad);

  TxMode m20, m40;
  m20.mode = m40.mode = TxMode::Mode::HT;
  m20.ht_mcs = m40.ht_mcs = 1;
  m40.bw_mhz = 40;
  CHECK(cross_path_probe_frame(CHANNEL_WIDTH_20) == cross_path_probe_frame(CHANNEL_WIDTH_20));
  CHECK(cross_path_probe_frame(CHANNEL_WIDTH_20).size() == build_stream_radiotap(m20).size() + 24 + 150);
  CHECK(cross_path_probe_frame(CHANNEL_WIDTH_40).size() == build_stream_radiotap(m40).size() + 24 + 320);
  {
    const auto f = cross_path_probe_frame(CHANNEL_WIDTH_20);
    const size_t o = build_stream_radiotap(m20).size();
    const uint8_t hdr[24] = {0x08, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff,
                             0xff, 0xff, 0x57, 0x42, 0x75, 0x05, 0xd6, 0x00,
                             0x57, 0x42, 0x75, 0x05, 0xd6, 0x00, 0x00, 0x00};
    const uint8_t body[8] = {117, 205, 37, 75, 132, 226, 234, 242};
    CHECK(std::equal(hdr, hdr + 24, f.begin() + o));
    CHECK(std::equal(body, body + 8, f.begin() + o + 24));
  }
  CHECK(cross_path_samples(CHANNEL_WIDTH_20) == 8192);
  CHECK(cross_path_samples(CHANNEL_WIDTH_40) == 16384);

  auto fake = std::make_shared<Fake>();
  fake->regs[0x820] = 0x12345632;
  fake->regs[0x410c] = 0xabcd0001; // [1:0] = 1: undo restores this, not 3
  fake->regs[0x4c] = 0x00a00000;
  fake->regs[0x3c] = 0x04000011;
  fake->regs[0x4c00] = 0x30123;
  const uint32_t s4c = 0x00a00000, s3c = 0x04000011;
  jaguar3::CrossPathListener8822e l(RtlAdapter(fake, std::make_shared<Logger>()));

  l.undo();
  CHECK(fake->writes.empty() && !l.active());
  CHECK(!l.park(true) && fake->writes.empty());

  l.setup();
  std::vector<uint32_t> want;
  for (uint32_t n = 0; n < 16; ++n) want.push_back(0x700F0001u + 0x100000u * n);
  CHECK(fake->to(0x4130) == want);
  CHECK(want[1] == 0x701F0001u && want[15] == 0x70FF0001u);
  CHECK((fake->to(0x1c38) == std::vector<uint32_t>{0, 0xffffffff}));
  CHECK((fake->regs[0x410c] & 3) == 0 && fake->regs[0x410c] >> 2 == 0xabcd0001u >> 2);
  CHECK((fake->regs[0x820] & 0xff) == 0x31 && (fake->regs[0x820] >> 8) == 0x123456);
  CHECK(l.active());

  CHECK(l.park(true));
  CHECK((fake->regs[0x4c] & (1u << 22)) && !(fake->regs[0x3c] & (1u << 26)));
  l.set_gain(2);
  CHECK(fake->to(0x4108).back() == 0x300c0);
  fake->writes.clear();
  CHECK(l.undo());
  auto w = fake->to(0x4130);
  CHECK(w.size() == 18 && w.back() == 0x70fb0001);
  CHECK(std::equal(w.begin(), w.end(), jaguar3::kAfeRestoreSeqB)); // no TXGAPK record: IQK table
  CHECK(fake->regs[0x1c38] == 0xffa1005e && fake->regs[0x410c] == 0xabcd0001u);
  CHECK(fake->regs[0x820] == 0x12345632);
  CHECK(fake->regs[0x4c] == s4c && fake->regs[0x3c] == s3c);
  CHECK(fake->regs[0x4c00] == 0x30123); // path-B RF 0x00 back after set_gain(2)
  CHECK(!l.active());
  fake->writes.clear();
  CHECK(l.undo());
  CHECK(fake->writes.empty());

  // bring-up's last 0x4130 writer was TXGAPK: undo replays that table
  {
    bool txgapk = true;
    jaguar3::CrossPathListener8822e lt(RtlAdapter(fake, std::make_shared<Logger>()),
                                       [&] { return txgapk; });
    lt.setup();
    fake->writes.clear();
    CHECK(lt.undo());
    auto t = fake->to(0x4130);
    CHECK(t.size() == 16 && std::equal(t.begin(), t.end(), jaguar3::kAfeTxgapkRestoreSeq));
    CHECK(t[0] == 0x700b8041 && t[15] == 0x70fb8041);
    txgapk = false; // the record is read at setup: IQK ran last this time
    lt.setup();
    fake->writes.clear();
    CHECK(lt.undo());
    t = fake->to(0x4130);
    CHECK(t.size() == 18 && std::equal(t.begin(), t.end(), jaguar3::kAfeRestoreSeqB));
  }

  // a restored bit that does not read back: undo reports it and still closes
  fake->regs[0x410c] = 0xabcd0003;
  l.setup();
  fake->stuck[0x410c] = 3; // [1:0] stay 0
  CHECK(!l.undo() && !l.active());
  fake->stuck.clear();
  fake->regs[0x410c] = 0xabcd0001;

  // a park that does not take
  fake->stuck[0x3c] = 1u << 26;
  l.setup();
  CHECK(!l.park(true));
  CHECK(fake->regs[0x4c] == s4c && fake->regs[0x3c] == s3c);
  l.undo();
  CHECK(fake->regs[0x4c] == s4c);

  // 0x4c[22] sticks low: park clears 0x3c[26] first, the restore must set it back
  fake->stuck.clear();
  fake->stuck[0x4c] = 1u << 22;
  l.setup();
  CHECK(!l.park(true));
  CHECK(fake->regs[0x4c] == s4c && fake->regs[0x3c] == s3c);
  l.undo();

  // the session: order of work, guards, capture parameters
  {
    fake->stuck.clear();
    fake->write_delay_us = 300;
    std::vector<std::string> log;
    std::atomic<size_t> send_count{0}, sends_seen_while_listener_inactive{0};
    int cur_index = 77, get_index_count = 0;
    uint32_t gain_at_capture = 0;
    bool relayout_fails = false, capture_times_out = false;
    bool relayout_throws_half = false, relayout_throws_full = false;
    bool capture_throws = false, drained_throws = false, set_index_throws = false;
    std::atomic<bool> send_throws{false};
    std::atomic<int> undrained_polls{0}; // drained() reads false this many more times
    LaParams last;
    jaguar3::CrossPathListener8822e listener(RtlAdapter(fake, std::make_shared<Logger>()));
    jaguar3::CrossPathListener8822e listener2(RtlAdapter(fake, std::make_shared<Logger>()));
    auto make_io = [&](jaguar3::CrossPathListener8822e &lis) {
      jaguar3::CrossPathSession::Io io;
      io.relayout = [&, l = &lis](bool half) {
        log.push_back(std::string("relayout ") + (half ? "half" : "full") +
                      " listener=" + (l->active() ? "1" : "0"));
        if (half ? relayout_throws_half : relayout_throws_full)
          throw std::ios_base::failure("control transfer failed");
        return !relayout_fails;
      };
      io.get_index = [&] {
        ++get_index_count;
        return cur_index;
      };
      io.set_index = [&](int i) {
        if (set_index_throws)
          throw std::ios_base::failure("control transfer failed");
        cur_index = i;
        log.push_back("set_index " + std::to_string(i));
      };
      io.capture = [&](const LaParams &p) {
        log.push_back("capture");
        if (capture_throws)
          throw std::ios_base::failure("control transfer failed");
        last = p;
        gain_at_capture = fake->regs[0x4c00];
        LaResult r;
        if (capture_times_out) {
          r.poll_timeout = true;
          return r;
        }
        r.ok = true;
        r.samples.assign(p.max_samples, uint64_t{1} << 61);
        return r;
      };
      io.send = [&, l = &lis](const uint8_t *, size_t) {
        if (!l->active()) ++sends_seen_while_listener_inactive;
        ++send_count;
        if (send_throws)
          throw std::ios_base::failure("bulk transfer failed");
        return true;
      };
      io.drained = [&] {
        if (drained_throws)
          throw std::ios_base::failure("control transfer failed");
        if (undrained_polls > 0) {
          --undrained_polls;
          log.push_back("undrained");
          return false;
        }
        return true;
      };
      return io;
    };
    auto sleep_ms = [](int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); };
    jaguar3::CrossPathSession s(make_io(listener), listener);
    jaguar3::CrossPathSession s2(make_io(listener2), listener2);
    std::vector<IqSample> out;

    CHECK(!s.park(true) && !s.capture(60, 0, out) && log.empty()); // no session open
    CHECK(!s.begin(CHANNEL_WIDTH_80) && !s.begin(CHANNEL_WIDTH_10) && log.empty());
    // a queue that never drains: begin refuses and writes nothing
    undrained_polls = 1 << 30;
    fake->writes.clear();
    CHECK(!s.begin(CHANNEL_WIDTH_20) && !s.active());
    CHECK(fake->writes.empty() && get_index_count == 0 && send_count == 0);
    CHECK(std::all_of(log.begin(), log.end(), [](const std::string &e) { return e == "undrained"; }));
    undrained_polls = 0;
    log.clear();
    CHECK(s.begin(CHANNEL_WIDTH_20) && s.active());
    CHECK(get_index_count == 1);
    CHECK(log[0] == "relayout half listener=0");
    while (send_count < 3) sleep_ms(1);
    CHECK(sends_seen_while_listener_inactive == 0);
    CHECK(s.park(true) && s.park(false));
    undrained_polls = 3; // frames still queued when the train stops
    CHECK(s.end());
    CHECK(log[log.size() - 2] == "relayout full listener=0");
    CHECK(undrained_polls == 0);
    {
      const auto full = std::find(log.begin(), log.end(), "relayout full listener=0");
      CHECK(std::count(log.begin(), full, "undrained") == 3); // all before the re-layout
    }
    CHECK(log.back() == "set_index 77");
    CHECK(!s.active() && !listener.active());
    const size_t sends = send_count;
    sleep_ms(20);
    CHECK(send_count == sends);
    log.clear();
    CHECK(s.end()); // no session open: true, nothing done
    CHECK(log.empty());

    CHECK(s.begin(CHANNEL_WIDTH_20));
    CHECK(get_index_count == 2);
    log.clear();
    CHECK(!s.begin(CHANNEL_WIDTH_40));
    CHECK(log.empty() && get_index_count == 2); // the refused begin wrote nothing
    CHECK(s.width() == CHANNEL_WIDTH_20 && s.active() && listener.active());

    relayout_fails = true;
    CHECK(!s2.begin(CHANNEL_WIDTH_20));
    CHECK(!s2.active() && !listener2.active());
    relayout_fails = false;

    log.clear();
    CHECK(s.capture(60, 1, out));
    CHECK(last.trig_mode == LaTrigMode::AdcMacTrig && last.mac_sig == LaMacSig::Manual);
    CHECK(last.dma_type == 1 && last.dbg_port == 0xa80 && last.smp_rate == 2 && last.max_samples == 8192);
    CHECK(last.block_len == 248 && last.trigger_time_us == 300);
    CHECK(last.poll_ms == 2 && last.poll_tries == 2000);
    CHECK(out.size() == 8192 && !out[0].bad);
    CHECK((log == std::vector<std::string>{"set_index 60", "capture"}));
    CHECK(fake->to(0x4108).back() == kCrossPathGainWord[1]);
    CHECK(gain_at_capture == (kCrossPathGainWord[1] & 0xfffff)); // gain written before the capture
    log.clear();
    CHECK(s.capture(60, 2, out));
    CHECK((log == std::vector<std::string>{"capture"})); // same index: no re-set
    CHECK(fake->to(0x4108).back() == kCrossPathGainWord[2]);

    capture_times_out = true;
    CHECK(!s.capture(64, 1, out) && out.empty());
    s.end();
    CHECK(log.back() == "set_index 77");
    CHECK(!listener.active());

    log.clear();
    CHECK(!s.capture(128, 0, out) && !s.capture(60, 4, out));
    CHECK(s.begin(CHANNEL_WIDTH_40));
    log.clear();
    capture_times_out = false;
    CHECK(!s.capture(128, 0, out) && !s.capture(-1, 0, out) && !s.capture(60, 4, out) &&
          !s.capture(60, -1, out) && log.empty());
    CHECK(s.capture(60, 0, out) && last.smp_rate == 1 && last.max_samples == 16384);
    CHECK(s.end());

    // throwing Io callables: every call returns, none throws
    log.clear();
    drained_throws = true; // begin: the drain check throws, nothing written
    fake->writes.clear();
    CHECK(!s.begin(CHANNEL_WIDTH_20) && !s.active() && log.empty() && fake->writes.empty());
    drained_throws = false;

    relayout_throws_half = true; // begin: the half re-layout throws
    CHECK(!s.begin(CHANNEL_WIDTH_20) && !s.active() && !listener.active());
    CHECK(log.back() == "relayout full listener=0"); // a half-applied layout goes back
    relayout_throws_half = false;

    CHECK(s.begin(CHANNEL_WIDTH_20));
    capture_throws = true;
    out.assign(5, IqSample{});
    CHECK(!s.capture(64, 1, out) && out.empty() && s.active());
    capture_throws = false;
    set_index_throws = true;
    CHECK(!s.capture(68, 1, out) && out.empty() && s.active());
    set_index_throws = false;
    CHECK(s.park(true));

    send_throws = true; // the train survives a throwing send
    const size_t sc = send_count;
    while (send_count < sc + 3) sleep_ms(1);
    send_throws = false;

    relayout_throws_full = true; // end: the full re-layout throws
    log.clear();
    CHECK(!s.end() && s.active() && !listener.active());
    CHECK(log.back() == "set_index 77"); // later steps still ran
    relayout_throws_full = false;
    log.clear();
    CHECK(s.end() && !s.active()); // the retry closes it
    CHECK((log == std::vector<std::string>{"relayout full listener=0", "set_index 77"}));
    CHECK((fake->regs[0x4c] & (1u << 22)) == (s4c & (1u << 22)));

    CHECK(s.begin(CHANNEL_WIDTH_20));
    drained_throws = true; // end: a throwing drain check counts as not drained
    log.clear();
    CHECK(!s.end() && !s.active());
    CHECK((log == std::vector<std::string>{"relayout full listener=0", "set_index 77"}));
    drained_throws = false;

    CHECK(s.begin(CHANNEL_WIDTH_20));
    undrained_polls = 1 << 30; // end: the queue never drains
    log.clear();
    CHECK(!s.end() && !s.active() && !listener.active());
    CHECK(log.back() == "set_index 77");
    CHECK(std::count(log.begin(), log.end(), "relayout full listener=0") == 1);
    undrained_polls = 0;
  }

  if (fails) return 1;
  std::puts("cross_path: ok");
  return 0;
}
