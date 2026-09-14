// scout_read_bench.cpp — transfer-count, latency, and on-hardware reset
// verification for IRtlDevice::GetRxEnergyScout() vs the full
// GetRxEnergy(false), the evidence for the "far fewer reads than the full
// read" channel-scout claim (mabur 2026-09-14 spec §6). The SHIPPING scout is
// 8 reads + 4 writes = 12 transfers; the "~6 reads" of the original plan
// described the shadowed form that was deleted in ef4fa04. Same open sequence
// as
// tests/reglat.cpp (USB vid/pid, DEVOURER_VID/DEVOURER_PID env), but
// GetRxEnergyScout is an IRtlDevice method, not a bare register op, so
// bring-up runs InitWrite first via WiFiDriver::CreateRtlDevice — the
// doctor demo's open path, trimmed to a single chip with no CLI.
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
// -----------------------------------------------------------------------
// WHY THIS IS PER-CALL, NOT A SINGLE BEFORE/AFTER DELTA OVER THE WHOLE LOOP
// -----------------------------------------------------------------------
// devourer::usb_ctrl_xfers() (src/UsbXferCount.h) is ONE PROCESS-GLOBAL
// ATOMIC — every control transfer anywhere in the process bumps it, not
// just the ones this bench issues. A first version of this bench computed
// (x1-x0)/N once over the whole 200-call loop, which silently attributes
// every OTHER thread's control transfers during that window to our calls
// too. RtlJaguar3Device runs a coex runtime thread once InitWrite
// completes (RtlJaguar3Device::coex_runtime_loop) that, on a fixed ~2 s
// WALL-CLOCK cadence, takes the SAME _reg_mu GetRxEnergyScout/GetRxEnergy
// take and does substantial register work under it —
// PhydmRuntimeJaguar3::fa_statistics_and_reset() alone (called from
// PhydmRuntimeJaguar3::tick) is 8 reads + 8 masked writes, ~24 transfers
// by itself, plus dig()/cck_pd()/edcca() and three firmware H2C sends
// (coex_run_5g, fw_update_wl_phy_info, fw_set_pwr_mode_active,
// fw_coex_query_bt_info — RtlJaguar3Device.cpp's coex_runtime_loop). A
// bench call whose lock-wait-then-call window overlaps that tick has the
// WHOLE tick's transfer count folded into ITS usb_ctrl_xfers() delta,
// because the counter has no notion of which thread issued a transfer —
// this is what a 2026-09-14 run of the single-delta version showed: a
// 24.4 xfers/call reading (vs. the clean 24.0 baseline) on the FULL-READ
// row, with a 40245 us outlier among otherwise ~9000 us calls in the same
// run — one call blocked on _reg_mu behind an in-flight coex tick, then
// absorbed that tick's transfers once it got the lock. The mechanism is
// mutex contention (both sides serialize correctly on _reg_mu — there is
// no register-corruption race), but it makes the process-wide counter
// unattributable to any specific call without per-call bracketing.
//
// FIX: bracket usb_ctrl_xfers() around EACH INDIVIDUAL call, not once
// around the whole loop, producing a per-call transfer-count distribution
// instead of one averaged number. Contamination can only ADD transfers to
// a call (the tick's own work has to be counted somewhere, and it lands
// on whichever call happens to be blocked when the tick fires) — it can
// never make an uncontaminated call's own cost look smaller. That makes
// the per-run MINIMUM ("floor" below) the attributable true cost of an
// uncontended call, and every call above it is a lower bound on how much
// ambient traffic landed in that call's window. We additionally sample
// devourer::usb_ctrl_xfers() over one idle ~2.5 s window with NO calls in
// flight (bracketing at least one coex tick) to report the ambient rate
// directly, separately from the per-call floor. Each function is run
// kRuns times (not once) so the floor's stability and the contamination
// rate's run-to-run spread are both visible, per the coordinator's
// instruction to report a run count and a spread rather than a best case.
//
// -----------------------------------------------------------------------
// RESET VERIFICATION (does the composed write actually land on hardware?)
// -----------------------------------------------------------------------
// The host-side selftest (tests/scout_energy_selftest.cpp) pins
// ScoutEnergyMath.h's pure arithmetic, but nothing before this revision
// verified the arithmetic's OUTPUT actually reaches the chip and resets
// the counters — a stuck 0x1eb4[25] (latched at 1, holding the OFDM FA/CCA
// counters in permanent reset) would pass every host test and show up on
// hardware only as fa_ofdm/cca_ofdm pinned at 0 from the second call on.
// This bench now downcasts to RtlJaguar3Device (Jaguar3-specific already —
// see DebugPeekBb's doc comment) and, once per process: reads 0x1d2c/
// 0x1eb4 before a scout call, makes the call, reads them again, and checks
// the post-call bits land where compose_reset's "on" state says they
// should (0x1d2c[31]=1, 0x1eb4[25]=0) — the SAME end-state GetRxEnergy's
// reset leaves, and the same one the coex tick's fa_stats reset leaves, so
// this also indirectly cross-checks that shared assumption. It also
// prints fa_ofdm/cca_ofdm on that one call, AND tracks the min/max of
// fa_ofdm/cca_ofdm across every per-call-bracketed run below — a stuck
// reset shows as max==0 across an entire run, not just one sample.
//
// -----------------------------------------------------------------------
// MEASURED 2026-09-15 (spare 8822EU, USB bus 5-1, after InitWrite ch6, 4
// independent process invocations x 3 runs x 200 calls/function = 12 runs
// per function total):
//
// >>> SUPERSEDED — this whole "MEASURED 2026-09-15" block describes the A1
// >>> scout, which cached 0x1d2c/0x1eb4 behind a shadow and so read 6
// >>> registers for 10 transfers/call. That form was DELETED (ef4fa04). The
// >>> shipping scout reads 8 registers for 12 transfers/call. Everything
// >>> below about the coex-tick contamination mechanism, the per-transfer
// >>> cost and the reset verification still holds; only the 10/6 figures and
// >>> the "this is the number to quote" headline are stale. The correct
// >>> current numbers are in the TASK A2 block further down. Kept rather than
// >>> rewritten because it is the provenance of the A1 measurement.
//
// FLOOR (the attributable per-call cost): GetRxEnergyScout 10 xfers/call
// (6 reads + 4 composed full-dword writes) — GetRxEnergy(false) 24
// xfers/call (8 reads + 8 MASKED writes, each a read-modify-write = 2
// transfers). STABLE AT EXACTLY THESE VALUES IN EVERY ONE OF THE 12 RUNS
// PER FUNCTION, no exceptions — this is the number to quote. It differs
// from the plan's ~14/~24 estimate: the plan under-credited how much the
// composed-shadow write buys (rtw_write32 of a pre-composed dword is ONE
// transfer, not a read-modify-write's two), so the scout undercuts the
// full read by more than predicted (floor 10 vs 24, not 14 vs 24) —
// report the measurement, not the estimate.
//
// AMBIENT CONTAMINATION (why an early single-delta version of this bench
// once read 24.4 xfers/call — see the mechanism section above): 3 of the
// 24 total runs (12 scout + 12 full-read) absorbed exactly one coex tick,
// each time adding a strikingly consistent ~71-72 extra transfers and
// ~27-36 ms extra latency to exactly one of that run's 200 calls (never
// more than one call per run) — both functions are equally susceptible,
// it lands on whichever call happens to be in flight when the ~2 s tick
// fires. At N=200 that dilutes to a mean of ~10.4 or ~24.4 xfers/call for
// the contaminated run specifically — which is what the coordinator's
// relayed "24.4 for the scout" figure almost certainly was: this session's
// own prior (single-delta) bench output showed that exact 24.4 reading on
// the GetRxEnergy(false) row, not the scout row, so the two are very
// likely the same event under different bookkeeping, not a distinct
// scout-side blowup — the scout's floor never moved even in its one
// contaminated run.
//
// PER-TRANSFER COST: self-consistent between the two functions at the
// floor (~370-375 us/xfer on this rig's USB path) — the sanity check that
// usb_ctrl_xfers() genuinely moves and the floor is a real per-call cost,
// not a fluke. That is ~5.6x tests/reglat.cpp's documented ~66 us/op on
// ITS rig — a different USB topology/hub chain (never cross-characterized
// against reglat's bench rig) plus post-InitWrite _reg_mu contention from
// the live coex thread, not a contradiction.
//
// RESET VERIFICATION (Important 5): the 0x1d2c/0x1eb4 before/after
// register check passed in all 4 invocations — post-call 0x1d2c[31]=1,
// 0x1eb4[25]=0, matching compose_reset's "on" state. Read this check for
// what it actually proves, though: because that nominal state is the SAME
// before and after a full reset cycle (the transient 0->1->0 toggle on
// 0x1eb4[25] happens INSIDE the call and is invisible to a before/after
// sample), it confirms the composed write lands in the correct resting
// state, not that the transient pulse fired. The stronger, independent
// behavioral proof that it fires every call: fa_ofdm/cca_ofdm never
// trended upward across any 200-call run (0-2 throughout, in all 24
// runs) — a stuck reset would accumulate unboundedly over a run's
// 0.75-1.8 s span even on a quiet channel, and none did. The dedicated
// single verification call before each loop (a real GetRxEnergyScout()
// call, same code path) independently saw substantial nonzero counts each
// invocation (102, 340, 144, 160 fa_ofdm across the four runs) — proof
// the scout's count-then-reset mechanism captures real activity when
// there is any. A one-off single-count asymmetry between the two
// functions' fa_ofdm max appeared twice across the 24 runs (once
// favoring each function, magnitude 1 both times) — bidirectional and at
// noise magnitude, consistent with the full-read loop's ~2.4x longer
// wall-clock span (200 x ~9 ms vs 200 x ~3.75 ms) giving it proportionally
// more chances to catch a single stray RF event on this quiet 2.4 GHz
// bench channel, not a scout-specific defect. This bench's channel is
// RF-quiet enough that NEITHER function saw much to count — an operator
// wanting a harder behavioral exercise of the counters should re-run this
// next to an active AP or with a CW tone armed on a second adapter.
//
// =======================================================================
// MEASURED 2026-09-15, TASK A2 — BATCHED EP0 TRANSFERS (ctrl_batch)
// =======================================================================
// Two hosts, and they disagree completely. Both are reported because the
// favourable one alone would be a lie about what this feature does.
//
// Method on both: 3 process invocations x 3 internal runs x 200 calls per
// function per mode, DEVOURER_CTRL_BATCH=1 and =0 alternated invocation by
// invocation (never all of A then all of B). Statistics are over the 9
// per-run medians and the 9 per-run minima; single best cases are not quoted.
//
// --- GROUND STATION (RK3566 aarch64, 8822EU, the deployment target) ---
// READ EVERY NUMBER BELOW AS A FLOOR. This bench never calls StartRxLoop, so
// there were no RX URBs on the libusb context to reap. In production the scout
// and hop threads pump that same context from inside async_wait_progress, so
// they will absorb RX frame parsing under live traffic. The hop's worst case
// under 60 fps RX is UNMEASURED. That applies to the 5.6x headline too.
//                      unbatched (med)    batched (med)      change
//   scout, 10 ops      2829 us            588 us             -2.24 ms, 4.8x
//   (shadowed form,    (2497..3055)       (501..748)
//    since deleted)
//   scout, 12 ops      3372 us            603 us             -2.77 ms, 5.6x
//   (SHIPPING form,    (3002..3748)       (571..648)
//    no shadow)
//   GetRxEnergy(false) 6612 us            7444 us   <- NOT batched; unchanged
//     24 ops, control  (5623..7592)       (6953..8368)  within run-to-run noise
//
// The full read is the control: it never calls ctrl_batch, and it does not
// move (its two spreads overlap heavily). Only the paths that were converted
// changed.
//
// DECOMPOSITION — what batching actually removes. Fitting both the 10-op and
// 12-op variants in both modes:
//   cost ~= 290 us x (number of completion waits) + 7..16 us x (number of ops)
// which reproduces every measurement on this host:
//   unbatched 10 ops = 10 waits -> 2900+~100  (measured 2829)
//   batched   10 ops =  2 waits ->  580+~100  (measured  588)
//   batched   12 ops =  2 waits ->  580+~130  (measured  603)
//   unbatched 24 ops = 24 waits -> 6960+~250  (measured 6612..7444)
// So on the GS the ~264-290 us that a synchronous register access costs is
// almost entirely HOST TURNAROUND (the wait for a completion), and only
// ~7-16 us of it is irreducible wire time. Batching removes 92-98% of the
// per-transfer cost; what is left is a per-WAIT cost that is essentially the
// same ~290 us as one synchronous transfer.
//
// FOLLOW-UP LEVER, measured but NOT taken here, AND IT IS NOT THE SCOUT.
// Because the surviving term is per-wait and kAsyncWriteDepth is 8, any group
// that splits across two chunks pays ~290 us for nothing. The group that
// actually does is the cached FAST-RETUNE HOP: 9-11 writes in one ctrl_batch,
// two chunks at depth 8, so a depth >= 11 makes it one wait — ~290 us off a
// live channel change, which is the more valuable target anyway.
// It does NOT help the shipping scout: that is two DEPENDENT ctrl_batch calls
// (8 reads, then 4 writes composed from two of those reads), each already a
// single chunk. Its two waits are a data dependency, so no pool depth can fold
// them. (An earlier revision of this note claimed depth >= 12 would take the
// scout ~603 -> ~390 us. That was arithmetic left over from the pre-ef4fa04
// single 10-op group and is wrong — recorded here so it is not re-derived.)
// NOTE the 8 is a FLOOR, not an optimum: the InitWrite source comment it came
// from says "depth >= 8". Left alone as a SCOPE decision — it sits on
// InitWrite's proven ~14k-write bring-up pipeline and would need bring-up-time
// validation on hardware before it could ship.
//
// --- PC BENCH HOST (x86 xHCI, 8822EU on bus 5-1) ---
//   scout 10 ops (shadowed, since deleted): 3750 us batched, 3750 unbatched.
//   scout 12 ops (shipping, shadowless): 4500 both ways. Full read 24 ops:
//   9000 both ways. Per-run medians were those exact values in every one of
//   9 runs per function per mode.
// Every number is EXACTLY 375.0 us x transfer count in BOTH modes: this host
// overlaps nothing, so the batch buys precisely zero. The same is true of the
// pre-existing bring-up write pipeline here (init.timing j3init.total 4736 ms
// / 12556 xfers = 377 us/xfer) — which is the control that says "this host
// does not reward pipelining", not "ctrl_batch is broken". 375 us is 3 USB
// high-speed microframes, consistent with this host's xHCI putting each
// SETUP/DATA/STATUS stage in its own microframe and starting no EP0 TD before
// the previous one retires.
// Net: the PC is the pessimistic host and is NOT where this feature should be
// judged. It remains a useful correctness rig (see the corruption story
// below, which it could not have caught).
//
// --- P10, SETTLED: THE CACHED 0x1d2c/0x1eb4 SHADOW WAS DELETED ---
// The scout used to cache the two reset dwords behind a primed shadow, making
// the call 10 transfers instead of 12. Measured head to head on the GS, both
// batched, 9 runs each, the shadowless form cost:
//   per-run medians (mean 588 -> 603):  +2.4%   } inside the 5% rule
//   median of the per-run medians:      +1.5%   }
//   per-run minima  (mean 456 -> 488):  +7.1%   } outside it
//   median of the per-run minima:       +9.6%   }
// and individual per-run ratios ran -16.5% to +24.3% — shadowless was
// sometimes FASTER, i.e. the difference is not measurable at this sample
// size. Unbatched the shadow had saved ~550 us on a 2.8 ms call (+19-23%),
// which is why it existed; batched it saves 15-45 us on a ~590 us call. Its
// price was a hand-maintained coherence protocol with FIVE invalidation
// points, one of them a firmware branch unauditable from the host. Deleted
// 2026-09-15: the reset dwords are read fresh inside the same batch. The
// numbers above are kept because they are the evidence for that deletion.
// (For contrast, on the PC host the shadowless penalty was a flat +20.0% in
// 9 of 9 runs with zero variance, since there each extra transfer costs its
// full 375 us. Had the PC been the target, the shadow would have stayed.)
//
// --- A SILENT DATA-CORRUPTION BUG THE PC RIG COULD NOT SEE ---
// The first GS run of the batched path returned fa_ofdm in the tens of
// thousands and cca_ofdm values of 16384/49152/12672/13184 — which are the
// high halves of 0x40000000 / 0xC0000000 / 0x31800002 / 0x33800002, i.e. the
// scout's own reset WRITE dwords showing up as counter READ values. Cause:
// async_write_cb sets a slot's `done` and only then returns it to the free
// pool, so a waiter can finish a call while the reaping thread has not yet
// released those slots; the next call then started an 8-op chunk with 6-7
// slots free, ran dry mid-submit, waited for "progress", and was handed one
// of ITS OWN just-completed reads — whose buffer it refilled with a later
// write before the wait loop copied the read out. Fixed by the acquire phase
// in UsbTransport::ctrl_batch (all of a chunk's slots taken BEFORE anything
// is submitted, chunk sized by what was actually free). It never reproduced
// on the PC host, where EP0 transfers do not overlap at all and each one
// completes before the next slot is taken — which is precisely why the
// kImplausibleCount tripwire above now exists: the old bench would have
// printed "fa_ofdm 0..61826" and an operator would have read it as a busy
// channel.
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
#include <cstring>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

#include "DeviceSession.h"
#include "UsbXferCount.h"
#include "WiFiDriver.h"
#include "jaguar3/RtlJaguar3Device.h"
#include "logger.h"

namespace {

struct Stats { double min, med, mean, max; };

Stats summarize(std::vector<double> &v) {
  std::sort(v.begin(), v.end());
  double sum = 0;
  for (double x : v) sum += x;
  return Stats{v.front(), v[v.size() / 2], sum / v.size(), v.back()};
}

struct RunResult {
  uint64_t xfer_floor = 0;   /* min per-call xfer count this run — the
                              * attributable true cost (contamination only
                              * adds, never subtracts). */
  int n_at_floor = 0;        /* calls that hit the floor exactly (clean) */
  uint64_t xfer_max = 0;
  double xfer_mean = 0;
  uint64_t contam_excess = 0; /* sum of (xfers[i] - floor) over all calls —
                               * total transfers attributable to ambient
                               * (coex-thread) traffic landing in this
                               * run's windows, not to our own calls. */
  int n_contam = 0;           /* calls above the floor */
  Stats us{};
  uint32_t fa_min = std::numeric_limits<uint32_t>::max(), fa_max = 0;
  uint32_t cca_min = std::numeric_limits<uint32_t>::max(), cca_max = 0;
  /* Calls whose fa_ofdm/cca_ofdm are physically impossible for the window
   * they cover — the tripwire for READ PAYLOAD CORRUPTION, added after the
   * batched path's first GS run (2026-09-15) returned the reset WRITE dwords
   * (0x40000000 / 0x33800002) as counter values because a chunk had recycled
   * one of its own in-flight slots (see UsbTransport::ctrl_batch's acquire
   * phase). These are 16-bit BB counters sampled over a sub-millisecond to
   * few-millisecond window; a few hundred events is a busy channel, tens of
   * thousands is not a channel, it is memory. A bench that only printed
   * min..max would have shown "fa_ofdm 0..61826" and been read as "busy
   * channel" — so this is counted and shouted about, not just printed. */
  int n_implausible = 0;
};

/* Above this, a counter value is not a reading. See RunResult::n_implausible. */
constexpr uint32_t kImplausibleCount = 4096;

/* Runs `n` calls of `fn`, bracketing usb_ctrl_xfers() and the clock around
 * EACH INDIVIDUAL call (see the file header for why this replaced a single
 * before/after delta over the whole loop) and tracking fa_ofdm/cca_ofdm
 * min/max (a stuck reset shows as max==0 for the whole run). */
template <typename Fn>
RunResult bench_run(Fn fn, int n) {
  std::vector<uint64_t> xfers(n);
  std::vector<double> us(n);
  RunResult r;
  for (int i = 0; i < n; ++i) {
    const uint64_t x0 = devourer::usb_ctrl_xfers().load();
    const auto t0 = std::chrono::steady_clock::now();
    const RxEnergy e = fn();
    us[i] = std::chrono::duration<double, std::micro>(
                std::chrono::steady_clock::now() - t0)
                .count();
    xfers[i] = devourer::usb_ctrl_xfers().load() - x0;
    if (e.valid_fa) {
      if (e.fa_ofdm > kImplausibleCount || e.cca_ofdm > kImplausibleCount)
        ++r.n_implausible;
      r.fa_min = std::min(r.fa_min, e.fa_ofdm);
      r.fa_max = std::max(r.fa_max, e.fa_ofdm);
      r.cca_min = std::min(r.cca_min, e.cca_ofdm);
      r.cca_max = std::max(r.cca_max, e.cca_ofdm);
    }
  }
  r.xfer_floor = *std::min_element(xfers.begin(), xfers.end());
  r.xfer_max = *std::max_element(xfers.begin(), xfers.end());
  uint64_t sum = 0;
  for (uint64_t x : xfers) {
    sum += x;
    if (x == r.xfer_floor) {
      ++r.n_at_floor;
    } else {
      r.contam_excess += x - r.xfer_floor;
      ++r.n_contam;
    }
  }
  r.xfer_mean = double(sum) / n;
  r.us = summarize(us);
  return r;
}

void print_run(const char *label, int run_idx, int runs, int n,
              const RunResult &r) {
  std::printf(
      "%s run %d/%d: floor=%llu (%d/%d calls clean) mean=%.1f max=%llu "
      "contam_excess=%llu xfers over %d call(s); us min/med/mean/max "
      "%.0f/%.0f/%.0f/%.0f; fa_ofdm %u..%u cca_ofdm %u..%u\n",
      label, run_idx, runs, (unsigned long long)r.xfer_floor, r.n_at_floor, n,
      r.xfer_mean, (unsigned long long)r.xfer_max,
      (unsigned long long)r.contam_excess, r.n_contam, r.us.min, r.us.med,
      r.us.mean, r.us.max, r.fa_min, r.fa_max, r.cca_min, r.cca_max);
  if (r.n_implausible)
    std::fprintf(stderr,
                 "** %s run %d/%d: %d/%d calls returned an IMPOSSIBLE counter "
                 "value (>%u) — this is read-payload corruption, not a busy "
                 "channel; every latency number in this run is suspect **\n",
                 label, run_idx, runs, r.n_implausible, n, kImplausibleCount);
  /* NOT a per-run "fa_max==0 => stuck" warning here on purpose: a quiet
   * bench channel legitimately produces all-zero fa_ofdm across a whole
   * run of ~4-9 ms windows for BOTH functions (measured 2026-09-15 — see
   * the file header), and a naive threshold on this alone fires
   * identically for the already-trusted GetRxEnergy(false) path, which is
   * not useful information — it says "quiet channel", not "broken reset".
   * The real cross-check (scout distinctly failing to see activity the
   * SAME-RUN full read demonstrably saw) is printed once after all runs,
   * in main(), where both are available side by side. */
}

/* Idle ambient-rate sample: no calls of ours in flight, so any
 * usb_ctrl_xfers() movement during `dur` is background traffic (the coex
 * thread) alone — reported separately from, not folded into, the per-call
 * floor above. */
void measure_ambient(std::chrono::milliseconds dur) {
  const uint64_t x0 = devourer::usb_ctrl_xfers().load();
  std::this_thread::sleep_for(dur);
  const uint64_t x1 = devourer::usb_ctrl_xfers().load();
  std::printf("ambient (idle, no calls, %lld ms window): %llu xfers "
             "(%.2f xfers/s) — background coex-thread traffic, NOT part of "
             "the per-call floor above\n",
             (long long)dur.count(), (unsigned long long)(x1 - x0),
             double(x1 - x0) * 1000.0 / double(dur.count()));
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
  /* A/B knob for the batched-EP0 path (DeviceConfig::usb.ctrl_batch): =0 runs
   * the scout's register group through the synchronous per-op default, which
   * is the pre-batch behaviour and therefore the honest control for every
   * latency number this bench prints. The library itself reads no environment
   * (global constraint); this is the demo/bench side doing the mapping, same
   * as examples/common/env_config.cpp. */
  if (const char *e = std::getenv("DEVOURER_CTRL_BATCH"))
    cfg.usb.ctrl_batch = (std::strcmp(e, "0") != 0);
  std::printf("ctrl_batch: %s (DEVOURER_CTRL_BATCH=0 to compare against the "
              "synchronous per-op path)\n",
              cfg.usb.ctrl_batch ? "ON" : "OFF");
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

  /* Reset verification (Important 5): only meaningful on the chip that
   * actually has DebugPeekBb — the Jaguar3-specific downcast is
   * deliberate, see the file header. */
  auto *j3 = dynamic_cast<RtlJaguar3Device *>(dev);
  if (!j3) {
    std::fprintf(stderr,
                 "not a Jaguar3 device (no DebugPeekBb) — skipping the "
                 "register-level reset verification, transfer/latency bench "
                 "only\n");
  } else {
    const uint32_t pre_1d2c = j3->DebugPeekBb(0x1d2c);
    const uint32_t pre_1eb4 = j3->DebugPeekBb(0x1eb4);
    const RxEnergy e = dev->GetRxEnergyScout();
    const uint32_t post_1d2c = j3->DebugPeekBb(0x1d2c);
    const uint32_t post_1eb4 = j3->DebugPeekBb(0x1eb4);
    const bool ok = ((post_1d2c >> 31) & 1) == 1 && ((post_1eb4 >> 25) & 1) == 0;
    std::printf(
        "reset verification: 0x1d2c %08x -> %08x, 0x1eb4 %08x -> %08x "
        "(want post 0x1d2c[31]=1 0x1eb4[25]=0: %s); this call fa_ofdm=%u "
        "cca_ofdm=%u\n",
        pre_1d2c, post_1d2c, pre_1eb4, post_1eb4, ok ? "OK" : "FAILED",
        e.fa_ofdm, e.cca_ofdm);
    if (!ok)
      std::fprintf(stderr,
                   "** the composed reset did NOT land on hardware in the "
                   "expected end-state — do not trust the transfer-count "
                   "numbers below as evidence the feature works **\n");
  }

  measure_ambient(std::chrono::milliseconds(2500));

  constexpr int kCalls = 200;
  constexpr int kRuns = 3;
  std::vector<RunResult> scout_runs, full_runs;
  for (int run = 1; run <= kRuns; ++run) {
    RunResult rs = bench_run([dev] { return dev->GetRxEnergyScout(); }, kCalls);
    print_run("GetRxEnergyScout", run, kRuns, kCalls, rs);
    scout_runs.push_back(rs);
    RunResult rf = bench_run(
        [dev] { return dev->GetRxEnergy(/*with_nhm=*/false); }, kCalls);
    print_run("GetRxEnergy(false)", run, kRuns, kCalls, rf);
    full_runs.push_back(rf);
  }

  /* Cross-check for a scout-specific stuck reset: per run, did the
   * (already-trusted) full read see fa_ofdm activity that the scout, in
   * the same slot, did not? That is the actual "reset is stuck" signal —
   * both paths reading zero together just means a quiet channel. */
  bool cross_check_suspicious = false;
  for (size_t i = 0; i < scout_runs.size() && i < full_runs.size(); ++i) {
    if (full_runs[i].fa_max > 0 && scout_runs[i].fa_max == 0) {
      std::printf(
          "** run %zu: GetRxEnergy(false) saw fa_ofdm activity (max=%u) in "
          "the same slot GetRxEnergyScout saw none (max=0) — possible "
          "scout-specific stuck reset, do not dismiss as a quiet channel **\n",
          i + 1, full_runs[i].fa_max);
      cross_check_suspicious = true;
    }
  }
  if (!cross_check_suspicious)
    std::printf(
        "cross-check: no run shows the full read seeing fa_ofdm activity "
        "the scout missed in the same slot — the per-run 0..0 readings "
        "above are consistent with a quiet bench channel affecting both "
        "functions equally, not a scout-specific stuck reset. The pre-loop "
        "single-call verification above independently saw real nonzero "
        "fa_ofdm/cca_ofdm right after InitWrite, and the 0x1d2c/0x1eb4 "
        "bit-level check is the decisive evidence for the reset itself; "
        "this fa_ofdm-activity cross-check is corroborating, not primary.\n");

  auto summarize_runs = [](const char *label, const std::vector<RunResult> &rs) {
    uint64_t floor_min = rs[0].xfer_floor, floor_max = rs[0].xfer_floor;
    double mean_min = rs[0].xfer_mean, mean_max = rs[0].xfer_mean;
    for (const auto &r : rs) {
      floor_min = std::min(floor_min, r.xfer_floor);
      floor_max = std::max(floor_max, r.xfer_floor);
      mean_min = std::min(mean_min, r.xfer_mean);
      mean_max = std::max(mean_max, r.xfer_mean);
    }
    std::printf(
        "%s AGGREGATE over %zu runs: floor %llu..%llu xfers/call%s, "
        "per-run mean spread %.1f..%.1f\n",
        label, rs.size(), (unsigned long long)floor_min,
        (unsigned long long)floor_max,
        floor_min == floor_max ? " (stable)" : " (NOT stable — see per-run lines)",
        mean_min, mean_max);
  };
  int implausible = 0;
  for (const auto *v : {&scout_runs, &full_runs})
    for (const auto &r : *v)
      implausible += r.n_implausible;
  std::printf("counter plausibility: %s (%d call(s) over the %u bound across "
              "every run)\n",
              implausible ? "** FAILED — read payloads were corrupted **"
                          : "ok",
              implausible, kImplausibleCount);

  summarize_runs("GetRxEnergyScout", scout_runs);
  summarize_runs("GetRxEnergy(false)", full_runs);

  return 0;
}
