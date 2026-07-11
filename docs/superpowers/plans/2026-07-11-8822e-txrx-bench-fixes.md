# 8822E TX+RX Bench Fixes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix the four devourer bugs found during mabur's 2026-07-11 bench validation: the 8822E TX+RX 0x41e8 TX corruption (blocker), the J2/J3 HT radiotap LDPC/STBC flag drop, the coex thread's silent death, and the flaky-by-default zerocopy RX.

**Architecture:** Four self-contained commits on branch `fix/8822e-txrx-tx-break` off fork master (ab89f1b + spec commit). Config-gated behavior changes ride `DeviceConfig` (the library reads no environment); demos map new env vars in `examples/common/env_config.cpp`. Host-testable logic gets a headless ctest selftest; register-level behavior is validated on the bench (drone 8822EU at `root@192.168.10.152` + host 8812AU) in dedicated validation tasks.

**Tech Stack:** C++20, CMake, libusb; ARM cross-builds via the mabur repo's `build-arm` tree; bench decode via mabur `tools/bench/*.py`.

**Spec:** `docs/superpowers/specs/2026-07-11-8822e-txrx-bench-fixes-design.md`

## Global Constraints

- Repo root for all devourer tasks: `/home/gilankpam/Projects/drone/devourer` (fork `gilankpam/devourer`, branch `fix/8822e-txrx-tx-break`).
- The library reads no environment variables — new knobs are `DeviceConfig` fields; env spellings live only in `examples/common/env_config.cpp`.
- Host builds/tests need `nix-shell -p pkg-config libusb1 --run "<cmd>"` (NixOS).
- Logger calls use fmt-style placeholders: `_logger->error("x = {}", v)`.
- Working tree starts with ONE uncommitted edit (the temporary `DEVOURER_FORCE_PATHB_REF` getenv hack in `src/jaguar3/RtlJaguar3Device.cpp`) — Task 1 discards it; the config field replaces it.
- Do not touch Jaguar1's radiotap MCS case — it already parses LDPC/STBC correctly (`src/jaguar1/RtlJaguarDevice.cpp:850-899`).
- The drone (`root@192.168.10.152`) currently runs `/tmp/maburd-master` with `DEVOURER_FORCE_PATHB_REF=1`; bench tasks stop/replace it and must leave a working maburd running at the end.

---

### Task 1: Branch setup and temp-hack removal

**Files:**
- Modify: `src/jaguar3/RtlJaguar3Device.cpp` (revert to HEAD)

**Interfaces:**
- Produces: clean branch `fix/8822e-txrx-tx-break` at fork master; a tree where `git status` is clean and `apply_tx_power_current`'s `skip_b` is the stock two-condition form (`_rx_wanted && _variant == C8822E`).

- [ ] **Step 1: Create the branch and drop the temp hack**

```bash
cd /home/gilankpam/Projects/drone/devourer
git checkout -b fix/8822e-txrx-tx-break master
git checkout -- src/jaguar3/RtlJaguar3Device.cpp
git status -s
```

Expected: `git status -s` prints nothing.

- [ ] **Step 2: Verify the host build is green at the baseline**

```bash
nix-shell -p pkg-config libusb1 --run "cmake --build build -j$(nproc)" 2>&1 | tail -2
```

Expected: ends with a `Built target` line, no errors. (No commit — this task only prepares the tree.)

---

### Task 2: Fix 1 — 0x41e8 skip becomes opt-in (`rx.protect_pathb_agc`)

**Files:**
- Modify: `src/DeviceConfig.h` (Rx struct, after `enable_with_tx`, ~line 69)
- Modify: `src/jaguar3/RtlJaguar3Device.cpp` (`apply_tx_power_current`, ~line 877)
- Modify: `src/jaguar3/RtlJaguar3Device.h` (0x41e8 comment blocks, ~lines 92–99 and ~245–247)
- Modify: `src/jaguar3/RadioManagementJaguar3.h` (~lines 90–115 comment)
- Modify: `src/jaguar3/RadioManagementJaguar3.cpp` (~lines 253–257 comment)
- Modify: `examples/common/env_config.cpp` (rx section, next to the `DEVOURER_TX_WITH_RX` mapping ~line 49)
- Modify: `CLAUDE.md` (the `DEVOURER_TX_WITH_RX` knob bullet)

**Interfaces:**
- Produces: `DeviceConfig::Rx::protect_pathb_agc` (`bool`, default `false`); env `DEVOURER_PROTECT_PATHB_AGC` (demos only). Consumed by Task 6 (bench A/B) and by mabur unchanged (defaults are now TX-correct).

- [ ] **Step 1: Add the config field**

In `src/DeviceConfig.h`, inside `struct Rx`, immediately after the `enable_with_tx` field:

```cpp
    /* env: DEVOURER_PROTECT_PATHB_AGC — 8822E legacy RX-desense guard: while
     * RX is wanted, skip every path-B OFDM TXAGC reference write (0x41e8),
     * leaving it at table default. WARNING: the skip corrupts ALL TX in
     * TX+RX mode below the PLCP level (bench-proven 2026-07-11: 0 decodable
     * frames with the skip vs thousands without, same binary). The desense
     * it guarded against ("any nonzero 0x41e8 -> near-deaf RX") did not
     * reproduce at 1 m bench range and is untested at distance. Enable only
     * for RX-desense range experiments. */
    bool protect_pathb_agc = false;
```

- [ ] **Step 2: Gate the skip on the field**

In `src/jaguar3/RtlJaguar3Device.cpp`, `apply_tx_power_current` (~line 877), replace:

```cpp
  const bool skip_b =
      _rx_wanted && _variant == jaguar3::ChipVariant::C8822E;
```

with:

```cpp
  const bool skip_b = _rx_wanted &&
                      _variant == jaguar3::ChipVariant::C8822E &&
                      _cfg.rx.protect_pathb_agc;
```

And update the function's lead comment (currently "The 0x41e8 TX+RX quirk is 8822E-specific, so the skip flag is derived here — once — from _rx_wanted AND the variant; the 8822C keeps its path-B ref writes.") to:

```cpp
/* Re-program TXAGC from the current knob state (see header). The legacy
 * 0x41e8 skip (rx.protect_pathb_agc, default OFF) is 8822E-specific and
 * opt-in: skipping the path-B OFDM ref corrupts all TX in TX+RX mode
 * (bench-proven 2026-07-11), so the default now writes both paths. */
```

- [ ] **Step 3: Add the env mapping**

In `examples/common/env_config.cpp`, directly under the
`cfg.rx.enable_with_tx = env_str("DEVOURER_TX_WITH_RX") != nullptr;` line:

```cpp
  cfg.rx.protect_pathb_agc = env_flag("DEVOURER_PROTECT_PATHB_AGC");
```

- [ ] **Step 4: Rewrite the stale claim sites**

Locate every remaining prose claim with `grep -rn "41e8" src CLAUDE.md docs`. Rewrite these four sites (leave the `RadioManagementJaguar3.cpp:641` log string — it accurately describes whichever mode runs):

a. `src/jaguar3/RtlJaguar3Device.h` (~92–99), the paragraph ending "...every ref write takes skip_path_b_ofdm_ref from _rx_wanted." — replace that sentence with:

```
 * The legacy 8822E TX+RX 0x41e8 skip is opt-in via rx.protect_pathb_agc
 * (default OFF): every ref write derives skip_path_b_ofdm_ref from
 * _rx_wanted AND that flag. Skipping corrupts all TX in TX+RX mode
 * (bench-proven 2026-07-11); the desense the skip guarded against did not
 * reproduce at bench range and is untested at distance.
```

b. `src/jaguar3/RtlJaguar3Device.h` (~245–247), the `_rx_wanted` field comment sentence "...no churn can ever touch 0x41e8 while RX is alive (the 8822E RX-desense ...)" — append to it: `Only enforced when rx.protect_pathb_agc opts in (see DeviceConfig).`

c. `src/jaguar3/RadioManagementJaguar3.h` (~90–115), the `skip_path_b_ofdm_ref` parameter doc — append one sentence: `Callers pass true only under the opt-in rx.protect_pathb_agc guard; the default path writes 0x41e8 like the 8822C, because skipping it corrupts TX in TX+RX mode (2026-07-11 bench).`

d. `CLAUDE.md`, in the `DEVOURER_TX_WITH_RX` bullet, replace the sentence "On the 8822E, TX+RX mode leaves the path-B OFDM TXAGC reference (0x41e8) at table default — any nonzero value there desenses the EU's RX to near-deaf (hardware-bisected, value-independent)." with:

```
On the 8822E, TX+RX mode used to leave the path-B OFDM TXAGC reference
(0x41e8) at table default to guard a claimed RX desense — that skip
corrupts ALL TX in TX+RX mode (bench-proven 2026-07-11: zero decodable
frames), so the default now writes both paths; the legacy skip is opt-in
via DEVOURER_PROTECT_PATHB_AGC for RX-desense range experiments (the
desense did not reproduce at 1 m).
```

- [ ] **Step 5: Build**

```bash
cd /home/gilankpam/Projects/drone/devourer
nix-shell -p pkg-config libusb1 --run "cmake --build build -j$(nproc)" 2>&1 | tail -2
```

Expected: `Built target` lines, no errors.

- [ ] **Step 6: Commit**

```bash
git add -A
git commit -m "jaguar3: 8822E path-B TXAGC ref (0x41e8) written by default; legacy skip opt-in

The unconditional skip corrupted every frame sent in TX+RX mode below the
PLCP level (bench 2026-07-11: 0 decodable frames stock vs 5833/15s with the
ref written, single-binary env A/B, drone 8822EU -> host 8812AU ch149 MCS0).
The RX desense the skip guarded against did not reproduce at bench range;
rx.protect_pathb_agc / DEVOURER_PROTECT_PATHB_AGC restores it for range
experiments."
```

---

### Task 3: Fix 2 — HT radiotap LDPC/STBC decode for J2/J3 (+ selftest)

**Files:**
- Create: `src/RadiotapTxFlags.h`
- Create: `tests/radiotap_txflags_selftest.cpp`
- Modify: `CMakeLists.txt` (selftest registration, next to the `ToneMaskSelftest` block ~line 450)
- Modify: `src/jaguar3/RtlJaguar3Device.cpp` (MCS case, ~line 1365)
- Modify: `src/jaguar2/RtlJaguar2Device.cpp` (MCS case, ~line 1196)

**Interfaces:**
- Produces: `devourer::RadiotapMcsField` and
  `devourer::decode_radiotap_mcs_field(const uint8_t* arg)` in
  `src/RadiotapTxFlags.h` — `arg` points at the 3-byte radiotap MCS field
  (known, flags, index). Consumed by both device files and the selftest.

- [ ] **Step 1: Write the failing selftest**

Create `tests/radiotap_txflags_selftest.cpp`:

```cpp
/* Headless wire-contract guard for the radiotap TX-flag path: what
 * build_stream_radiotap() emits must decode back to the TxMode that went in.
 * Guards the J2/J3 regression where HT LDPC/STBC known/flag bits were
 * silently ignored (only the VHT case honoured them). No device is opened. */
#include <cstdio>
#include <cstring>
#include <vector>

#include "RadiotapBuilder.h"
#include "RadiotapTxFlags.h"
#include "ieee80211_radiotap.h"

static int failures = 0;

#define CHECK_EQ(expr, want)                                                   \
  do {                                                                         \
    long got_ = long(expr);                                                    \
    if (got_ != long(want)) {                                                  \
      std::fprintf(stderr, "FAIL %s:%d: %s = %ld, want %ld\n", __FILE__,       \
                   __LINE__, #expr, got_, long(want));                         \
      failures++;                                                              \
    }                                                                          \
  } while (0)

/* Walk a built radiotap header and return the decoded MCS field. */
static devourer::RadiotapMcsField decode_ht(const std::vector<uint8_t> &rt) {
  struct ieee80211_radiotap_iterator it;
  auto *hdr = reinterpret_cast<struct ieee80211_radiotap_header *>(
      const_cast<uint8_t *>(rt.data()));
  devourer::RadiotapMcsField f;
  if (ieee80211_radiotap_iterator_init(&it, hdr, rt.size(), nullptr) != 0) {
    std::fprintf(stderr, "FAIL: iterator init\n");
    failures++;
    return f;
  }
  while (ieee80211_radiotap_iterator_next(&it) == 0) {
    if (it.this_arg_index == IEEE80211_RADIOTAP_MCS)
      f = devourer::decode_radiotap_mcs_field(it.this_arg);
  }
  return f;
}

int main() {
  /* HT MCS0/20, LDPC+STBC (mabur's MAX_RANGE control mode). */
  devourer::TxMode m;
  m.mode = devourer::TxMode::Mode::HT;
  m.ht_mcs = 0;
  m.bw_mhz = 20;
  m.ldpc = true;
  m.stbc = true;
  auto f = decode_ht(devourer::build_stream_radiotap(m));
  CHECK_EQ(f.have_mcs, 1);
  CHECK_EQ(f.mcs, 0);
  CHECK_EQ(f.bw40, 0);
  CHECK_EQ(f.sgi, 0);
  CHECK_EQ(f.ldpc, 1);
  CHECK_EQ(f.stbc, 1);

  /* HT MCS7/40/SGI, no LDPC/STBC. */
  m = devourer::TxMode{};
  m.mode = devourer::TxMode::Mode::HT;
  m.ht_mcs = 7;
  m.bw_mhz = 40;
  m.sgi = true;
  f = decode_ht(devourer::build_stream_radiotap(m));
  CHECK_EQ(f.have_mcs, 1);
  CHECK_EQ(f.mcs, 7);
  CHECK_EQ(f.bw40, 1);
  CHECK_EQ(f.sgi, 1);
  CHECK_EQ(f.ldpc, 0);
  CHECK_EQ(f.stbc, 0);

  /* VHT wire contract: pin the byte positions the device parsers read
   * (known bit0 = STBC, arg[2] bit0 = STBC flag, arg[8] bit0 = LDPC). */
  m = devourer::TxMode{};
  m.mode = devourer::TxMode::Mode::VHT;
  m.vht_mcs = 3;
  m.vht_nss = 1;
  m.bw_mhz = 80;
  m.ldpc = true;
  m.stbc = true;
  auto rt = devourer::build_stream_radiotap(m);
  /* 22-byte VHT radiotap: header(8) + TX_FLAGS(2) + VHT info(12) at off 10. */
  CHECK_EQ(rt.size(), 22);
  CHECK_EQ(rt[10] & 0x01, 0x01); /* known: STBC        */
  CHECK_EQ(rt[12] & 0x01, 0x01); /* flags: STBC        */
  CHECK_EQ(rt[18] & 0x01, 0x01); /* coding[u0]: LDPC   */
  CHECK_EQ(rt[13], 4);           /* bw code: 80 MHz    */

  if (failures) {
    std::fprintf(stderr, "%d failure(s)\n", failures);
    return 1;
  }
  std::printf("radiotap_txflags: OK\n");
  return 0;
}
```

Register it in `CMakeLists.txt`, immediately after the `ToneMaskSelftest` block:

```cmake
add_executable(RadiotapTxFlagsSelftest tests/radiotap_txflags_selftest.cpp)
target_compile_features(RadiotapTxFlagsSelftest PRIVATE cxx_std_20)
target_link_libraries(RadiotapTxFlagsSelftest PRIVATE devourer)
add_test(NAME radiotap_txflags COMMAND RadiotapTxFlagsSelftest)
```

- [ ] **Step 2: Verify it fails to build (header doesn't exist yet)**

```bash
cd /home/gilankpam/Projects/drone/devourer
nix-shell -p pkg-config libusb1 --run "cmake --build build -j$(nproc) --target RadiotapTxFlagsSelftest" 2>&1 | grep -E "error|Error" | head -3
```

Expected: `fatal error: RadiotapTxFlags.h: No such file or directory`.

- [ ] **Step 3: Implement the decoder header**

Create `src/RadiotapTxFlags.h`:

```cpp
#pragma once

/* Decoded view of the 3-byte radiotap MCS field (known, flags, index) on the
 * TX/inject path — one shared reading for the per-generation send_packet
 * parsers, so the HT LDPC/STBC known/flag bits can't silently diverge again
 * (J2/J3 historically ignored them; only the VHT case honoured them; J1
 * carried its own correct copy). Bit positions per the radiotap MCS spec and
 * RadiotapBuilder::build_ht. */

#include <cstdint>

#include "ieee80211_radiotap.h"

namespace devourer {

struct RadiotapMcsField {
  bool have_mcs = false;
  uint8_t mcs = 0;   /* HT MCS index 0..31, valid when have_mcs */
  bool bw40 = false; /* MCS BW subfield == 40 MHz */
  uint8_t sgi = 0;
  uint8_t ldpc = 0;
  uint8_t stbc = 0;  /* STBC stream count 0..3 */
};

inline RadiotapMcsField decode_radiotap_mcs_field(const uint8_t *arg) {
  RadiotapMcsField f;
  const uint8_t known = arg[0];
  const uint8_t flags = arg[1];
  if ((flags & IEEE80211_RADIOTAP_MCS_BW_MASK) == IEEE80211_RADIOTAP_MCS_BW_40)
    f.bw40 = true;
  f.sgi = (flags & 0x04) ? 1 : 0;
  if ((known & 0x10) && (flags & 0x10)) /* FEC-type known + LDPC flag */
    f.ldpc = 1;
  if (known & 0x20) /* STBC known: flags bits [6:5] = stream count */
    f.stbc = (flags >> 5) & 0x3;
  if ((known & IEEE80211_RADIOTAP_MCS_HAVE_MCS) && arg[2] <= 31) {
    f.have_mcs = true;
    f.mcs = arg[2];
  }
  return f;
}

}  // namespace devourer
```

- [ ] **Step 4: Run the selftest — expect PASS**

```bash
nix-shell -p pkg-config libusb1 --run "cmake --build build -j$(nproc) --target RadiotapTxFlagsSelftest && ctest --test-dir build -R radiotap_txflags --output-on-failure"
```

Expected: `radiotap_txflags: OK`, `100% tests passed`.

- [ ] **Step 5: Wire the decoder into Jaguar3**

In `src/jaguar3/RtlJaguar3Device.cpp`: add `#include "RadiotapTxFlags.h"` next to the existing `#include "RadiotapPeek.h"`, then replace the whole `case IEEE80211_RADIOTAP_MCS: { ... } break;` block (~lines 1365–1379, the one reading `mcs_known`/`mcs_flags` and setting only BW/SGI/rate) with:

```cpp
    case IEEE80211_RADIOTAP_MCS: {
      const devourer::RadiotapMcsField m =
          devourer::decode_radiotap_mcs_field(it.this_arg);
      if (m.bw40)
        bwidth = CHANNEL_WIDTH_40;
      sgi = m.sgi;
      ldpc = m.ldpc;
      stbc = m.stbc;
      if (m.have_mcs) {
        fixed_rate = MGN_MCS0 + m.mcs;
        rate_from_radiotap = true;
      }
    } break;
```

- [ ] **Step 6: Wire the decoder into Jaguar2**

In `src/jaguar2/RtlJaguar2Device.cpp`: add `#include "RadiotapTxFlags.h"` with the other src includes, then replace its `case IEEE80211_RADIOTAP_MCS: { ... } break;` (~lines 1196–1209) with the identical block from Step 5.

- [ ] **Step 7: Full host build + test suite**

```bash
nix-shell -p pkg-config libusb1 --run "cmake --build build -j$(nproc) && ctest --test-dir build --output-on-failure" 2>&1 | tail -4
```

Expected: `100% tests passed`.

- [ ] **Step 8: Commit**

```bash
git add src/RadiotapTxFlags.h tests/radiotap_txflags_selftest.cpp CMakeLists.txt \
        src/jaguar3/RtlJaguar3Device.cpp src/jaguar2/RtlJaguar2Device.cpp
git commit -m "radiotap TX: honour HT LDPC/STBC known bits on Jaguar2/3 (+ selftest)

J2/J3 send_packet ignored the MCS field's FEC/STBC known bits, so HT frames
tagged LDPC/STBC (e.g. mabur's MAX_RANGE robustness flags) silently flew as
plain BCC SISO; only the VHT case honoured them. J1 already carried the fix.
Shared decode in RadiotapTxFlags.h + headless builder/parser contract test."
```

---

### Task 4: Fix 3 — coex runtime survives transient tick failures

**Files:**
- Modify: `src/jaguar3/RtlJaguar3Device.cpp` (`coex_runtime_loop`, ~lines 318–376)

**Interfaces:**
- Produces: no API change. Behavior: a coex tick exception logs `[E]` and retries; 5 consecutive failures exit the loop with a final `[E]`.

- [ ] **Step 1: Replace the catch-and-die**

In `coex_runtime_loop`, add a failure counter next to the existing locals
(`uint64_t tick = 0, c2h = 0, rx = 0;`):

```cpp
  uint64_t tick = 0, c2h = 0, rx = 0;
  int fail_streak = 0;
  constexpr int kMaxFailStreak = 5;
```

Then replace:

```cpp
    try {
      std::lock_guard<std::mutex> lk(_reg_mu);
      _hal.coex_run_5g();
      _hal.pwr_track(); /* thermal TX-power compensation (sustains upper 5 GHz) */
      _hal.fw_update_wl_phy_info();
      _hal.fw_set_pwr_mode_active();
      _hal.fw_coex_query_bt_info();
    } catch (...) { break; }
```

with:

```cpp
    /* A transient register/USB error must not kill the thread that sustains
     * 5 GHz TX (coex re-apply + pwr_track) — log, retry next tick, and only
     * give up after kMaxFailStreak consecutive failures (a chip that is
     * really gone fails every tick and still lets the thread exit). */
    try {
      std::lock_guard<std::mutex> lk(_reg_mu);
      _hal.coex_run_5g();
      _hal.pwr_track(); /* thermal TX-power compensation (sustains upper 5 GHz) */
      _hal.fw_update_wl_phy_info();
      _hal.fw_set_pwr_mode_active();
      _hal.fw_coex_query_bt_info();
      fail_streak = 0;
    } catch (const std::exception &e) {
      ++fail_streak;
      _logger->error("Jaguar3 coex: tick {} failed ({}) — {}/{} consecutive",
                     tick + 1, e.what(), fail_streak, kMaxFailStreak);
      if (fail_streak >= kMaxFailStreak) {
        _logger->error("Jaguar3 coex: giving up after {} consecutive failures "
                       "— sustained 5 GHz TX will degrade", kMaxFailStreak);
        break;
      }
      continue;
    } catch (...) {
      ++fail_streak;
      _logger->error("Jaguar3 coex: tick {} failed (unknown exception) — {}/{} "
                     "consecutive", tick + 1, fail_streak, kMaxFailStreak);
      if (fail_streak >= kMaxFailStreak) {
        _logger->error("Jaguar3 coex: giving up after {} consecutive failures "
                       "— sustained 5 GHz TX will degrade", kMaxFailStreak);
        break;
      }
      continue;
    }
```

(The `continue` skips the success-path tick log; `next_tick` was already advanced, so the retry lands on the next 2 s boundary.)

- [ ] **Step 2: Build**

```bash
nix-shell -p pkg-config libusb1 --run "cmake --build build -j$(nproc)" 2>&1 | tail -2
```

Expected: `Built target` lines, no errors.

- [ ] **Step 3: Commit**

```bash
git add src/jaguar3/RtlJaguar3Device.cpp
git commit -m "jaguar3: coex runtime retries transient tick failures instead of dying silently

catch(...){break} meant one USB glitch permanently killed the thread that
sustains 5 GHz TX (coex re-apply + pwr_track) with no log line. Now: [E] log
with the exception message, retry next tick, give up loudly only after 5
consecutive failures."
```

---

### Task 5: Fix 4 — zerocopy RX default off (+ time-boxed look)

**Files:**
- Modify: `src/DeviceConfig.h` (`Usb::rx_zerocopy`, ~line 282)
- Modify: `examples/common/env_config.cpp` (comment on the `DEVOURER_RX_ZEROCOPY` line, ~line 174)
- Modify: `docs/performance-tuning.md` (zerocopy section caveat)
- Read-only: `src/UsbTransport.cpp` (~30 min inspection; findings go in the commit message)

**Interfaces:**
- Produces: `rx_zerocopy` defaults `false`; `DEVOURER_RX_ZEROCOPY=1` opts in (mapping line already handles both directions — only its comment changes).

- [ ] **Step 1: Flip the default**

In `src/DeviceConfig.h`, change the `rx_zerocopy` field to:

```cpp
    /* env: DEVOURER_RX_ZEROCOPY — allocate the async RX ring from kernel DMA
     * memory (libusb_dev_mem_alloc / USBDEVFS_ALLOC) so incoming frames DMA
     * straight into the userspace buffer, eliminating the per-URB usbfs copy
     * on reap. Linux + capable HCD only. DEFAULT OFF: intermittent
     * zero-frame delivery observed on an xhci desktop host (2026-07-11 —
     * same host/dongle/channel worked on one run, deaf on the next; the heap
     * path was 100% reliable). Opt in with =1 until root-caused. */
    bool rx_zerocopy = false;
```

In `examples/common/env_config.cpp`, update the comment on the mapping line to:

```cpp
  if (env_str("DEVOURER_RX_ZEROCOPY")) /* default false; =1 opts in to DMA */
    cfg.usb.rx_zerocopy = env_flag("DEVOURER_RX_ZEROCOPY");
```

In `docs/performance-tuning.md`, find the zerocopy section (`grep -n zerocopy docs/performance-tuning.md`) and add after its intro paragraph:

```markdown
> **Known issue (2026-07-11):** on an xhci desktop host the zerocopy ring
> intermittently delivered zero frames (same host/dongle/channel: one run
> fine, the next deaf; heap path 100% reliable). Zerocopy is therefore
> default-OFF (`DEVOURER_RX_ZEROCOPY=1` opts in) until root-caused.
```

- [ ] **Step 2: Time-boxed (~30 min) inspection of the zerocopy path**

Read `src/UsbTransport.cpp` around the `_rx_zerocopy` uses (alloc, URB submit, reap, fallback). Questions to answer (notes only, no speculative fixes):
- Is `libusb_dev_mem_alloc` failure the ONLY fallback trigger, or is there a success-but-no-delivery mode with no detection?
- Are the DMA buffers re-submitted with the same pointers after reap (stale mapping risk on re-enumeration/reset)?
- Anything obviously racy between the reset in `claim_interface_then_reset` and buffers allocated on the pre-reset device handle?

Write 3–6 sentences of findings for the commit message (even "nothing obvious; needs instrumented repro" is a valid finding).

- [ ] **Step 3: Build + verify default via rxdemo log line**

```bash
nix-shell -p pkg-config libusb1 --run "cmake --build build -j$(nproc) --target rxdemo" 2>&1 | tail -1
```

Expected: `Built target rxdemo`. (The "N zerocopy DMA / N heap" split is printed at RX-loop start; the on-hardware check happens in Task 7.)

- [ ] **Step 4: Commit**

```bash
git add src/DeviceConfig.h examples/common/env_config.cpp docs/performance-tuning.md
git commit -m "usb: zerocopy RX ring default off (intermittent zero-delivery on xhci host)

<paste Step 2 findings here>

Observed 2026-07-11 during mabur bench validation: stock rxdemo deaf on one
run, fine on the next, heap path reliable throughout. Opt back in with
DEVOURER_RX_ZEROCOPY=1."
```

---

### Task 6: Bench validation — fix 1 A/B on real hardware

**Files:**
- No source changes. Uses: mabur repo ARM build tree (`/home/gilankpam/Projects/drone/mabur/build-arm`), drone `root@192.168.10.152`, host 8812AU.

**Interfaces:**
- Consumes: branch binaries (txdemo). Pass/fail gates the merge.

- [ ] **Step 1: Cross-build branch txdemo for ARM**

```bash
cd /home/gilankpam/Projects/drone/mabur
TT=armv7l-unknown-linux-musleabihf; CCB="$(readlink -f toolchain/cc)/bin"
export MABUR_CROSS_CC="$CCB/$TT-gcc" MABUR_CROSS_CXX="$CCB/$TT-g++" \
  PATH="$CCB:$(readlink -f toolchain/pkg-config)/bin:$PATH" \
  MABUR_STAGING="$PWD/toolchain/staging" \
  PKG_CONFIG_LIBDIR="$(readlink -f toolchain/libusb-out-dev)/lib/pkgconfig" \
  PKG_CONFIG_SYSROOT_DIR="" \
  CPATH="$(readlink -f toolchain/libusb-out-dev)/include"
unset PKG_CONFIG_PATH
cmake --build build-arm --target txdemo -j$(nproc) 2>&1 | tail -1
```

Expected: `Built target txdemo`. (The build-arm tree compiles the SIBLING `../devourer` checkout — the branch.)

- [ ] **Step 2: Deploy and run stock TX+RX (no env) on the drone**

```bash
scp -O build-arm/devourer/txdemo root@192.168.10.152:/tmp/txdemo_fix
ssh root@192.168.10.152 'killall maburd-master txdemo_fix 2>/dev/null; sleep 10; \
  DEVOURER_CHANNEL=149 DEVOURER_TX_RATE=MCS0 DEVOURER_TX_WITH_RX=thread \
  DEVOURER_LOG_LEVEL=warn nohup timeout 60 /tmp/txdemo_fix >/tmp/txd_fix_a.log 2>&1 & \
  sleep 18; echo running'
```

- [ ] **Step 3: Capture on the host — expect TX now works with NO knob**

```bash
cd /home/gilankpam/Projects/drone/devourer
DEVOURER_PID=0x8812 DEVOURER_CHANNEL=149 DEVOURER_EVENTS=stdout \
DEVOURER_LOG_LEVEL=warn DEVOURER_STREAM_OUT=1 \
timeout 15 ./build/rxdemo > /tmp/fixA.jsonl 2>/dev/null
grep -c '"ev":"rx.frame"' /tmp/fixA.jsonl
```

Expected: **several thousand** (baseline before fix: 0). Frames are
`"rate":12` (HT MCS0), `"crc":0`. Note: no `DEVOURER_RX_ZEROCOPY` needed —
Task 5 already made heap the default.

- [ ] **Step 4: Legacy-knob A/B — expect the old broken behavior returns**

```bash
ssh root@192.168.10.152 'killall txdemo_fix 2>/dev/null; sleep 12; \
  DEVOURER_CHANNEL=149 DEVOURER_TX_RATE=MCS0 DEVOURER_TX_WITH_RX=thread \
  DEVOURER_PROTECT_PATHB_AGC=1 DEVOURER_LOG_LEVEL=warn \
  nohup timeout 60 /tmp/txdemo_fix >/tmp/txd_fix_b.log 2>&1 & sleep 18; echo running'
cd /home/gilankpam/Projects/drone/devourer
DEVOURER_PID=0x8812 DEVOURER_CHANNEL=149 DEVOURER_EVENTS=stdout \
DEVOURER_LOG_LEVEL=warn DEVOURER_STREAM_OUT=1 \
timeout 15 ./build/rxdemo > /tmp/fixB.jsonl 2>/dev/null
grep -c '"ev":"rx.frame"' /tmp/fixB.jsonl
ssh root@192.168.10.152 'killall txdemo_fix 2>/dev/null'
```

Expected: **0** rx.frame (knob faithfully restores the legacy skip).

- [ ] **Step 5: Record results**

Append both counts to the branch commit via an empty commit or note them for the Task 8 doc update (no source change here). If Step 3 < 1000 frames or Step 4 > 0 frames: STOP, do not merge, re-investigate.

---

### Task 7: Bench validation — fixes 2–4 on real hardware

**Files:**
- No source changes. Same bench setup as Task 6.

- [ ] **Step 1: LDPC/STBC on air**

```bash
ssh root@192.168.10.152 'sleep 2; DEVOURER_CHANNEL=149 \
  DEVOURER_TX_RATE=MCS0/LDPC/STBC DEVOURER_TX_WITH_RX=thread \
  DEVOURER_LOG_LEVEL=warn nohup timeout 60 /tmp/txdemo_fix >/tmp/txd_fix_c.log 2>&1 & \
  sleep 18; echo running'
cd /home/gilankpam/Projects/drone/devourer
DEVOURER_PID=0x8812 DEVOURER_CHANNEL=149 DEVOURER_EVENTS=stdout \
DEVOURER_LOG_LEVEL=warn DEVOURER_STREAM_OUT=1 \
timeout 15 ./build/rxdemo > /tmp/fixC.jsonl 2>/dev/null
python3 -c "
import json
from collections import Counter
fr=[json.loads(l) for l in open('/tmp/fixC.jsonl') if '\"rx.frame\"' in l]
print(len(fr), Counter((f['ldpc'],f['stbc'],f['crc']) for f in fr).most_common(3))"
ssh root@192.168.10.152 'killall txdemo_fix 2>/dev/null'
```

Expected: thousands of frames, dominant tuple `(1, 1, 0)` — ldpc:1 stbc:1
crc-clean (baseline: ldpc/stbc were 0/0). If LDPC/STBC frames decode
noticeably worse than plain MCS0 (compare counts with Task 6 Step 3),
record that as a finding — it informs whether mabur keeps its `flags.*`
defaults.

- [ ] **Step 2: Coex soak (fix 3 regression check)**

```bash
ssh root@192.168.10.152 'DEVOURER_CHANNEL=149 DEVOURER_TX_RATE=MCS0 \
  DEVOURER_TX_WITH_RX=thread DEVOURER_LOG_LEVEL=info \
  nohup timeout 70 /tmp/txdemo_fix >/tmp/txd_soak.log 2>&1 & sleep 65; \
  grep -c "coex: tick" /tmp/txd_soak.log; grep -c "giving up" /tmp/txd_soak.log; \
  tail -2 /tmp/txd_soak.log'
```

Expected: tick count ≥ 3 (ticks 1–3 always log, then every 15th), `giving
up` count = 0, clean `tx.stats` tail — the loop still runs and shuts down
cleanly.

- [ ] **Step 3: Zerocopy default check (fix 4)**

```bash
cd /home/gilankpam/Projects/drone/devourer
DEVOURER_PID=0x8812 DEVOURER_CHANNEL=149 DEVOURER_LOG_LEVEL=info \
DEVOURER_EVENTS=off timeout 8 ./build/rxdemo 2>&1 | grep "async queue"
DEVOURER_RX_ZEROCOPY=1 DEVOURER_PID=0x8812 DEVOURER_CHANNEL=149 \
DEVOURER_LOG_LEVEL=info DEVOURER_EVENTS=off timeout 8 ./build/rxdemo 2>&1 | grep "async queue"
```

Expected: first run reports `(0 zerocopy DMA, 8 heap)`; second reports the
DMA split (e.g. `(8 zerocopy DMA, 0 heap)`).

---

### Task 8: Merge, mabur pin bump, redeploy, docs

**Files:**
- Modify (mabur repo): `third_party/devourer` submodule pin, `docs/bench-validation.md`
- Drone: replace `/tmp/maburd-master`, env-free launch

- [ ] **Step 1: Merge and push the fork**

```bash
cd /home/gilankpam/Projects/drone/devourer
git checkout master && git merge --no-ff fix/8822e-txrx-tx-break \
  -m "Merge fix/8822e-txrx-tx-break: 0x41e8 TX fix, HT flag decode, coex retry, zerocopy default off"
git push origin master
```

- [ ] **Step 2: Bump mabur's submodule pin**

```bash
cd /home/gilankpam/Projects/drone/mabur
git -C third_party/devourer fetch origin && git -C third_party/devourer checkout origin/master
git add third_party/devourer
git commit -m "chore: bump devourer to 8822E TX+RX fixes (0x41e8, HT flags, coex, zerocopy)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

- [ ] **Step 3: Rebuild + redeploy maburd, env-free launch**

```bash
cd /home/gilankpam/Projects/drone/mabur
# same cross env exports as Task 6 Step 1, then:
cmake --build build-arm --target maburd -j$(nproc) 2>&1 | tail -1
scp -O build-arm/drone/maburd root@192.168.10.152:/tmp/maburd-master
ssh root@192.168.10.152 'chmod +x /tmp/maburd-master; \
  nohup /tmp/maburd-master -c /etc/mabur.json >/tmp/mabur-master.log 2>&1 & \
  sleep 25; tail -1 /tmp/mabur-master.log'
```

Expected: a `stats:` line with `tx_failed=0` and `sent` climbing. NOTE: no
`DEVOURER_FORCE_PATHB_REF` / `DEVOURER_PROTECT_PATHB_AGC` in the launch.

- [ ] **Step 4: E2E confirmation capture**

```bash
cd /home/gilankpam/Projects/drone/devourer
DEVOURER_PID=0x8812 DEVOURER_CHANNEL=149 DEVOURER_EVENTS=stdout \
DEVOURER_LOG_LEVEL=warn DEVOURER_STREAM_OUT=1 \
timeout 15 ./build/rxdemo > /tmp/final.jsonl 2>/dev/null
python3 /home/gilankpam/Projects/drone/mabur/tools/bench/live_decode.py /tmp/final.jsonl
```

Expected: ≥ 10k frames, stream 0 loss 0.000%, recovered RTP packets > 1000.
maburd's CRIT/T0 frames should now show `ldpc:1` (fix 2 live).

- [ ] **Step 5: Update mabur bench doc + commit**

In `docs/bench-validation.md`: mark bugs 4–7 as FIXED with the fork merge
commit hash; rewrite the "Bench state" bullets (drone runs env-free
master-built maburd; `../devourer` has no local edits; captures no longer
need `DEVOURER_RX_ZEROCOPY=0` since the default flipped); keep the
RX-desense-at-range item open, now referencing `DEVOURER_PROTECT_PATHB_AGC`
as the A/B lever for the E-series.

```bash
cd /home/gilankpam/Projects/drone/mabur
git add docs/bench-validation.md
git commit -m "docs(bench): devourer bugs 4-7 fixed and re-validated on air

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

## Self-Review

- **Spec coverage:** fix 1 → Task 2 (+6), fix 2 → Task 3 (+7.1), fix 3 → Task 4 (+7.2), fix 4 → Task 5 (+7.3), validation §1–4 → Tasks 6–7, wrap-up §5 → Task 8. J1 audit resolved during planning: J1 already correct (Global Constraints note). ✓
- **Placeholders:** the one intentional fill-in is Task 5 Step 4's `<paste Step 2 findings here>` — produced by its own Step 2, not external knowledge. ✓
- **Type consistency:** `RadiotapMcsField`/`decode_radiotap_mcs_field` identical in Task 3 Steps 1/3/5/6; `protect_pathb_agc` spelling identical in Task 2 and Task 6. ✓
