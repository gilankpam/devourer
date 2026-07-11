# 8822E TX+RX bench fixes — design

**Date:** 2026-07-11 · **Repo:** gilankpam/devourer (fork; fork-only, no
upstream PRs planned) · **Base:** `master` @ ab89f1b

Fixes the four bugs found during mabur's 2026-07-11 bench validation
(evidence and full narrative: mabur repo `docs/bench-validation.md`, bugs
4–7). One branch, `fix/8822e-txrx-tx-break`, four self-contained commits,
merged to fork master after on-air validation.

## Background

mabur (drone-side FPV streamer) drives an RTL8812EU/8822E (Jaguar3) in
devourer's TX+RX single-handle mode (`rx.enable_with_tx` + `StartRxLoop`).
Bench validation found that **no frame maburd ever transmitted was decodable
on air**, while all USB-level TX counters looked healthy. Root cause was
bisected (single binary, env A/B) to the 8822E TX+RX quirk that skips the
path-B OFDM TXAGC reference write. Three more defects were found en route.

## Fix 1 — 0x41e8 skip corrupts all TX in TX+RX mode (blocker)

**Defect.** `RtlJaguar3Device::apply_tx_power_current` derives
`skip_path_b_ofdm_ref` from `_rx_wanted && variant == C8822E`, leaving BB
register `0x41e8[16:10]` (path-B OFDM TXAGC reference) at its table default
whenever RX is wanted. The chip transmits OFDM through both chains, so the
misdriven path-B chain corrupts the combined waveform below the PLCP level:
receivers log CCA/false-alarm hash, zero decodable frames. Measured
2026-07-11 (drone 8822EU → host 8812AU, ch149, HT MCS0, same binary):
stock = 0 frames / 15 s; ref written = 5,833 clean frames / 15 s. The
skip's original rationale — "any nonzero 0x41e8 desenses 8822E RX to
near-deaf (hardware-bisected, value-independent)" — did NOT reproduce at
1 m bench range (RX heard 4,165/5,000 of a flood with the ref written vs
3,872/5,000 stock). Long-range desense remains untested.

**Change.**

- `DeviceConfig::Rx` gains `bool protect_pathb_agc = false;`. Doc tag:
  legacy 8822E RX-desense guard; enabling it restores the skip, which
  corrupts all TX in TX+RX mode (bench-proven); for RX-desense experiments
  only. Env spelling `DEVOURER_PROTECT_PATHB_AGC` mapped in
  `examples/common/env_config.cpp`.
- `apply_tx_power_current`:
  `skip_b = _rx_wanted && variant == C8822E && _cfg.rx.protect_pathb_agc;`
- Delete the session's temporary `DEVOURER_FORCE_PATHB_REF` getenv hack
  (uncommitted working-tree edit) — this field replaces it, keeping the
  library env-free per repo convention.
- Rewrite the stale claims in: `RtlJaguar3Device.h` (quirk comment block,
  `_rx_wanted` field comment), `RadioManagementJaguar3.{h,cpp}` comments,
  and CLAUDE.md's TX+RX paragraph. Each must state both measured facts:
  skip ⇒ TX corruption (proven 2026-07-11); write ⇒ RX desense (claimed
  earlier, not reproduced at 1 m, untested at range).
- Known behavior change beyond TX+RX mode: the RX-only `Init()` path also
  sets `_rx_wanted`, so an RX-only app that explicitly calls a TX-power
  setter would now write 0x41e8. Pure-RX apps do not call TXAGC applies, so
  nothing changes in practice; documented, not special-cased.

## Fix 2 — HT radiotap LDPC/STBC flags silently dropped

**Defect.** `send_packet`'s `IEEE80211_RADIOTAP_MCS` case (Jaguar3:
`RtlJaguar3Device.cpp` ~1365) parses BW/SGI/MCS but ignores the FEC and
STBC known/flag bits; only the VHT case reads them. Devourer's own
`build_ht()` emits them (known 0x10/0x20, flags 0x10/0x20), so HT callers
(mabur's MAX_RANGE LDPC/STBC robustness flags) silently fly as plain HT.

**Change.** In the MCS case:

```cpp
if (mcs_known & 0x10) ldpc = (mcs_flags >> 4) & 1;   /* FEC known */
if (mcs_known & 0x20) stbc = (mcs_flags >> 5) & 0x3; /* STBC streams */
```

The existing `GetTxCaps().stbc_ok` guard stays the clamp. Audit Jaguar1 and
Jaguar2 `send_packet` MCS cases for the same omission and apply the same
two lines where present. Add a headless ctest selftest
(`RadiotapTxFlagsSelftest`, repo selftest style): `build_ht`/`build_vht`
outputs fed through the parse logic must yield the ldpc/stbc/sgi/bw that
went in.

## Fix 3 — coex runtime thread dies silently and permanently

**Defect.** `coex_runtime_loop`'s 2 s tick body is
`try { … } catch (...) { break; }`: one transient register/USB error kills
the thread that sustains 5 GHz TX (`coex_run_5g` + `pwr_track`), with no
log line. Compiled-WARN production binaries get zero indication.

**Change.** Catch `std::exception` (log its message) and `catch (...)`
(log "unknown exception") so nothing escapes the thread, log `[E]` with the
tick number, increment a consecutive-failure counter, and continue; a
successful tick resets the counter; after 5 consecutive failures log
`[E] coex runtime giving up after 5 consecutive failures — sustained 5 GHz
TX will degrade` and exit the loop. No API change.

## Fix 4 — #253 zerocopy RX intermittently deaf

**Defect.** The new default-on zerocopy DMA RX ring delivered zero frames
on one rxdemo run and worked on another (same host/dongle/channel,
2026-07-11); the heap path is 100% reliable. Root cause unknown.

**Change.** Flip `DeviceConfig::Usb::rx_zerocopy` default `true → false`
(opt-in via `DEVOURER_RX_ZEROCOPY=1`). Update the field comment and the
performance doc with the observed failure ("intermittent zero-delivery on
an xhci desktop host, 2026-07-11; default off until root-caused").
Time-boxed (~30 min) inspection of `UsbTransport.cpp`'s alloc/submit/reap
fallback for an obvious flaw; findings recorded in the commit message
either way. No open-ended debugging.

## Validation (bench acceptance, reusing the 2026-07-11 harness)

Setup: drone `root@192.168.10.152` (8822EU, maburd/txdemo ARM builds via
mabur `tools/build-arm.sh` tree), host 8812AU + rxdemo, ch149. All captures
run with `DEVOURER_RX_ZEROCOPY` unset (heap default after fix 4).

1. **Fix 1 A/B:** branch ARM txdemo, `DEVOURER_TX_WITH_RX=thread`, no other
   env → expect thousands of clean MCS0 `rx.frame` decodes / 15 s (was 0).
   With `DEVOURER_PROTECT_PATHB_AGC=1` → expect the legacy hash (knob
   verified). Then maburd rebuilt on the branch, launched env-free →
   ≥99% CRC-clean MCS0 + a working `live_play.py` video session.
2. **Fix 2:** txdemo `DEVOURER_TX_RATE=MCS0/LDPC/STBC` → host `rx.frame`
   shows `ldpc:1 stbc:1` (was 0/0). maburd E2E re-run to confirm the
   8812AU still decodes LDPC/STBC frames cleanly; if not, record the
   finding (mabur can disable via its now-functional `flags.*` config).
3. **Fix 3:** 60 s TX+RX soak with INFO logs — coex ticks continue, clean
   shutdown joins the thread. (Fault injection impractical; review + soak.)
4. **Fix 4:** stock rxdemo logs the heap URB line by default;
   `DEVOURER_RX_ZEROCOPY=1` still selects DMA. Host `ctest` green,
   including the new selftest.
5. **Wrap-up (mabur side):** merge to fork master; bump mabur
   `third_party/devourer` submodule pin; rebuild + redeploy ARM maburd
   (env-free launch); update mabur `docs/bench-validation.md` (bugs 4–7 →
   fixed, bench-state section rewritten). Deliberately left open: the
   RX-desense-at-range question — to be answered during the E-series using
   the new `protect_pathb_agc` knob.

## Out of scope

- TX-path-A-only mapping in TX+RX mode (the "rightest" 0x41e8 fix): a
  possible later refinement if E-series range tests show real desense with
  the ref written.
- Root-causing zerocopy beyond the time-boxed look.
- Upstream PRs (fork-only; commits kept self-contained so PRs stay cheap).
