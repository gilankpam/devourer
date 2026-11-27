# LA-mode IQ capture — per-tone CSI from Jaguar silicon

No Jaguar-class chip exports per-subcarrier CSI through a normal host
path. phydm's **LA (logic-analyzer / ADC-sampling) mode** is the debug
escape hatch: the baseband DMAs a raw sample bus into the top of the TX
packet buffer, and the host reads it back through the `0x0140`-page +
`0x8000`-window. An offline FFT of a captured L-LTF **is** the full
per-tone channel estimate H(k) — one-shot and slow, but it turns a dongle
into a per-subcarrier channel sounder. Implementation:
`src/LaCapture.{h,cpp}` (one algorithm, per-family `LaRegs` — the
NhmReader pattern); vendored source: `reference/rtl88x2bu` (11AC dialect)
and `reference/rtl88x2cu`/`rtl88x2eu` (JGR3) `hal/phydm/phydm_adc_sampling.c`
+ `phydm_debug.c`.

## Silicon support

`PHYDM_IC_SUPPORT_LA_MODE` (phydm_pre_define.h): **8814A, 8822B, 8821C,
8822C, 8822E**. The **8812A and 8821A have no LA block** — `0x7c0` never
latches the arm bits there and the module reports `no_la_block`
(`la.nosupport` event). Bench-validated on all five supported chips
(USB) plus the **8821CE over PCIe** (same module through `RtlAdapter`;
readback 8 ms via MMIO vs ~2 s over USB control transfers).

| chip | TXFF window | size (HALF mode) |
|---|---|---|
| 8814A | 0x30000..0x40000 | 64 KB |
| 8822B / 8822C / 8822E | 0x20000..0x40000 | 128 KB |
| 8821C / 8821CE | 0x8000..0x10000 | 32 KB (`0x7cc[30]` doubles it) |

## Usage

```sh
# one-shot capture, CCA-triggered, 20 Msps, ADC bus path A:
DEVOURER_LA_CAPTURE=cca/20M/dma0/port:0x880/t200 \
DEVOURER_LA_OUT=/tmp/la.bin DEVOURER_LA_MAX=8192 build/rxdemo

python3 tools/la_decode.py score /tmp/la.bin --offset-mhz 4  # layout scoring
python3 tools/la_decode.py plot  /tmp/la.bin                 # PSD + |x|
python3 tools/la_csi.py /tmp/la.bin --plot h.png             # per-tone H(k)
```

Spec grammar: `<trig>[/<rate>M][/dma<N>][/port:0xNNN[.hdr<H>][.bit<B>]]`
`[/edge<0|1>][/t<us>][/all]` — `<trig>` ∈ `manual|crcok|crcfail|cca|bb|mac`.
`t<us>` is the post-trigger time; the rest of the ring is pre-trigger, so
a `crcok` capture (trigger = frame **end**) holds the whole frame
including the preamble. `DEVOURER_LA_MAX` caps readback and keeps the
**newest** samples (the trigger vicinity). `DEVOURER_LA_DELAY_MS`
(default 2000) is the arm delay after RX is live. The demo writes a
32-byte `DVLA` header + LE u64 records; events: `la.capture`,
`la.timeout`, `la.nosupport`, `la.wedged`.

## Sample format (established empirically — tests/la_cw_score.sh)

At `dma0/port:0x880`, every chip packs **one complex sample per 64-bit
word**: 12-bit two's complement, **I=[11:0], Q=[23:12]** of the low dword
(`qi12_l` in tools/la_decode.py — the naive I/Q order is spectrally
inverted), at exactly the configured sample rate. JGR3 (8822C/E)
additionally carries the **second RX path** in the high dword
(I=[55:44], Q=[43:32], `iq12_h`) — a coherent two-chain capture in one
trigger. CW line scores (peak/median): 8822CU 50.8 dB, 8821C 36.8 dB,
8822BU 23.6 dB, 8814AU 19.1 dB.

## Hardware findings the vendor code hides

- **Manual REG trigger ordering**: the `0x7c0[5]` 0→1 edge must be pulsed
  **after** the dump-enable bit `0x7c0[1]` is set. The vendor sets it
  earlier in `phydm_la_set_mac_iq_dump`; that order never completes on
  real 8822BU silicon (edge consumed before arm).
- **dma_type alone doesn't select a live bus** — the BB dbg-port value
  routes the sampled bus, and `dbg_port=0` is a dead bus (clean poll
  timeout). ADC pairing: `dma0`+`0x880` (path A), `dma1`+`0xa80` (path B).
- **8821C cut A** has no LA clock bit (`0x95c[23]` skipped, like vendor
  `phydm_la_clk_en`); expect a poll timeout there.
- **TX-buffer conflict**: capture windows occupy the top of the TXFF and
  the smoke test's capture→TX→capture cell passes, but don't run a
  capture concurrent with heavy TX. The vendor flow leaks `0x0106=0x69`
  (TXFF debug access); LaCapture restores it.

## Validation

- `tests/la_capture_smoke.sh <PID> [ch] [tx_pid] [vid]` — per-chip:
  manual trigger, crcok trigger, capture→TX→capture. 3/3 PASS on
  8822BU, 8814AU, 8821C(8811CU), 8822CU, 8822EU.
- `tests/la_cw_score.sh` — layout scoring against a B210 CW at a known
  offset (layout-scoring methodology).
- `tests/la_sdr_crosscheck.sh <PID> [ch] [vid] [min_notch]` — the
  **notch protocol**: the B210 transmits an L-LTF train with an
  asymmetric tone set zeroed; the chip's H(k) must reproduce the notch at
  exactly those tones (local-neighbour depth). 8822BU: 21–31 dB on all
  six tones. **Chip-vs-SDR |H(k)| correlation was refuted as a
  methodology**: receivers at different antennas see independent fading
  (measured ~0 correlation with both sides self-consistent).
- `ctest` `la_csi_math` — synthetic L-LTF through a 2-tap channel + CFO +
  AWGN; |H(k)| recovery correlation > 0.99.

## Per-chip CSI quality caveats

- **8822CU (Jaguar3)**: ~8 dB adjacent-tone leakage floor on **both** RX
  paths (common LO — phase-noise/capture-path spreading; unchanged by FFT
  window backoff or multi-period CFO refinement). Single-tone features
  bound at ~5–8 dB at their edges; features ≥2 tones wide resolve at
  15–23 dB. Pass the crosscheck with `MIN_NOTCH=5`.
- The B210 helper (`tests/la_sdr_compare.py txltf`) transmits the LTF
  train as long precomputed buffers — per-burst `send()` calls underrun
  from python and nothing airs.

## Cross-path capture (8822E)

On the 8822E one chain can hear the other: chain A transmits a probe train
while chain B, re-pointed as a listener, feeds the path-B ADC tap
(`dma1`/`0xa80`) into the LA capture. Each capture is the radiated probe as
the second chain receives it at a chosen flat TXAGC index and listener gain.
That gives a relative TX-power reading per index without an external
receiver, which is what the TX power ladder's self-calibration needs.
Implementation: `src/jaguar3/CrossPath8822e.{h,cpp}` (listener and
`CrossPathSession`), shared types in `src/CrossPathCapture.h`.

The five `IRtlRadio` calls. None of them throws: a failed register transfer
is a false return.

- `CrossPathSupported()`: true on an 8822E with `rfe_type` 21–24 after
  bring-up, with no CW tone running. Every other chip and RFE type returns
  false, and the other four calls then refuse without writing anything.
  Validated at 5 GHz only; the call does not check the band, and a 2.4 GHz
  session is untested.
- `CrossPathBegin(bw)`: `bw` is 20 or 40 MHz, and refused when wider than
  the tuned width. Re-lays the TX buffer to its lower half (the upper half is
  the capture window), sets up the chain-B listener, then starts the probe
  train: HT MCS1 frames (150-byte body at 20 MHz, 320 at 40) sent back to
  back through `send_packet` until `CrossPathEnd`. Refused while a session
  is open, and when the TX queue does not drain within 500 ms (nothing
  written). A failed step undoes the steps before it.
- `CrossPathPark(parked)`: parks or unparks the listener's front end
  (`0x4c[22]`, `0x3c[26]`); false if the bits do not take.
- `CrossPathCapture(tx_index, gain, out)`: sets the flat index override
  (and waits 15 ms) when it changed, writes the listener gain word (3 ms),
  runs one capture (8192 samples at 20 MHz, 16384 at 40) and decodes it into
  `out`. A timed-out or wedged capture returns false with `out` empty, and
  the session can still be closed. A capture holds the device register lock
  throughout; on a trigger that never fires that is the full poll,
  2000 × 2 ms, about 4.5 s with the readback.
- `CrossPathEnd()`: stops the train, waits (up to 500 ms) until the chip's
  queue reports every page free, undoes the listener, re-lays the TX buffer
  to full and restores the flat index override saved at `CrossPathBegin`.
  The listener undo puts `0x4130` back on the AFE table bring-up last wrote
  there (the TXGAPK restore, or the IQK restore when TXGAPK did not run) and
  reads back the other bits it restored. A 40 MHz probe arms the wide-TX IGI
  floor; End clears it again when the session found it clear. The session
  writes neither channel nor width, and the rate-diff table is not written
  either: the flat override goes back to the value it had, and the rate-diff
  walk that runs under it is the one that was there. Returns true when every
  step succeeded and the queue drained before the re-layout. A drain timeout
  is logged at error level and returns false; the undo and the full
  re-layout still run, since leaving TX confined to the half layout is
  worse, but TX may be wedged afterwards. A step that threw keeps the
  session open, so a later End retries. True with no session open. `Stop()`
  calls it first, and so do `Init` and `InitWrite`.

The caller's obligations:

- Nothing else sends from `CrossPathBegin` until `CrossPathEnd`. The
  re-layout moves the queue boundaries under any frame still in the buffer:
  done on a full queue it leaves more free pages than exist and every later
  bulk-OUT times out. The session waits for a drained queue at both ends;
  the train alone keeps the half layout nearly full, and it takes about
  100 ms to empty after the train stops.
- The chip stays on one channel and width for the whole session.
- At 40 MHz the result belongs to the tuned primary channel and offset.
  `ChannelOffset = 0` is HT40+ in this driver, so the upper channel of a
  standard 5 GHz pair (40, 48, ... 144, 161) needs offset 2. Tune the pair
  the link flies: at the wrong pair the bench read EVM 2–5 dB off.
- The probe goes out with the session's TX state: the flat index plus any
  `SetTxPowerOffsetQdb` offset, A-MPDU when `SetAmpduMode` is on, and the
  per-packet TX-power default bank (`SetTxPacketPowerOffsetQdb`). Turn
  A-MPDU off and zero the offset and the per-packet default before
  `CrossPathBegin`.
- One control thread. `Stop()` calls `CrossPathEnd()`, so `Stop` and the
  session calls run on one thread, or the caller serializes them.
- No CW tone runs during a session: it makes the flat index override
  refuse, so captures and the end-of-session restore would not apply.
  `CrossPathSupported()` is false while one is active.

A note on H2C: during the half layout the H2C queue keeps its full-layout
address, which sits inside the capture window. This backend sends its
runtime H2C commands through the HMEBOX registers, not the H2C queue, so
nothing lands there during a session; an H2C packet sent then would.

## Risks

Same class as `BbDbgportReader` (0x8fc/0x95c/0x1c3c writes on a live
demod): every touched register is snapshotted and restored, chip liveness
is checked between phases, and a dead read latches the module wedged
(`la.wedged`). Recovery ladder: `libusb_reset_device` → port-level
usbreset → VBUS power-cycle. Readback cost: two vendor-control reads per
8-byte sample (~1 s per 1 K samples on USB2; MMIO-fast on PCIe).
