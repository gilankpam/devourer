#ifndef RX_SENSE_H
#define RX_SENSE_H

#include <cstdint>

/* RxEnergy — a frame-free RX energy / channel-busy snapshot. This is the read
 * side of the DEVOURER_CW_TONE emitter: a coarse "how much in-band energy /
 * channel activity is here" measurement that does NOT require receiving a frame.
 *
 * Filled by IRtlDevice::GetRxEnergy() from the chip's phydm facilities:
 *   - false-alarm (FA) + CCA (channel-busy) counters,
 *   - the DIG initial-gain index (a noise-floor proxy),
 *   - and, where triggered, the NHM in-band power histogram.
 *
 * All values are channel-wide scalars — no Realtek 88xx chip exports
 * per-subcarrier CSI to the host, so this is energy, not a spectrum. Build a
 * coarse spectrum by sweeping channels/bins and sampling this per bin.
 *
 * FA/CCA counts are the DELTA since the previous read that reset them (each
 * GetRxEnergy() call resets both the OFDM and CCK halves; a JGR3
 * GetRxEnergyScout() call resets only the OFDM half — see valid_cck below),
 * so a strong in-band carrier shows up as a jump in cca_ofdm / fa_ofdm and a
 * rise in igi. Every field carries a valid_* flag because the facilities
 * differ by chip generation.
 *
 * valid_fa does NOT cover the CCK counters on its own: on a chip whose scout
 * path skips the CCK reset (JGR3), a scout call fills fa_ofdm/cca_ofdm with
 * a real delta (valid_fa=true) while leaving fa_cck/cca_cck at zero with
 * NO reset behind that zero (valid_cck=false) — a caller that sums the OFDM
 * and CCK halves un-gated would silently undercount every scout-sourced
 * sample relative to a full-read sample, which is exactly what
 * src/chanmig/EvidenceStore.h and src/hopset/HopsetSense.h now gate on
 * valid_cck to avoid. The inverse also follows: because the scout skipped
 * the CCK reset during its dwell, the NEXT GetRxEnergy() call's fa_cck/
 * cca_cck is a delta since the last actual CCK reset (the last GetRxEnergy,
 * not the last GetRxEnergyScout in between) — i.e. it can span more than
 * one nominal sampling interval whenever scout calls were interleaved.
 *
 * The read splits into two very different costs, which is why the caller picks
 * (`GetRxEnergy(bool with_nhm)`): the scalars below are a handful of register
 * reads, while the NHM histogram arms a ~2 ms measurement window and then polls
 * a ready bit at 1 ms granularity. Anything sampling at a sub-second cadence —
 * or throwing the read away to reset the counters before an observation
 * window — wants with_nhm=false. */
struct RxEnergy {
  /* phydm false-alarm + CCA counters (delta since the previous reset of
   * each half — see the valid_cck note above for why that is not always
   * "the previous read"). */
  bool valid_fa = false;
  uint32_t fa_ofdm = 0;  /* OFDM false-alarm count */
  uint32_t fa_cck = 0;   /* CCK false-alarm count */
  uint32_t cca_ofdm = 0; /* OFDM CCA (channel-busy) count */
  uint32_t cca_cck = 0;  /* CCK CCA count */
  /* True when fa_cck/cca_cck came from a real reset-then-read (GetRxEnergy).
   * False means the CCK half was never reset for this sample — either the
   * generation has no CCK facility, or (JGR3) the read was a scout call
   * that deliberately skips the CCK toggles to stay cheap — and fa_cck/
   * cca_cck are 0 with no delta behind that 0, not "no CCK activity".
   * Consumers that fold fa_cck/cca_cck into fa_ofdm/cca_ofdm MUST gate on
   * this, not on valid_fa (which covers the OFDM half only). */
  bool valid_cck = false;

  /* DIG initial-gain index (0x0c50[6:0] on the AC BB): the AGC backs the gain
   * off as the in-band floor rises, so a higher IGI means a busier/noisier
   * channel. Noise-floor proxy. */
  bool valid_igi = false;
  uint8_t igi = 0;

  /* NHM in-band power histogram: 12 IGI-referenced power buckets and the
   * measurement duration. A frame-free power distribution (fuller than the
   * scalar FA counts). Only populated when the caller asked for it
   * (GetRxEnergy(with_nhm=true)). */
  bool valid_nhm = false;
  uint8_t nhm[12] = {};
  uint16_t nhm_duration = 0;

  /* Active/frame-free ABSOLUTE noise floor (dBm) — the vendor idle-noise
   * monitor, distinct from the passive rssi-snr floor in RxQuality. Heavy
   * (~10 ms of USB round-trips), so it is only filled when the caller opted in
   * (DEVOURER_RX_NOISE_FLOOR). Jaguar2 fills it live (HW idle-noise report,
   * wedge-free); Jaguar1 8812A/8821A fill it from an RX-idle CAL measurement;
   * Jaguar3 and Kestrel fill it from an absolute-threshold NHM idle window
   * (no idle-noise report on those; NoiseFloorMath.h); the RTL8733B leaves it
   * invalid. */
  bool valid_noise_floor = false;
  int8_t abs_noise_floor_dbm = 0;
};

#endif /* RX_SENSE_H */
