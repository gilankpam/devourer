#ifndef DEVOURER_BULK_OUT_TIMEOUT_H
#define DEVOURER_BULK_OUT_TIMEOUT_H

#include <cstddef>

namespace devourer {

/* The libusb timeout a bulk-OUT of `len` bytes may safely carry.
 *
 * libusb cancels a transfer on timeout, and a transfer longer than one USB
 * packet can be cancelled after the device already accepted some of its
 * packets. With several sender threads queued on one endpoint, the next queued
 * transfer then streams straight in as the rest of the half-received packet:
 * the Realtek TXDMA misparses it (TXDMA_STATUS PAYLOAD_UDN), keeps its pages,
 * NAKs every later bulk-OUT, and TX is dead until re-init. Bench-proven on the
 * 8822EU drone (4 senders, carrier-sense-free jam, 2026-09-24: `rc=-7 got
 * 1536/4439`, then nothing; finishing the cancelled tail afterwards did NOT
 * help -- the splice had already happened). So a multi-packet transfer gets
 * no timeout: it waits for the chip, the same backpressure the async path
 * (tx_async, timeout 0) already relies on. A transfer of at most one packet is
 * accepted whole or not at all, so it keeps the caller's timeout.
 *
 * `max_packet` 0 = unknown: assume 64 (the smallest bulk packet), i.e. err
 * towards never cancelling. */
inline int bulk_out_timeout_ms(size_t len, unsigned max_packet, int requested) {
  const size_t mps = max_packet ? max_packet : 64;
  return len > mps ? 0 : requested;
}

} // namespace devourer

#endif /* DEVOURER_BULK_OUT_TIMEOUT_H */
