/* Headless guard for src/BulkOutTimeout.h: a bulk-OUT longer than one USB
 * packet must never be given a timeout (libusb cancels on timeout). With
 * several sender threads queued on one endpoint, cancelling a transfer the
 * chip has half-received lets the NEXT queued transfer's bytes stream in as
 * the rest of that packet -- the Realtek TXDMA misparses (TXDMA_STATUS
 * PAYLOAD_UDN), holds its pages, and TX is dead until re-init (8822EU drone,
 * 4 senders, carrier-sense-free jam, bench 2026-09-24). A single-packet
 * transfer is accepted whole or not at all, so it keeps its timeout. */
#include "BulkOutTimeout.h"

#include <cstdio>

using devourer::bulk_out_timeout_ms;

static int g_fail = 0;

static void check(const char *what, long got, long want) {
  if (got != want) {
    std::printf("FAIL %s: got %ld want %ld\n", what, got, want);
    ++g_fail;
  }
}

int main() {
  /* High-speed endpoint, 512-byte packets. */
  check("control frame keeps timeout", bulk_out_timeout_ms(93, 512, 20), 20);
  check("exactly one packet keeps timeout", bulk_out_timeout_ms(512, 512, 20),
        20);
  check("one byte over never cancels", bulk_out_timeout_ms(513, 512, 20), 0);
  check("3-frame URB never cancels", bulk_out_timeout_ms(4439, 512, 50), 0);
  check("caller's infinite stays infinite", bulk_out_timeout_ms(93, 512, 0), 0);

  /* SuperSpeed endpoint, 1024-byte packets. */
  check("SS 1000 B keeps timeout", bulk_out_timeout_ms(1000, 1024, 20), 20);
  check("SS 1025 B never cancels", bulk_out_timeout_ms(1025, 1024, 20), 0);

  /* Unknown packet size: assume the smallest bulk packet (64, full-speed),
   * i.e. err towards never cancelling. */
  check("unknown mps, 64 B keeps timeout", bulk_out_timeout_ms(64, 0, 20), 20);
  check("unknown mps, 93 B never cancels", bulk_out_timeout_ms(93, 0, 20), 0);

  if (g_fail) {
    std::printf("bulk_out_timeout_selftest: %d failure(s)\n", g_fail);
    return 1;
  }
  std::printf("bulk_out_timeout_selftest: all passed\n");
  return 0;
}
