/* ctrl_batch_selftest.cpp — the SEMANTIC contract of
 * IRtlTransport::ctrl_batch (batched EP0 register transfers, mabur in-flight
 * hop 2026-09-14), pinned against a fake transport with no bus under it.
 *
 * What this file is for: the contract, not the plumbing. Every transport
 * implementing ctrl_batch — the synchronous default here, UsbTransport's
 * pipelined EP0 version, any future one — owes the caller all four of:
 *
 *   1. ORDER. Ops execute in submission order, so a read placed after a
 *      write to the same address in one batch observes that write. (On USB
 *      this rests on EP0 completing URBs in submission order, the same
 *      property the pipelined write batch already relies on.)
 *   2. IN-PLACE READS. Each read op's value lands in THAT op's `value`
 *      field. Not the next one's, not the k-th used slot's. This is the
 *      clause the UsbTransport implementation is most able to get subtly
 *      wrong: if the submit of op i fails mid-chunk and the completion loop
 *      then walks ops and slots with two independent cursors, every op after
 *      the failure silently receives its neighbour's register value — a
 *      wrong number, not an error. Case 2 below pins exactly that shape at
 *      the contract level (the USB implementation additionally makes it
 *      structurally impossible by pairing each op with its own slot pointer
 *      and copying inside the wait loop, which is also required for a second
 *      reason: async_write_cb returns the slot to the free pool the instant
 *      it completes, so a deferred copy pass reads buffers the pool already
 *      considers reusable).
 *   3. FAILURE IS REPORTED, NOT SWALLOWED. A single failed op makes the
 *      whole call return false...
 *   4. ...but does NOT abort the batch: the remaining ops still run. A hop's
 *      write group is a sequence the chip needs finished (a 3-wire bracket
 *      left open, a BB reset left asserted), so bailing out halfway is worse
 *      than finishing and reporting.
 *
 * ONE DELIBERATE DIVERGENCE, so case 2's "every op still ran" is not read as
 * a universal: clause 4 as written is what the SYNCHRONOUS default owes, and
 * that is what this file pins. UsbTransport::ctrl_batch attempts every op but
 * genuinely skips two classes — an op whose libusb_submit_transfer failed,
 * and the rest of a chunk after a completion wait gave up — because there is
 * no way to issue them. It still runs every LATER chunk, still finishes the
 * remainder synchronously if the pool dies, and still returns false. That is
 * intended: "attempted, and any failure reported" is the portable clause,
 * "executed" is the default's stronger guarantee.
 */
#include <cstdio>
#include <cstdlib>
#include <map>
#include <set>
#include <vector>

#include "RtlTransport.h"

namespace {

struct Fake : devourer::IRtlTransport {
  std::map<uint16_t, uint32_t> regs;
  std::vector<uint16_t> order;   /* every register touched, in order */
  std::set<uint16_t> fail_write; /* writes to these addresses report failure */

  /* ---- the two methods the default ctrl_batch actually drives ---- */
  uint32_t read32(uint16_t a) override {
    order.push_back(a);
    return regs[a];
  }
  bool write32(uint16_t a, uint32_t v) override {
    order.push_back(a);
    if (fail_write.count(a))
      return false;
    regs[a] = v;
    return true;
  }

  /* ---- the rest of the pure-virtual surface: minimal stubs ---- */
  bool is_usb() const override { return false; }
  uint8_t read8(uint16_t) override { return 0; }
  uint16_t read16(uint16_t) override { return 0; }
  bool write8(uint16_t, uint8_t) override { return false; }
  bool write16(uint16_t, uint16_t) override { return false; }
  bool write_bytes(uint16_t, const uint8_t *, size_t) override { return false; }
  bool tx_async(uint8_t, uint8_t *, size_t, unsigned) override { return false; }
  int tx_sync(uint8_t, uint8_t *, size_t, int) override { return -1; }
  void rx_loop(int, int, const std::function<void(const uint8_t *, int)> &,
               const std::function<bool()> &) override {}
};

int fails = 0;

} // namespace

#define CHECK(c)                                                               \
  do {                                                                         \
    if (!(c)) {                                                                \
      std::fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #c);         \
      ++fails;                                                                 \
    }                                                                          \
  } while (0)

int main() {
  /* ---- case 1: order, and a read sees an earlier write in the same batch */
  {
    Fake f;
    f.regs[0x2c08] = 0x00050003;
    std::vector<devourer::CtrlOp> ops = {
        {false, 0x2c08, 0}, {true, 0x1d2c, 0x1234}, {false, 0x1d2c, 0}};
    CHECK(f.ctrl_batch(ops));
    CHECK(ops[0].value == 0x00050003);
    CHECK(ops[2].value == 0x1234);
    CHECK(f.order.size() == 3 && f.order[0] == 0x2c08 && f.order[1] == 0x1d2c &&
          f.order[2] == 0x1d2c);
  }

  /* ---- case 2: a failing op mid-batch must not mis-pair the reads after it,
   * must not abort the rest of the batch, and must make the call false.
   * Distinct register values so a one-slot mis-pairing is visible as a wrong
   * number rather than a coincidence. */
  {
    Fake f;
    f.regs[0x2d04] = 0xaaaa0000;
    f.regs[0x2d08] = 0xbbbb1111;
    f.regs[0x2d10] = 0xcccc2222;
    f.fail_write.insert(0x1c90);
    std::vector<devourer::CtrlOp> ops = {{false, 0x2d04, 0},
                                         {true, 0x1c90, 0xdead},
                                         {false, 0x2d08, 0},
                                         {true, 0x0808, 0xbeef},
                                         {false, 0x2d10, 0}};
    CHECK(!f.ctrl_batch(ops)); /* (3) failure reported */
    CHECK(ops[0].value == 0xaaaa0000);
    CHECK(ops[2].value == 0xbbbb1111); /* (2) own value, not the neighbour's */
    CHECK(ops[4].value == 0xcccc2222);
    CHECK(f.order.size() == 5); /* (4) every op still ran */
    CHECK(f.regs[0x0808] == 0xbeef); /* the write after the failure landed */
    /* The write op's `value` is an input and must come back untouched — a
     * caller re-issuing the same vector must not find it clobbered. */
    CHECK(ops[1].value == 0xdead);
  }

  /* ---- case 3: an empty batch is a successful no-op (the cached-hop path
   * builds its vector conditionally and can legitimately end up empty). */
  {
    Fake f;
    std::vector<devourer::CtrlOp> ops;
    CHECK(f.ctrl_batch(ops));
    CHECK(f.order.empty());
  }

  /* ---- case 4: a write-only batch, the shape the cached FastRetune issues */
  {
    Fake f;
    std::vector<devourer::CtrlOp> ops = {
        {true, 0x1c90, 1}, {true, 0x3c60, 2}, {true, 0x4c60, 3},
        {true, 0x1c90, 4}, {true, 0x1830, 5}, {true, 0x4130, 6},
        {true, 0x0000, 7}, {true, 0x0000, 8}, {true, 0x0000, 9},
    };
    CHECK(f.ctrl_batch(ops));
    CHECK(f.order.size() == 9);
    CHECK(f.regs[0x1c90] == 4); /* last write wins — order held */
    CHECK(f.regs[0x0000] == 9);
  }

  if (fails)
    return EXIT_FAILURE;
  std::puts("ctrl_batch_selftest: ok");
  return 0;
}
