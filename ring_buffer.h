// spsc ring buffer for the bno055 → sd path. centerpiece of the project.
//
// producer: i2c dma complete isr (assembles sample, pushes one).
// consumer: main loop (drains in batches into fatfs).
//
// the whole reason this exists: sd writes stall 50-200ms occasionally for
// internal gc. the sample loop can't tolerate that, so we decouple them.
// producer never waits, consumer never loses data, no locks.
//
// alternatives considered (see readme for the full version):
//   - malloc'd queue: heap on the isr path, no thanks
//   - double buffer: too small for a 200ms stall, awkward semantics
//   - mutex queue: isr can't block
// spsc ring wins because head and tail are touched by exactly one side
// each — no shared write state means no critical section.

#pragma once

#include <stdint.h>
#include <stdbool.h>

// 24 bytes, naturally 4-byte aligned. bno055 ndof output: we keep euler
// (3 × i16) + quaternion (4 × i16) + linear accel (3 × i16) + a 32-bit
// timestamp. quat is redundant with euler but it's free off the same
// burst read and avoids gimbal-lock issues if anyone post-processes.
typedef struct {
    uint32_t ts;             // ms since boot (hal_getttick at i2c completion)
    int16_t  eh, er, ep;     // euler heading/roll/pitch, 1/16 deg lsb
    int16_t  qw, qx, qy, qz; // quaternion, 1/16384 lsb
    int16_t  lax, lay, laz;  // linear accel (gravity removed), 1/100 m/s² lsb
} sample_t;

// 64 slots, justified:
//   worst observed sd stall ~200ms on consumer cards
//   200ms × 100hz (bno055 ndof fusion rate) = 20 samples per stall
//   2x for back-to-back stalls (real cards cluster gc events)
//   round up to power of 2 → 64
// 64 × 24b = 1.5kb. f446re has 128k sram. trivial.
//
// note this is way smaller than the 512 slots i used in my icm-20948
// design — that one ran at 1khz. the buffer is sized to the rate, not
// to some abstract notion of "big enough". if you swap to raw mode on
// the bno055 (accel+gyro at 1khz, no fusion) you'd want ~512 again.
#define RB_SIZE 64u
#define RB_MASK (RB_SIZE - 1u)  // power of 2 so we mask instead of mod

typedef struct {
    // free-running 32-bit counters. we don't wrap them, only mask when
    // indexing into buf. that way (head - tail) gives fill directly via
    // uint32 modular arithmetic, and we don't lose a slot to the
    // empty-vs-full ambiguity of the wrapped-pointer style.
    volatile uint32_t head;  // isr writes, main reads
    volatile uint32_t tail;  // main writes, isr reads

    // highest fill ever observed at push time. producer-only writer, main
    // reads it racily for the uart stats line. 32-bit aligned writes are
    // atomic on m4 so a torn read is impossible — worst case main sees a
    // stale value, which is fine for a diagnostic.
    volatile uint32_t hwm;

    sample_t buf[RB_SIZE];
} rb_t;

void rb_init(rb_t *rb);
uint32_t rb_count(const rb_t *rb);
uint32_t rb_watermark(const rb_t *rb);

// push and pop live in the header as static inline so the compiler can
// fold them into the isr / main loop without depending on lto. they're
// short enough that this isn't bloat.

static inline bool rb_push(rb_t *rb, const sample_t *s) {
    // don't need to disable interrupts: spsc means head and tail are
    // only touched by one side each. the volatile reads kill the
    // compiler from caching tail in a register across calls.
    uint32_t h = rb->head;
    uint32_t t = rb->tail;

    if ((uint32_t)(h - t) >= RB_SIZE) {
        // full. caller bumps a drop counter. we deliberately don't
        // overwrite oldest — silent data loss is exactly the failure
        // mode we want to make visible.
        return false;
    }

    rb->buf[h & RB_MASK] = *s;

    // publish. the slot store above must land before the head bump
    // becomes visible to the consumer, otherwise it could pop garbage.
    // on single-core m4 (f446re) with normal sram + one in-order
    // observer per side, program order is preserved by the cpu and
    // volatile stops compiler reordering. exception entry/exit have
    // implicit barriers built in, so there's no ordering hole at the
    // isr boundary either. no explicit __DMB() needed here on m4.
    // on m7 this line needs a __DMB() before it (see readme).
    rb->head = h + 1;

    uint32_t fill = (h + 1) - t;
    if (fill > rb->hwm) rb->hwm = fill;
    return true;
}

static inline bool rb_pop(rb_t *rb, sample_t *out) {
    uint32_t t = rb->tail;
    uint32_t h = rb->head;

    if (h == t) return false;

    *out = rb->buf[t & RB_MASK];

    // release the slot. same single-core / single-observer reasoning as
    // push — the read above must finish before tail moves, and program
    // order + volatile gets that for free on m4.
    rb->tail = t + 1;
    return true;
}
