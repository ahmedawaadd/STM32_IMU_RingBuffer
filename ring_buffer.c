#include "ring_buffer.h"

void rb_init(rb_t *rb) {
    rb->head = 0;
    rb->tail = 0;
    rb->hwm = 0;
    // not zeroing buf: empty is head==tail, so uninit slots are never
    // read. saves a 1.5kb memset on boot — not much, but it's also
    // pointless work.
}

// both of these are racy reads from main while the isr is running. that's
// intentional — stats don't need to be consistent with anything else.
// torn reads aren't possible because rb_t members are 32-bit aligned and
// m4 makes single-word loads atomic.

uint32_t rb_count(const rb_t *rb) {
    uint32_t h = rb->head;
    uint32_t t = rb->tail;
    return (uint32_t)(h - t);
}

uint32_t rb_watermark(const rb_t *rb) {
    return rb->hwm;
}
