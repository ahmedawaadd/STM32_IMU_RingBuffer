# stm32 imu logger (f446re + bno055)

real-time orientation + linear-accel logger for stm32f446re with an
adafruit bno055. samples at 100hz over i2c (dma) and dumps to sd via
fatfs. the whole reason this exists is to demonstrate an spsc ring
buffer doing real work in firmware where you actually need one.

## the problem

sd cards stall. consumer-grade cards do internal garbage collection and
wear leveling on their own schedule, and when it kicks in you get write
latency spikes of 50-200ms with no warning. there's no way to predict it
and no way to make the card hurry up.

meanwhile the bno055 is producing a fused sample every 10ms, and that
loop cannot slip. miss a sample = corrupt the log. block in the i2c isr
waiting for the sd = the next tim2 tick lands on top of you and we're
done.

so: producer (the sample loop) and consumer (the sd writer) are running
at wildly different and *unpredictable* rates. you need something
between them that lets the producer never wait and the consumer never
lose data.

## why a ring buffer specifically

the candidates:

- **malloc'd queue (linked list).** heap allocation in an isr is asking
  for trouble — fragmentation, non-deterministic timing, requires a
  lock on the free list. no.
- **double / triple buffer.** works for fixed-size frames but here the
  consumer can stall for 20+ samples worth of time, and a "double
  buffer" sized for 20 entries per side is just a ring with worse
  semantics (no smooth backpressure, awkward half-empty handling).
- **mutex-protected queue.** can't block the isr, full stop.
- **lock-free spsc ring.** producer and consumer touch disjoint index
  variables — head only written by producer, tail only written by
  consumer. no locking, no waiting, bounded time on both sides.

the ring buffer is the right shape for this problem: both sides run
independently and the buffer is literally the thing that decouples
their rates.

## why 100hz and not faster

the bno055 has on-chip sensor fusion and the fusion engine runs at
100hz in ndof mode. that's the rated output rate — sampling faster just
hands back the same data multiple times. for raw-only mode (accel +
gyro) the chip can do up to ~1khz, but at that point a non-fused part
like the icm-20948 or lsm6dsv is a better fit and you wouldn't pay for
the bno055's fusion engine.

so the showcase here is at a smaller scale than a 1khz raw logger, but
the architecture is identical and the ring buffer still does its job:
a 200ms sd stall is 20 samples, which puts the watermark at ~30% of a
64-slot buffer. clearly visible on the plot.

## design choices, defended

- **single producer, single consumer.** producer is the i2c dma
  complete isr. consumer is the main loop. the no-locking trick depends
  on this exclusivity — break it and you need cas loops.
- **power-of-2 size (64).** so we mask instead of mod. cortex-m4 has
  no hardware divide. `% 64` compiles to a `udiv` call (~12 cycles).
  `& 63` is one cycle.
- **size = 64 = 1.5kb.** worst observed sd stall ~200ms × 100hz =
  20 samples per stall. 2x for back-to-back stalls (real cards
  cluster gc events). round up to power of 2 → 64. f446re has 128kb
  sram, so 1.5kb is nothing.
- **free-running 32-bit head/tail.** we don't wrap the counters, only
  mask when indexing. `head - tail` gives fill via uint32 modular
  arithmetic. avoids the "lose one slot to distinguish empty from
  full" trick of the wrapped-pointer variant.
- **volatile head/tail/hwm.** stops the compiler from caching them in
  registers across the isr boundary. **volatile is not a memory
  barrier** — it constrains the compiler, not the cpu. see the m7
  section below for why that matters there and not here.
- **watermark counter.** every push checks if `(head - tail) > hwm`
  and updates. this is the proof — after a real test run i can read
  hwm over uart and see exactly how close we came to overflowing.
  without it i'd be claiming the buffer is sized right based on vibes.
- **i2c @ 400khz, dma.** fast mode. a 20-byte burst read takes ~650us
  including the bno055's clock-stretching on internal page switches —
  comfortable inside the 10ms tim2 period.
- **isr does kick i2c + push, nothing else.** parsing the 20 bytes
  happens in the i2c dma complete callback (still isr context, but
  the timing-critical bit — the 100hz tim2 edge — is already over).
- **f_write in batches of 64.** 64 × 24b = ~3 sd blocks per call.
  fatfs has per-call overhead so batching is cheaper. at 100hz a full
  batch is 640ms of data so we'll usually write in smaller chunks;
  the cap mostly matters *after* a stall when there's a backlog.
- **no periodic f_sync.** sync forces a fat table update, which is the
  single worst latency event on sd. we only sync on shutdown. a crash
  loses a few seconds of recent samples and the fat is recoverable.
- **ndof fusion mode (0x0c).** gives euler + quaternion + linear-accel
  + gravity off one register block. we log euler + quaternion +
  linear-accel (24 bytes per sample, naturally aligned). quat is
  redundant with euler but it's free off the same burst, and it
  avoids gimbal-lock issues for any post-processing.

## instrumentation output

main loop emits this line every 100ms over usart2 @ 115200:

```
t=12345 cnt=3 pct=4 hwm=19 hwm_pct=29 drops=0
```

| field | meaning |
|---|---|
| `t` | ms since boot |
| `cnt` | current ring buffer fill (samples) |
| `pct` | cnt as % of capacity |
| `hwm` | max fill ever observed |
| `hwm_pct` | hwm as % of capacity |
| `drops` | rb_push failures. should stay 0 in any well-sized run |

i pipe this into a python script and plot `pct` vs `t`. sd gc events
look like ramp-then-drain shapes: buffer fills during the stall, drains
fast once the card recovers. that ramp is the ring buffer doing exactly
what it's there to do.

## results (fill in after running)

| metric | value |
|---|---|
| sd card under test | _ |
| run duration | _ min |
| samples logged | _ |
| max observed sd write stall | _ ms |
| max ring buffer fill (hwm) | _ / 64 ( _ %) |
| mean fill | _ samples |
| drops | _ |
| log file size | _ mb |

if `hwm_pct` settles well under 50% over a long run, the buffer is
comfortably sized. if `drops > 0` ever, either the buffer is too small
for this card or this card is way worse than the spec sheet claims —
both worth knowing.

## hardware

- mcu: nucleo-f446re (cortex-m4, 180mhz, 128k sram)
- sensor: adafruit bno055 breakout (i2c @ addr 0x28)
- sd: microsd breakout on sdio (4-bit)
- debug: usart2 on the nucleo's st-link vcp

### wiring

| bno055 pin | nucleo-f446re pin | notes |
|---|---|---|
| vin | 5v | adafruit board has 3v3 regulator on it |
| gnd | gnd | |
| sda | pb9 (d14, arduino sda) | i2c1 sda |
| scl | pb8 (d15, arduino scl) | i2c1 scl |
| rst | pa8 (d7) | optional — wire this if you want a hard-reset escape hatch |

| microsd | nucleo pin |
|---|---|
| d0 | pc8 |
| d1 | pc9 |
| d2 | pc10 |
| d3 | pc11 |
| ck | pc12 |
| cmd | pd2 |

## bno055 quirks worth knowing

- **~650ms boot time** before the chip answers i2c. init blocks on
  `hal_delay` for this. don't try to talk to it sooner — you'll get
  nacks and waste time figuring out why.
- **clock stretching.** the bno055 stretches scl during register page
  switches, sometimes for hundreds of microseconds. stm32 i2c handles
  it transparently but it eats into your budget if you sample fast.
- **mode switching has a settling time.** config → fusion is 7ms
  minimum per the datasheet. we wait 20ms after writing opr_mode.
- **little-endian register layout.** which is great because so is the
  arm, but the icm-20948 is big-endian. if you're porting from a
  big-endian sensor the byte-shifts go the other way.
- **the chip can wedge.** rare but documented — the adafruit forums
  have a lot of threads about it. wire `rst` to a gpio if you want to
  recover from this without a power cycle.

## what i'd change for mpsc

if there were multiple producers — say a second i2c bus with another
sensor also calling `rb_push` — the spsc proof breaks. head would have
two writers and the simple non-atomic increment becomes a race. options:

- **cas loop on head.** each producer does `__LDREXW(&head)` /
  `__STREXW(new_head)` to reserve a slot atomically. then the consumer
  can't just check `head != tail` because slots between tail and the
  new head may not be populated yet — you need a per-slot "ready"
  flag the producer sets after writing, and the consumer scans for
  the next ready slot. doable, but a real step up in complexity.
- **per-producer rings + consumer round-robin.** each producer gets
  its own spsc ring (this code, unchanged), consumer pulls from each.
  simpler. usually the right answer if you can structure it that way.

for multi-consumer the same story plays out on tail, and you typically
reach for a real lock-free queue at that point (vyukov mpmc bounded
queue is a good reference). the elegance of spsc comes from not needing
any of that — preserve it if you can.

## what i'd change for cortex-m7

f446re is m4 like the f411 i had on the bench previously — no l1 cache,
single core, in-order pipeline. that's what makes the volatile-only
approach safe here. on m7 (e.g. h7-series) i'd need:

- **`__DMB()` between the slot write and the head bump in `rb_push`**
  (release semantics — slot data must be visible before the head bump
  is observable).
- **`__DMB()` between the head read and the slot read in `rb_pop`**
  (acquire semantics).
- **cache maintenance if the ring buffer is in a region dma also
  touches.** `SCB_CleanDCache_by_Addr` before any dma reads from it,
  `SCB_InvalidateDCache_by_Addr` after any dma writes into it. in
  this project the i2c dma writes into `bno_rx` (a separate buffer),
  not the ring buffer itself, so we'd be fine there — but it's the
  kind of thing that bites you if you forget. alternatively, put the
  ring in a non-cacheable mpu region and pay the perf cost.
- **tcm placement.** on h7 you can put the ring in dtcm for zero-wait
  access from the core, which is great, but dma masters can't see
  tcm. so the dma buffer and the ring stay separate, which is how we
  already do it.

on m4 single core with no cache, single in-order observer per side +
exception entry/exit having implicit barriers means `volatile` is
sufficient. that's why you can read this code and not see a single
`__DMB()`. on m7 you'd see them on every publish.

## file layout

| file | what's in it |
|---|---|
| [ring_buffer.h](ring_buffer.h) | the spsc ring. push/pop are inline here. start here. |
| [ring_buffer.c](ring_buffer.c) | init + the racy-by-design stat readers. tiny. |
| [main.c](main.c) | timer/i2c/sd glue, isr callbacks, drain loop, bno055 bringup. |

## build

cubeide project against the f4 cube hal + fatfs middleware. the only
non-cube source files are the three above. nothing fancy in the build —
the point is the design, not the toolchain.
