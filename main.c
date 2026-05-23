// orientation + linear-accel logger — adafruit bno055 @ 100hz over i2c
// (dma) → spsc ring → fatfs on sd. the interesting code is in
// ring_buffer.h; this file is glue.

#include "stm32f4xx_hal.h"
#include "fatfs.h"
#include "ring_buffer.h"
#include <stdio.h>
#include <string.h>

// peripheral handles set up by cubemx-generated init code (clock tree,
// i2c1 @ 400khz fast mode, tim2 @ 100hz, usart2 @ 115200, sdio 4-bit).
extern I2C_HandleTypeDef  hi2c1;
extern TIM_HandleTypeDef  htim2;
extern UART_HandleTypeDef huart2;
void system_clock_config(void);
void gpio_init(void);
void i2c1_init(void);
void tim2_init(void);
void uart2_init(void);
void sdio_init(void);

static FATFS  fs;
static FIL    log_file;
static rb_t   rb;

// bno055 i2c address. adafruit ties adr low → 0x28. hal wants the 7-bit
// address pre-shifted into the upper 7 bits of a byte.
#define BNO_ADDR (0x28u << 1)

// burst read covers euler (0x1a..0x1f), quaternion (0x20..0x27), and
// linear accel (0x28..0x2d) — one contiguous 20-byte read.
#define BNO_REG  0x1Au
#define BNO_LEN  20u
static uint8_t bno_rx[BNO_LEN];

// isr-side counters. volatile because main reads them for the stats line.
static volatile uint32_t drops;  // rb_push failures. should stay 0.

// tim2 fires at 100hz. kick the i2c dma and get out. anything heavier
// here shows up as timestamp jitter — the bno055 fusion output is
// internally locked to 100hz so even small misalignments are visible if
// you cross-correlate with another sensor.
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance != TIM2) return;
    HAL_I2C_Mem_Read_DMA(&hi2c1, BNO_ADDR, BNO_REG,
                         I2C_MEMADD_SIZE_8BIT, bno_rx, BNO_LEN);
}

// i2c dma done — assemble sample, push to ring. this is *the* producer.
// the timer isr does nothing but kick dma; all the work that doesn't
// have to be inline with the 100hz edge moves here.
//
// the i2c burst takes ~650us at 400khz (the bno055 stretches scl on
// page-switch boundaries inside the chip), which is well inside our
// 10ms period — no chance of one read still running when the next tim2
// tick lands. if you crank the rate up though, watch for this.
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance != I2C1) return;

    // bno055 registers are little-endian: lsb at the lower address.
    // (lucky — both arm and bno055 are little-endian, so this is just
    // a memcpy-shape read.) this is a difference from my icm-20948
    // code, which was big-endian and shifted the other way.
    sample_t s = {
        .ts  = HAL_GetTick(),
        .eh  = (int16_t)((bno_rx[1]  << 8) | bno_rx[0]),
        .er  = (int16_t)((bno_rx[3]  << 8) | bno_rx[2]),
        .ep  = (int16_t)((bno_rx[5]  << 8) | bno_rx[4]),
        .qw  = (int16_t)((bno_rx[7]  << 8) | bno_rx[6]),
        .qx  = (int16_t)((bno_rx[9]  << 8) | bno_rx[8]),
        .qy  = (int16_t)((bno_rx[11] << 8) | bno_rx[10]),
        .qz  = (int16_t)((bno_rx[13] << 8) | bno_rx[12]),
        .lax = (int16_t)((bno_rx[15] << 8) | bno_rx[14]),
        .lay = (int16_t)((bno_rx[17] << 8) | bno_rx[16]),
        .laz = (int16_t)((bno_rx[19] << 8) | bno_rx[18]),
    };
    if (!rb_push(&rb, &s)) drops++;
}

// bno055 init. has to happen *after* peripheral init and *before* tim2
// starts, otherwise the first few reads come back as nacks while the
// chip is still booting.
static void bno055_init(void) {
    uint8_t b;

    // bno055 takes ~650ms after power-on before it answers i2c. anything
    // sooner and you'll see nack stalls in the hal. one delay, once,
    // at startup — fine to be lazy here.
    HAL_Delay(700);

    // sanity check: chip_id at 0x00 should read 0xa0. if not, wiring
    // is wrong or the chip is wedged (the latter happens — adafruit
    // forum is full of it, recover with a hard reset on the rst pin).
    HAL_I2C_Mem_Read(&hi2c1, BNO_ADDR, 0x00, 1, &b, 1, 100);
    if (b != 0xA0) {
        while (1) { /* halt — would blink an led in real fw */ }
    }

    // set operation mode to ndof (full 9-dof fusion). chip boots in
    // config mode by default, so we don't need to drop to config first.
    // datasheet says 7ms switching time from config → fusion; 20ms to
    // be safe.
    b = 0x0C;  // ndof
    HAL_I2C_Mem_Write(&hi2c1, BNO_ADDR, 0x3D, 1, &b, 1, 100);
    HAL_Delay(20);
}

// instrumentation: dump fill % over uart every 100ms. the point of the
// project is to *see* the ring buffer absorbing stalls, not just claim
// it. pipe this into a python script with matplotlib and the gc spikes
// show up as ramp-then-drain shapes.
static void emit_stats(uint32_t now) {
    uint32_t cnt = rb_count(&rb);
    uint32_t hwm = rb_watermark(&rb);
    char line[96];
    int n = snprintf(line, sizeof(line),
        "t=%lu cnt=%lu pct=%lu hwm=%lu hwm_pct=%lu drops=%lu\r\n",
        (unsigned long)now,
        (unsigned long)cnt, (unsigned long)((cnt * 100u) / RB_SIZE),
        (unsigned long)hwm, (unsigned long)((hwm * 100u) / RB_SIZE),
        (unsigned long)drops);
    // blocking tx. 80 bytes at 115200 ≈ 7ms — at 100hz that's worth
    // checking: the producer pushes one sample during the uart tx, so
    // we lose one slot of headroom per stats emit. with a 64-slot
    // buffer that's still fine, but if you ever shrink rb_size, switch
    // this to dma uart.
    HAL_UART_Transmit(&huart2, (uint8_t *)line, n, 100);
}

int main(void) {
    HAL_Init();
    system_clock_config();
    gpio_init();
    i2c1_init();
    tim2_init();
    uart2_init();
    sdio_init();
    bno055_init();   // wake + switch to ndof fusion mode

    rb_init(&rb);

    // truncate on boot — each power-up is a new test run. f_sync inside
    // the loop is the single worst latency source on sd (it forces a
    // fat update), so we don't sync periodically. closing on shutdown
    // is what flushes it, and for crashes the fat is recoverable.
    f_mount(&fs, "", 1);
    f_open(&log_file, "imu.bin", FA_WRITE | FA_CREATE_ALWAYS);

    HAL_TIM_Base_Start_IT(&htim2);

    // batch size: 64 × 24b = 1536b ≈ 3 sd blocks per write. small enough
    // that latency stays bounded, large enough that fatfs per-call cost
    // doesn't dominate. at 100hz a batch of 64 represents 640ms of
    // data, so we'll usually write in much smaller chunks because the
    // rb drains as fast as we can pop. the 64-cap mostly matters
    // *after* a stall, when there's a backlog to clear in one go.
    sample_t batch[64];
    uint32_t last_stats = HAL_GetTick();

    for (;;) {
        size_t n = 0;
        while (n < 64 && rb_pop(&rb, &batch[n])) n++;

        if (n) {
            UINT bw;
            // this is where the 50-200ms stalls live. when it blocks,
            // the isr keeps pushing without us. that's the whole point.
            // nothing to do here but wait it out.
            f_write(&log_file, batch, n * sizeof(sample_t), &bw);
        }

        uint32_t now = HAL_GetTick();
        if (now - last_stats >= 100) {
            last_stats = now;
            emit_stats(now);
        }
    }
}
