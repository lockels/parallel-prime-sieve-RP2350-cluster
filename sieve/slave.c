#include "pico/cyw43_arch.h"
#include <pico/i2c_slave.h>
#include <pico/stdio.h>
#include <pico/time.h>

#define I2C_PORT i2c0
#define SDA_PIN 4
#define SCL_PIN 5

#ifndef SLAVE_ADDR
#define SLAVE_ADDR 0x10
#endif

#define SQRT_N 4001
#define SEG_SIZE 32768
#define SEG_WORDS (SEG_SIZE / 32)
#define SML_WORDS ((SQRT_N + 31) / 32)

static uint32_t seg[SEG_WORDS];
static uint16_t small_primes[SQRT_N];
static int small_count = 0;

static inline bool get_bit(uint32_t *arr, int i) {
    return (arr[i >> 5] >> (i & 31)) & 1;
}

static inline void clr_bit(uint32_t *arr, int i) {
    arr[i >> 5] &= ~(1u << (i & 31));
}

typedef struct {
    uint8_t rx_buf[9];
    uint8_t rx_idx;
    uint8_t tx_buf[5];
    uint8_t tx_idx;
    volatile bool work_ready;
} ctx_t;

static ctx_t ctx;

static void slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
    case I2C_SLAVE_RECEIVE:
        if (ctx.rx_idx < sizeof(ctx.rx_buf))
            ctx.rx_buf[ctx.rx_idx++] = i2c_read_byte_raw(i2c);
        else
            i2c_read_byte_raw(i2c);
        if (ctx.rx_idx == sizeof(ctx.rx_buf))
            ctx.work_ready = true;
        break;
    case I2C_SLAVE_REQUEST:
        i2c_write_byte_raw(i2c, ctx.tx_idx < sizeof(ctx.tx_buf)
                                    ? ctx.tx_buf[ctx.tx_idx++]
                                    : 0xFF);
        break;
    case I2C_SLAVE_FINISH:
        if (ctx.rx_idx == sizeof(ctx.rx_buf))
            ctx.tx_buf[0] = 0;  // busy — new job incoming
        ctx.rx_idx = 0;
        ctx.tx_idx = 0;
        break;
    }
}

static void build_small_primes(void) {
    static uint32_t sml[SML_WORDS];

    memset(sml, 0xFF, sizeof(sml));
    clr_bit(sml, 0);
    clr_bit(sml, 1);

    for (int i = 2; i * i < SQRT_N; i++) {
        if (get_bit(sml, i)) {
            for (int j = i * i; j < SQRT_N; j += i) {
                clr_bit(sml, j);
            }
        }
    }

    small_count = 0;
    for (int i = 2; i < SQRT_N; i++) {
        if (get_bit(sml, i))
            small_primes[small_count++] = (uint16_t)i;
    }
}

static uint32_t segmented_sieve(int lo, int hi) {
    build_small_primes();

    uint32_t count = 0;
    for (int base = lo; base <= hi; base += SEG_SIZE) {
        int seg_hi = base + SEG_SIZE - 1;
        if (seg_hi > hi) seg_hi = hi;

        memset(seg, 0xFF, sizeof(seg));
        if (base == 0) { clr_bit(seg, 0); clr_bit(seg, 1); }

        for (int pi = 0; pi < small_count; pi++) {
            int p     = small_primes[pi];
            int start = (base <= p * p) ? p * p : ((base + p - 1) / p) * p;
            for (int j = start; j <= seg_hi; j += p)
                clr_bit(seg, j - base);
        }

        for (int i = 0; i <= seg_hi - base; i++)
            if (get_bit(seg, i))
                count++;
    }

    return count;
}

int main() {
    stdio_init_all();

    if (cyw43_arch_init()) {
        while (true) sleep_ms(1000);
    }

    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    build_small_primes();
    ctx.tx_buf[0] = 0;
    i2c_slave_init(I2C_PORT, SLAVE_ADDR, &slave_handler);

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);

    while (true) {
        if (ctx.work_ready) {
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);
            ctx.work_ready = false;

            uint32_t lo, hi;
            memcpy(&lo, &ctx.rx_buf[1], 4);
            memcpy(&hi, &ctx.rx_buf[5], 4);

            uint32_t count = segmented_sieve((int)lo, (int)hi);

            ctx.tx_buf[0] = 1;
            memcpy(&ctx.tx_buf[1], &count, 4);
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);
        }
    }
}
