#include "pico/cyw43_arch.h"
#include <pico/i2c_slave.h>
#include <pico/stdio.h>

#include "sieve.h"

#define I2C_PORT i2c0
#define SDA_PIN 4
#define SCL_PIN 5

#ifndef SLAVE_ADDR
#define SLAVE_ADDR 0x10
#endif

typedef struct {
    uint8_t rx_buf[13];
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
            ctx.tx_buf[0] = 0; // busy — new job incoming
        ctx.rx_idx = 0;
        ctx.tx_idx = 0;
        break;
    }
}

int main() {
    stdio_init_all();

    if (cyw43_arch_init()) {
        while (true)
            sleep_ms(1000);
    }

    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    ctx.tx_buf[0] = 0;
    i2c_slave_init(I2C_PORT, SLAVE_ADDR, &slave_handler);

    while (true) {
        if (ctx.work_ready) {
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);
            ctx.work_ready = false;

            uint32_t range_lo, range_hi, sqrt_n;
            memcpy(&range_lo, &ctx.rx_buf[1], 4);
            memcpy(&range_hi, &ctx.rx_buf[5], 4);
            memcpy(&sqrt_n, &ctx.rx_buf[9], 4);

            gen_sml_sieve(sqrt_n);
            uint32_t count = segmented_sieve(range_lo, range_hi);

            ctx.tx_buf[0] = 1;
            memcpy(&ctx.tx_buf[1], &count, 4);
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);
        }
    }
}
