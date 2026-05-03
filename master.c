#include <hardware/i2c.h>
#include <pico/cyw43_arch.h>
#include <pico/stdio.h>

#include "sieve.h"

#define I2C_PORT i2c0
#define SDA_PIN 4
#define SCL_PIN 5

#define N_SLAVES 3
#define N_NODES (N_SLAVES + 1)

static const uint8_t slave_addr[N_SLAVES] = {0x10, 0x11, 0x12};

static bool send_task(uint8_t addr, uint32_t range_lo, uint32_t range_hi) {
    uint8_t buf[9];
    buf[0] = 0x01;
    memcpy(&buf[1], &range_lo, 4);
    memcpy(&buf[5], &range_hi, 4);

    int ret = i2c_write_blocking(I2C_PORT, addr, buf, sizeof(buf), false);
    if (ret != sizeof(buf))
        printf("[write 0x%02X] i2c error: %d\n", addr, ret);

    return ret == sizeof(buf);
}

static bool poll_result(uint8_t addr, uint32_t *count_out) {
    uint8_t buf[5] = {0};

    int ret = i2c_read_blocking(I2C_PORT, addr, buf, sizeof(buf), false);
    if (ret != sizeof(buf)) {
        printf("[poll 0x%02X] i2c error: %d", addr, ret);
        return false;
    }

    if (buf[0] != 1)
        return false;

    memcpy(count_out, &buf[1], 4);
    return true;
}



static double distributed_sieve(void) {
    uint32_t start = time_us_32();

    // Distribute work
    uint32_t chunk = N / N_NODES;
    for (int i = 0; i < N_SLAVES; i++) {
        uint32_t lo = chunk * (i + 1);
        uint32_t hi = (i != N_SLAVES - 1) ? lo + chunk - 1 : N - 1;

        if (!send_task(slave_addr[i], lo, hi))
            printf("Slave 0x%02X [OFFLINE]\n", slave_addr[i]);
    }

    // Compute local share of work
    uint32_t count = segmented_sieve(0, chunk - 1);

    // Collect remote results
    for (int i = 0; i < N_SLAVES; i++) {
        uint32_t slave_count = 0;
        uint32_t attempts = 0;

        while (!poll_result(slave_addr[i], &slave_count)) {
            if (++attempts > 2000) {
                printf("Slave 0x%02X [TIMEOUT]\n", slave_addr[i]);
                break;
            }
        }

        count += slave_count;
    }

    uint32_t end = time_us_32();

    double elapsed = ((double)end - start) / 1e6;
    printf("[DISTRIBUTED]: %lu primes (%.3fs)\n", (unsigned long)count,
           elapsed);

    return elapsed;
}

static double sequential_sieve(void) {
    uint32_t start = time_us_32();

    // Execute entire range locally
    uint32_t count = segmented_sieve(0, N - 1);

    uint32_t end = time_us_32();

    double elapsed = ((double)end - start) / 1e6;
    printf("[SEQUENTIAL]:  %lu primes (%.3fs)\n", (unsigned long)count,
           elapsed);

    return elapsed;
}

int main() {
    stdio_init_all();

    if (cyw43_arch_init()) {
        while (true) {
            sleep_ms(1000);
        }
    }

    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);

    gen_sml_sieve(SQRT_N);

    while (true) {
        printf("\n---------------- N: %d ----------------\n", N);

        double seq_time = sequential_sieve();

        double dis_time = distributed_sieve();

        printf("Speedup: %.3fx\n", seq_time / dis_time);

        sleep_ms(2000);
    }
}
