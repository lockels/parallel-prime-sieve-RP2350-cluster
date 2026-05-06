#include <hardware/i2c.h>
#include <pico/cyw43_arch.h>
#include <pico/stdio.h>

#include "sieve.h"

#define I2C_PORT i2c0
#define SDA_PIN 4
#define SCL_PIN 5

#define N_SLAVES 3
#define N_NODES (N_SLAVES + 1)

#define MAX_POLL_ATTEMPTS 2000

static const uint8_t slave_addr[N_SLAVES] = {0x10, 0x11, 0x12};

static bool send_task(uint8_t addr, uint32_t range_lo, uint32_t range_hi,
                      uint32_t sqrt_n) {
    uint8_t buf[13];
    buf[0] = 0x01;
    memcpy(&buf[1], &range_lo, 4);
    memcpy(&buf[5], &range_hi, 4);
    memcpy(&buf[9], &sqrt_n, 4);

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

typedef struct {
    uint32_t count;
    double elapsed;
} result_t;

static result_t distributed_sieve(uint32_t n, uint32_t sqrt_n, uint8_t p) {
    uint32_t t0 = time_us_32();

    uint32_t dist_start = time_us_32(); // Start distribution timer

    // Distribute work
    uint32_t chunk = n / p;
    for (int i = 0; i < p - 1; i++) {
        uint32_t lo = chunk * (i + 1);
        uint32_t hi = (i != p - 2) ? lo + chunk - 1 : n - 1;

        if (!send_task(slave_addr[i], lo, hi, sqrt_n))
            printf("Slave 0x%02X [OFFLINE]\n", slave_addr[i]);
    }

    uint32_t dist_end = time_us_32(); // End distribution timer

    uint32_t comp_start = time_us_32(); // Start computation timer

    // Compute local share of work
    gen_sml_sieve(sqrt_n);
    uint32_t count = segmented_sieve(0, chunk - 1);

    uint32_t comp_end = time_us_32(); // End computation timer

    uint32_t poll_start = time_us_32(); // Start polling timer

    // Collect remote results
    for (int i = 0; i < p - 1; i++) {
        uint32_t slave_count = 0;
        uint32_t attempts = 0;

        while (!poll_result(slave_addr[i], &slave_count)) {
            sleep_ms(1);
            if (++attempts > MAX_POLL_ATTEMPTS) {
                printf("Slave 0x%02X [TIMEOUT]\n", slave_addr[i]);
                break;
            }
        }

        count += slave_count;
    }

    uint32_t poll_end = time_us_32(); // End polling timer

    double elapsed = ((double)time_us_32() - t0) / 1e6;

    double comp_time = (double)(comp_end - comp_start) / 1e6;
    double comm_time = ((double)(dist_end - dist_start) / 1e6) +
        ((double)(poll_end - poll_start) / 1e6);

    printf("comp = %.2fs, comm = %.2fs, ratio: %.2fs\n", comp_time, comm_time,
            comp_time / comm_time);

    return (result_t){count, elapsed};
}

static result_t sequential_sieve(uint32_t n, uint32_t sqrt_n) {
    uint32_t t0 = time_us_32();

    gen_sml_sieve(sqrt_n);
    uint32_t count = segmented_sieve(0, n - 1); // Compute entire range locally

    double elapsed = ((double)time_us_32() - t0) / 1e6;

    return (result_t){count, elapsed};
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

    // Time for the user to see the serial output
    sleep_ms(2000);

    while (true) {
        printf("\n---- N = %d, SQRT_N = %d ----\n", N, SQRT_N);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);

        // Run the sequential algorithm first.
        result_t seq_res = sequential_sieve(N, SQRT_N);
        printf("[SEQUENTIAL]:  %d primes (%.2fs)\n", seq_res.count,
               seq_res.elapsed);

        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);

        // Increase the number of processors p, and compute speedup
        for (int p = 2; p <= N_NODES; p++) {
            printf("\n---- N = %d, SQRT_N = %d, p = %d ----\n", N, SQRT_N, p);
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);

            result_t dis_res = distributed_sieve(N, SQRT_N, p);
            printf("[DISTRIBUTED (%d nodes)]: %d primes (%.2fs)\n", p,
                    dis_res.count, dis_res.elapsed);

            printf("Speedup: %.3lfx\n", seq_res.elapsed / dis_res.elapsed);

            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);
        }

        sleep_ms(5000);
    }
}
