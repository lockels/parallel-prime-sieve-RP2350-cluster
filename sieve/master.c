#include "pico/cyw43_arch.h"
#include <hardware/i2c.h>
#include <pico/stdio.h>

#define I2C_PORT i2c0
#define SDA_PIN 4
#define SCL_PIN 5

#define N_SLAVES 3
#define N_NODES (N_SLAVES + 1)

static const uint8_t slave_addr[N_SLAVES] = {0x10, 0x11, 0x12};

static bool send_task(uint8_t addr, uint32_t lo, uint32_t hi) {
    uint8_t buf[9];
    buf[0] = 0x01;
    memcpy(&buf[1], &lo, 4);
    memcpy(&buf[5], &hi, 4);

    int ret = i2c_write_blocking(I2C_PORT, addr, buf, sizeof(buf), false);
    if (ret != sizeof(buf))
        printf("[write 0x%02X i2c error: %d\n]", addr, ret);

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

#define N 16000000 // 1031130
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

typedef struct {
    double count;
    double time;
} run_result_t;

static run_result_t run_distributed(void) {
    uint32_t start = time_us_64();

    uint32_t chunk = N / N_NODES;

    // 1) Distribute Work
    for (int i = 0; i < N_SLAVES; i++) {
        uint32_t lo = chunk * (i + 1);
        uint32_t hi = (i == N_SLAVES - 1) ? N - 1 : lo + chunk - 1;

        if (!send_task(slave_addr[i], lo, hi))
            printf("Slave 0x%02X [OFFLINE]\n", slave_addr[i]);
    }

    // 2) Compute Local work
    uint32_t count = segmented_sieve(0, chunk - 1);

    // 3) Collect remote results
    for (int i = 0; i < N_SLAVES; i++) {
        uint32_t slave_count = 0, attempts = 0;

        while (!poll_result(slave_addr[i], &slave_count)) {
            if (++attempts > 2000) {
                printf("Slave 0x%02X [TIMEOUT]\n", slave_addr[i]);
                break;
            }
        }

        count += slave_count;
    }

    double elapsed = ((double)time_us_64() - start) / 1e6;
    return (run_result_t){count, elapsed};
}

static run_result_t run_sequential(void) {
    uint32_t start = time_us_64();

    uint32_t count = segmented_sieve(0, N - 1);

    double elapsed = ((double)time_us_64() - start) / 1e6;

    return (run_result_t){count, elapsed};
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

    build_small_primes();

    // ---------------- N: 16000000 ----------------
    // [SEQUENTIAL]:  1031130 primes (3.722s)
    // [DISTRIBUTED]: 1031130 primes (0.952s)
    // Speedup: 3.908x

    while (true) {
        printf("\n---------------- N: %d ----------------\n", N);

        run_result_t seq = run_sequential();
        printf("[SEQUENTIAL]:  %lu primes (%.3fs)\n", (unsigned long)seq.count,
               seq.time);

        run_result_t dis = run_distributed();
        printf("[DISTRIBUTED]: %lu primes (%.3fs)\n", (unsigned long)dis.count,
               dis.time);

        printf("Speedup: %.3fx\n", seq.time / dis.time);

        sleep_ms(5000);
    }
}
