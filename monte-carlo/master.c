#include <hardware/i2c.h>
#include <pico/cyw43_arch.h>
#include <pico/stdio.h>

#define N_TRIALS 20000000

#define I2C_PORT i2c0
#define SDA_PIN 4
#define SCL_PIN 5

#define N_SLAVES 3
#define N_NODES (N_SLAVES + 1)

static const uint8_t slave_addr[N_SLAVES] = {0x10, 0x11, 0x12};
static const uint32_t master_seed = 0xDEADBEEF;
static const uint32_t slave_seed[N_SLAVES] = {0xCAFEBABE, 0xBAADF00D,
                                              0xFEEDFACE};
typedef struct {
    double elapsed;
    uint32_t hits;
} run_result_t;

void led_set(bool on) { cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on); }

static bool send_task(uint8_t addr, uint32_t trials, uint32_t seed) {
    uint8_t buf[9];
    buf[0] = 0x01;
    memcpy(&buf[1], &trials, 4);
    memcpy(&buf[5], &seed, 4);

    int ret = i2c_write_blocking(I2C_PORT, addr, buf, sizeof(buf), false);
    if (ret != sizeof(buf)) {
        printf("[write 0x%02X] i2c error: %d\n", addr, ret);
    }

    return ret == sizeof(buf);
}

static bool poll_result(uint8_t addr, uint32_t *sum_out) {
    uint8_t buf[5] = {0};

    int ret = i2c_read_blocking(I2C_PORT, addr, buf, sizeof(buf), false);
    if (ret != sizeof(buf)) {
        printf("[poll 0x%02X] i2c error: %d\n", addr, ret);
        return false;
    }

    if (buf[0] != 1) {
        return false;
    }

    memcpy(sum_out, &buf[1], 4);
    return true;
}

uint32_t xorshift32(uint32_t *state) {
    uint32_t x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return x;
}

uint32_t monte_carlo_hits(uint32_t trials, uint32_t seed) {
    uint32_t state = seed;
    uint32_t hits = 0;

    for (uint32_t i = 0; i < trials; i++) {
        float x = (float)xorshift32(&state) / (float)UINT32_MAX;
        float y = (float)xorshift32(&state) / (float)UINT32_MAX;

        if (x * x + y * y <= 1.0f) {
            hits++;
        }
    }

    return hits;
}

run_result_t run_sequential() {
    uint32_t start = time_us_64();

    uint32_t hits = monte_carlo_hits(N_TRIALS, master_seed);

    double elapsed = (double)(time_us_64() - start) / 1e6;

    return (run_result_t){elapsed, hits};
}

run_result_t run_distrubuted() {
    uint32_t start = time_us_64();

    // 1) Distrbute work
    uint32_t chunk = N_TRIALS / N_NODES;
    for (int i = 0; i < N_SLAVES; i++) {
        if (!send_task(slave_addr[i], chunk, slave_seed[i])) {
            printf("Slave 0x%02X [OFFLINE]\n", slave_addr[i]);
        }
    }

    // 2) Compute local share of work
    uint32_t hits = monte_carlo_hits(chunk, master_seed);

    // 3) Accumulate results.
    for (int i = 0; i < N_SLAVES; i++) {
        uint32_t slave_hits = 0, attempts = 0;

        while (!poll_result(slave_addr[i], &slave_hits)) {
            if (++attempts > 2000) {
                printf("Slave 0x%02X [TIMEOUT]\n", slave_addr[i]);
                break;
            }
        }

        hits += slave_hits;
    }

    double elapsed = (double)(time_us_64() - start) / 1e6;

    return (run_result_t){elapsed, hits};
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

    // [SEQUENTIAL]:  Time = 4.133s, Pi = 3.141
    // [DISTRIBUTED]: Time = 1.105s, Pi = 3.142
    // Speedup:       3.741x

    led_set(true);

    while (true) {
        printf("\n---------------- N_TRIALS: %d ----------------\n", N_TRIALS);

        run_result_t seq_result = run_sequential();
        double seq_pi = 4.0 * ((double)seq_result.hits / (double)N_TRIALS);
        printf("[SEQUENTIAL]:  Time = %.3lfs, Pi = %.6lf\n",
               seq_result.elapsed, seq_pi);

        run_result_t dis_result = run_distrubuted();
        double dis_pi = 4.0 * ((double)dis_result.hits / (double)N_TRIALS);
        printf("[DISTRIBUTED]: Time = %.3lfs, Pi = %.6lf\n",
               dis_result.elapsed, dis_pi);

        double speedup = seq_result.elapsed / dis_result.elapsed;
        printf("Speedup:       %.3lfx\n", speedup);

        sleep_ms(5000);
    }
}
