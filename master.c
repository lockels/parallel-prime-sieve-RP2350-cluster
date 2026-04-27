#include <hardware/i2c.h>
#include <pico/cyw43_arch.h>
#include <pico/stdio.h>

#define N_TOTAL 400

#define I2C_PORT i2c0
#define SDA_PIN 4
#define SCL_PIN 5

#define N_SLAVES 3
#define N_NODES (N_SLAVES + 1)

static const uint8_t slave_addrs[N_SLAVES] = {0x10, 0x11, 0x12};

void led_set(bool on) { cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on); }

static bool send_range(uint8_t addr, uint32_t start, uint32_t end) {
    uint8_t buf[9];
    buf[0] = 0x01;
    memcpy(&buf[1], &start, 4);
    memcpy(&buf[5], &end, 4);

    int ret = i2c_write_blocking(I2C_PORT, addr, buf, 9, false);
    if (ret != 9) {
        printf("[write 0x%02X] i2c error: %d\n", addr, ret);
    }
    return ret == 9;
}

static bool poll_result(uint8_t addr, uint32_t *sum_out) {
    uint8_t buf[5] = {0};
    int ret = i2c_read_blocking(I2C_PORT, addr, buf, 5, false);
    if (ret != 5) {
        printf("[poll 0x%02X] i2c error: %d\n", addr, ret);
        return false;
    }
    if (buf[0] != 1)
        return false;
    memcpy(sum_out, &buf[1], 4);
    return true;
}

static uint32_t local_sum(uint32_t start, uint32_t end) {
    uint32_t s = 0;
    for (uint32_t i = start; i <= end; i++)
        s += i;
    return s;
}

int main() {
    stdio_init_all();

    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    if (cyw43_arch_init()) { while (true) sleep_ms(1000); }

    led_set(true);

    while (true) {
        printf("\n--- Computing Distrubted Sum (range: 1..%lu) ---\n", N_TOTAL);

        // --- Step 1) Divide Work ---
        uint32_t chunk_size = N_TOTAL / N_NODES;
        uint32_t start[N_NODES], end[N_NODES];
        for (int i = 0; i < N_NODES; i++) {
            start[i] = i * chunk_size + 1;
            end[i] = (i + 1) * chunk_size;
        }

        // --- Step 2) Distrubte Work ---
        printf("Master <- [%lu..%lu]\n", start[0], end[0]);
        bool dispatched[N_SLAVES] = {false};

        for (int i = 0; i < N_SLAVES; i++) {
            uint32_t s = start[i + 1], e = end[i + 1];
            dispatched[i] = send_range(slave_addrs[i], s, e);

            if (dispatched[i])
                printf("Slave 0x%02X <- [%lu..%lu]\n", slave_addrs[i], s, e);
            else
                printf("Slave 0x%02X OFFLINE\n", slave_addrs[i]);
        }

        // --- Step 3) Compute Master's Share of Work ---
        uint32_t total = local_sum(start[0], end[0]);
        printf("Master: sum = %lu\n", total);

        // --- Step 4) Collect Results ---
        for (int i = 0; i < N_SLAVES; i++) {
            if (!dispatched[i])
                continue;

            uint32_t slave_sum = 0;
            int attempts = 0;
            while (!poll_result(slave_addrs[i], &slave_sum)) {
                sleep_ms(5);
                if (++attempts > 200) {
                    printf("Slave 0x%02X TIMEOUT\n", slave_addrs[i]);
                    goto next_round;
                }
            }

            printf("Slave 0x%02X: sum = %lu\n", slave_addrs[i], slave_sum);
            total += slave_sum;
        }

        uint32_t expected = (N_TOTAL / 2) * (N_TOTAL + 1);
        printf("\nTotal:    %lu\n", total);
        printf("Expected: %lu\n", expected);

    next_round:
        sleep_ms(3000);
    }
}
