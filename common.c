#include "common.h"

void led_set(bool on) { cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on); }

void led_blink(int times, int on_ms, int off_ms) {
  for (int i = 0; i < times; i++) {
    led_set(true);
    sleep_ms(on_ms);
    led_set(false);
    sleep_ms(off_ms);
  }
}

uint32_t xorshift32(uint32_t *state) {
  uint32_t x = *state;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  *state = x;
  return x;
}

long monte_carlo_hits(uint32_t trials, uint32_t seed) {
  uint32_t state = seed;
  long hits = 0;
  for (uint32_t i = 0; i < trials; i++) {
    double x = (double)xorshift32(&state) / (double)UINT32_MAX;
    double y = (double)xorshift32(&state) / (double)UINT32_MAX;
    if (x * x + y * y <= 1.0f) {
      hits++;
    }
  }
  return hits;
}
