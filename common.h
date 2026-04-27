#ifndef COMMON_H
#define COMMON_H

#include <pico/cyw43_arch.h>
#include <pico/time.h>

void led_set(bool on);

void led_blink(int times, int on_ms, int off_ms);

uint32_t xorshift32(uint32_t *state);

long monte_carlo_hits(uint32_t trials, uint32_t seed);

#endif
