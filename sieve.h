#pragma once

#include <stdint.h>

#define N 55000000
#define SQRT_N 7417

void gen_sml_sieve(uint32_t sqrt_n);

uint32_t segmented_sieve(uint32_t range_lo, uint32_t range_hi);
