#pragma once

#include <stdint.h>

/* ------------------ N = 49,000,000 => 2,944,531 primes ------------------ */
#define N 49000000
#define SQRT_N (7000 + 1)

void gen_sml_sieve(void);

uint32_t segmented_sieve(uint32_t range_lo, uint32_t range_hi);
