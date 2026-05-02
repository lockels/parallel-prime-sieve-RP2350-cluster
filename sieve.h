#pragma once

#include <stdint.h>

/* ------ N = 36,000,000 => 2,204,262 primes ------ */

#define N 36000000
#define SQRT_N (6000 + 1)

void gen_sml_sieve(void);

uint32_t segmented_sieve(uint32_t range_lo, uint32_t range_hi);
