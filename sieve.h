#pragma once

#include <stdint.h>

/* ------------------ N = 64,000,000 => 3,785,086 primes ------------------ */
#define N 64000000
#define SQRT_N 8000

void gen_sml_sieve(uint32_t sqrt_n);

uint32_t segmented_sieve(uint32_t range_lo, uint32_t range_hi);
