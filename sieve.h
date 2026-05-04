#pragma once

#include <stdint.h>

#define MAX_N 51000000
#define MAX_SQRT_N 7142

void gen_sml_sieve(uint32_t sqrt_n);

uint32_t segmented_sieve(uint32_t range_lo, uint32_t range_hi);
