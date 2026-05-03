#include <pico/cyw43_arch.h>
#include <pico/stdio.h>

#include "sieve.h"

/* ------ Memory for small primes up to sqrt(n) ------ */
#define SML_WORDS ((SQRT_N + 31) / 32)

static uint32_t sml_is_prime[SML_WORDS];
static uint16_t sml_prime[SQRT_N];
static uint32_t sml_prime_cnt;

/* ------ Memory for primes within segment [lo..hi] ------ */
#define SEG_SIZE 32768
#define SEG_WORDS (SEG_SIZE / 32)

static uint32_t seg_is_prime[SEG_WORDS];

/* ------ Utility functions ------ */
static inline uint32_t max_u32(uint32_t a, uint32_t b) { return (a > b) ? a : b; }

static inline bool get_bit(const uint32_t *buf, int i) {
    return (buf[i >> 5] >> (i & 31)) & 1u;
}

static inline void clr_bit(uint32_t *buf, int i) {
    buf[i >> 5] &= ~(1u << (i & 31));
}

/* Generates prime numbers up to sqrt(n) and stores the result in sml_prime.
 * Runs Sieve of Eratosthenes algorithm over the range [2...sqrt(n)] */
void gen_sml_sieve(void) {
    memset(sml_is_prime, 0xFF, sizeof(sml_is_prime));
    clr_bit(sml_is_prime, 0);
    clr_bit(sml_is_prime, 1);

    for (int p = 2; p * p <= SQRT_N; p++) {
        if (get_bit(sml_is_prime, p)) {
            for (int j = p * p; j <= SQRT_N; j += p)
                clr_bit(sml_is_prime, j);
        }
    }

    sml_prime_cnt = 0;
    for (int p = 2; p <= SQRT_N; p++) {
        if (get_bit(sml_is_prime, p))
            sml_prime[sml_prime_cnt++] = (uint16_t)p;
    }
}

/* Segmented Sieve of Eratosthenes. Returns the number of primes <= n. */
uint32_t segmented_sieve(uint32_t range_lo, uint32_t range_hi) {
    uint32_t count = 0;

    for (int lo = range_lo; lo <= range_hi; lo += SEG_SIZE) {
        int hi = lo + SEG_SIZE - 1;
        if (hi > range_hi)
            hi = range_hi;

        memset(seg_is_prime, 0xFF, sizeof(seg_is_prime));
        if (lo == 0) {
            clr_bit(seg_is_prime, 0);
            clr_bit(seg_is_prime, 1);
        }

        for (int p_idx = 0; p_idx < sml_prime_cnt; p_idx++) {
            int p = sml_prime[p_idx];

            int start = max_u32(p * p, ((lo + p - 1) / p) * p);
            for (int j = start; j <= hi; j += p)
                clr_bit(seg_is_prime, j - lo);
        }

        // Optimization: Instead of counting every bit individually,
        // __builtin_popcount uses hardware instructions to count the number of
        // set bits in an entire 32 bit word at once, allowing the algorithm to
        // count 32x faster
        int seg_len = hi - lo + 1;
        int full_words = seg_len / 32;
        int rem = seg_len % 32;

        for (int w = 0; w < full_words; w++)
            count += __builtin_popcount(seg_is_prime[w]);
        if (rem)
            count += __builtin_popcount(seg_is_prime[full_words] &
                                        ((1u << rem) - 1));
    }

    return count;
}
