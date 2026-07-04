/**
 * @file    test_timebase.c
 * @brief   Host tests for fusion/timebase.c.
 */

#include "fusion/timebase.h"

#include <stdio.h>

static int s_failures = 0;

#define CHECK_EQ_MS(desc, actual, expected, tol_ms)                         \
    do {                                                                    \
        long diff = (long) (actual) - (long) (expected);                   \
        if (diff < 0) {                                                    \
            diff = -diff;                                                  \
        }                                                                   \
        if (diff > (tol_ms)) {                                             \
            fprintf(stderr,                                                \
                    "FAIL: %s: got %lu ms, expected %lu ms (tol %ld)\n",    \
                    (desc), (unsigned long) (actual),                      \
                    (unsigned long) (expected), (long) (tol_ms));          \
            s_failures++;                                                   \
        }                                                                    \
    } while (0)

/* Two PPS observations 1000 ms apart with a known 1,000,000-tick delta
 * (i.e. a 1 MHz / 1 us tick, matching the documented tick unit) - check
 * that interpolation between them, and extrapolation just beyond the
 * second, both track the measured rate to well within 1 ms. */
static void
test_basic_interpolation(void)
{
    timebase_init();

    timebase_on_pps(1000000U, 500000U);
    timebase_on_pps(2000000U, 501000U);

    uint32_t mid = timebase_tick_to_itow_ms(1500000U);
    CHECK_EQ_MS("midpoint interpolation", mid, 500500U, 1U);

    uint32_t past_last = timebase_tick_to_itow_ms(2500000U);
    CHECK_EQ_MS("forward extrapolation", past_last, 501500U, 1U);

    uint32_t at_first_obs = timebase_tick_to_itow_ms(1000000U);
    CHECK_EQ_MS("re-query at first observation's tick", at_first_obs,
                500000U, 1U);
}

/* A PPS pair straddling the GPS week rollover (iTOW resets to 0 at the
 * week boundary): the affine mapping must treat this as the small
 * forward step it physically is, not a ~604,800,000 ms jump. */
static void
test_week_rollover(void)
{
    timebase_init();

    uint32_t itow_before_wrap = 604799500U; /* 500 ms before week end */
    uint32_t itow_after_wrap = 300U;        /* 300 ms into the new week */

    timebase_on_pps(1000000U, itow_before_wrap);
    /* 800 ms later at the same 1000 ticks/ms rate as above. */
    timebase_on_pps(1800000U, itow_after_wrap);

    /* 400 ms further on: 300 + 400 = 700 ms into the new week. */
    uint32_t result = timebase_tick_to_itow_ms(2200000U);
    CHECK_EQ_MS("post-rollover extrapolation", result, 700U, 1U);
}

/* With only a single PPS observation, tick_to_itow_ms should still give a
 * sane estimate using the documented nominal (1 tick = 1 us) rate. */
static void
test_single_observation_fallback(void)
{
    timebase_init();

    timebase_on_pps(5000000U, 100000U);

    /* 1,000,000 ticks later == 1000 ms later at the nominal rate. */
    uint32_t result = timebase_tick_to_itow_ms(6000000U);
    CHECK_EQ_MS("single-observation fallback", result, 101000U, 1U);
}

/* timebase_itow_ms_to_tick() must be the exact inverse of
 * timebase_tick_to_itow_ms() over the disciplined mapping - round-trip
 * a handful of tick values through both directions. */
static void
test_itow_to_tick_is_inverse(void)
{
    timebase_init();

    timebase_on_pps(1000000U, 500000U);
    timebase_on_pps(2000000U, 501000U);

    uint32_t ticks[] = {1000000U, 1250000U, 1500000U, 1999999U, 2500000U};
    for (size_t i = 0; i < sizeof(ticks) / sizeof(ticks[0]); i++) {
        uint32_t itow = timebase_tick_to_itow_ms(ticks[i]);
        uint32_t back = timebase_itow_ms_to_tick(itow);
        long diff = (long) back - (long) ticks[i];
        if (diff < 0) {
            diff = -diff;
        }
        if (diff > 1000) { /* within 1 ms at the 1000 ticks/ms rate */
            fprintf(stderr,
                    "FAIL: itow->tick roundtrip[%zu]: tick %u -> itow %u "
                    "-> tick %u\n",
                    i, ticks[i], itow, back);
            s_failures++;
        }
    }
}

int
main(void)
{
    test_basic_interpolation();
    test_week_rollover();
    test_single_observation_fallback();
    test_itow_to_tick_is_inverse();

    if (s_failures == 0) {
        printf("test_timebase: all tests passed\n");
        return 0;
    }

    fprintf(stderr, "test_timebase: %d failure(s)\n", s_failures);
    return 1;
}
