/**
 * @file    test_laptimer.c
 * @brief   SIL-style replay test for laptimer.c: drive a synthetic closed
 *          loop through laptimer_update() at a fixed sample rate and
 *          check reported lap times against the known ground truth.
 */

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "board/board_config.h"
#include "laptimer/gates.h"
#include "laptimer/laptimer.h"

/* No <math.h>/libm here on purpose (the host test toolchain doesn't link
 * libm) - this test needs a real running sin/cos to synthesise the
 * circular trajectory, so it carries the same small libm-free
 * approximation gates.c uses internally, rather than special-casing
 * fixed angles the way test_gates.c does. */
#define PI_F 3.14159265358979323846f
#define TWO_PI_F (2.0f * PI_F)

static float test_absf(float x)
{
    return (x < 0.0f) ? -x : x;
}

static void test_sincosf(float x, float *sin_out, float *cos_out)
{
    int32_t k = (int32_t) (x * (1.0f / TWO_PI_F));
    float r = x - (float) k * TWO_PI_F;

    if (r > PI_F) {
        r -= TWO_PI_F;
    } else if (r <= -PI_F) {
        r += TWO_PI_F;
    }

    float sign = 1.0f;
    float ar = r;
    if (ar < 0.0f) {
        ar = -ar;
        sign = -1.0f;
    }

    float cos_sign = 1.0f;
    if (ar > PI_F * 0.5f) {
        ar = PI_F - ar;
        cos_sign = -1.0f;
    }

    float ar2 = ar * ar;

    float s = ar
              * (1.0f
                 + ar2
                       * (-1.0f / 6.0f
                          + ar2
                                * (1.0f / 120.0f
                                   + ar2
                                         * (-1.0f / 5040.0f
                                            + ar2 * (1.0f / 362880.0f)))));

    float c = 1.0f
              + ar2
                    * (-1.0f / 2.0f
                       + ar2
                             * (1.0f / 24.0f
                                + ar2
                                      * (-1.0f / 720.0f
                                         + ar2 * (1.0f / 40320.0f))));

    *sin_out = sign * s;
    *cos_out = cos_sign * c;
}

/* Circular trajectory, traversed counter-clockwise at constant speed:
 *   east(t)  = R * cos(omega * t)
 *   north(t) = R * sin(omega * t)
 * so the tangent (velocity) direction at angle theta is
 * (-sin(theta), cos(theta)); at theta = 0 that is due north (+y), which
 * is where the start/finish gate is placed below. */
#define CIRCLE_RADIUS_M   50.0f
#define SPEED_MPS         20.0f
#define SAMPLE_RATE_HZ    104.0f
#define SAMPLE_DT_S       (1.0f / SAMPLE_RATE_HZ)

static float ground_truth_lap_ms(void)
{
    float circumference = 2.0f * PI_F * CIRCLE_RADIUS_M;
    return (circumference / SPEED_MPS) * 1000.0f;
}

static void test_circle_replay_lap_times(void)
{
    laptimer_init();
    gates_init();

    /* Start/finish at theta = 0: point (R, 0), heading = tangent there
     * (due north, +y => heading = +90 deg). */
    assert(gates_set(0U, CIRCLE_RADIUS_M, 0.0f, PI_F / 2.0f) == STATUS_OK);

    float omega = SPEED_MPS / CIRCLE_RADIUS_M; /* rad/s */
    float lap_ms_truth = ground_truth_lap_ms();

    /* Run for a bit over 3 laps so we get several completed-lap events
     * plus one still in progress. */
    float total_time_s = (lap_ms_truth / 1000.0f) * 3.4f;
    uint32_t nsamples
        = (uint32_t) (total_time_s / SAMPLE_DT_S) + 1U;

    /* Arbitrary start-of-week offset, far from the itow rollover so this
     * test only exercises steady-state timing (rollover is covered
     * separately below). */
    uint32_t itow0_ms = 100000U;

    float prev_east = CIRCLE_RADIUS_M;
    float prev_north = 0.0f;
    uint32_t prev_itow_ms = itow0_ms;

    uint16_t prev_lap_count = 0U;
    uint32_t observed_laps_ms[8];
    uint16_t observed_count = 0U;

    for (uint32_t i = 1U; i <= nsamples; i++) {
        float t = (float) i * SAMPLE_DT_S;
        float theta = omega * t;
        float sin_t, cos_t;
        test_sincosf(theta, &sin_t, &cos_t);
        float east = CIRCLE_RADIUS_M * cos_t;
        float north = CIRCLE_RADIUS_M * sin_t;
        uint32_t itow_ms
            = itow0_ms + (uint32_t) (t * 1000.0f + 0.5f);

        laptimer_update(east, north, prev_east, prev_north, itow_ms,
                         prev_itow_ms);

        uint16_t lap_count = laptimer_get_lap_count();
        if (lap_count != prev_lap_count) {
            assert(observed_count < 8U);
            observed_laps_ms[observed_count++] = laptimer_get_last_lap_ms();
            prev_lap_count = lap_count;
        }

        prev_east = east;
        prev_north = north;
        prev_itow_ms = itow_ms;
    }

    /* At 20 m/s around a 50 m-radius circle, one lap takes ~15.7 s, well
     * over LAP_MIN_LAP_TIME_MS, so the debounce never swallows a real
     * lap here. */
    assert(observed_count == 3U);

    float tolerance_ms = 30.0f; /* driven by the ~9.6 ms sample period */
    for (uint16_t i = 0U; i < observed_count; i++) {
        float err = test_absf((float) observed_laps_ms[i] - lap_ms_truth);
        printf("lap %u: %u ms (truth %.1f ms, err %.2f ms)\n", i + 1U,
               observed_laps_ms[i], (double) lap_ms_truth, (double) err);
        assert(err < tolerance_ms);
    }

    assert(laptimer_is_running());
    /* A lap is in progress: elapsed must be less than one full lap. */
    uint32_t elapsed = laptimer_get_current_elapsed_ms(prev_itow_ms);
    assert(elapsed < (uint32_t) lap_ms_truth);
}

static void test_sector_gate_within_lap(void)
{
    /* Same circle, but with a sector gate at the opposite side (theta =
     * pi) in addition to the start/finish at theta = 0. Check the
     * sector completes partway through the lap without disturbing the
     * lap clock/count. */
    laptimer_init();
    gates_init();

    assert(gates_set(0U, CIRCLE_RADIUS_M, 0.0f, PI_F / 2.0f) == STATUS_OK);
    assert(gates_set(1U, -CIRCLE_RADIUS_M, 0.0f, -PI_F / 2.0f) == STATUS_OK);

    float omega = SPEED_MPS / CIRCLE_RADIUS_M;
    float lap_ms_truth = ground_truth_lap_ms();
    /* Stop at 0.7 lap: past the sector gate at the half-way point, but
     * before the start/finish gate would complete the lap - this test
     * only checks the sector, not lap completion. */
    float total_time_s = (lap_ms_truth / 1000.0f) * 0.7f;
    uint32_t nsamples = (uint32_t) (total_time_s / SAMPLE_DT_S) + 1U;
    uint32_t itow0_ms = 0U;

    float prev_east = CIRCLE_RADIUS_M;
    float prev_north = 0.0f;
    uint32_t prev_itow_ms = itow0_ms;
    bool saw_sector = false;

    for (uint32_t i = 1U; i <= nsamples; i++) {
        float t = (float) i * SAMPLE_DT_S;
        float theta = omega * t;
        float sin_t, cos_t;
        test_sincosf(theta, &sin_t, &cos_t);
        float east = CIRCLE_RADIUS_M * cos_t;
        float north = CIRCLE_RADIUS_M * sin_t;
        uint32_t itow_ms = itow0_ms + (uint32_t) (t * 1000.0f + 0.5f);

        laptimer_update(east, north, prev_east, prev_north, itow_ms,
                         prev_itow_ms);

        if (laptimer_get_current_sector() == 1U) {
            saw_sector = true;
        }

        prev_east = east;
        prev_north = north;
        prev_itow_ms = itow_ms;
    }

    assert(saw_sector);
    /* The sector gate must not have completed a lap on its own. */
    assert(laptimer_get_lap_count() == 0U);

    float half_lap_ms = lap_ms_truth / 2.0f;
    float err = test_absf((float) laptimer_get_last_sector_ms() - half_lap_ms);
    printf("sector: %u ms (truth %.1f ms, err %.2f ms)\n",
           laptimer_get_last_sector_ms(), (double) half_lap_ms,
           (double) err);
    assert(err < 30.0f);
}

static void test_itow_rollover_does_not_spike_elapsed(void)
{
    /* Regression for week-rollover handling: start a lap right at the
     * boundary (crossing interpolated to land at itow 0) and confirm
     * querying elapsed time shortly after does not see the huge bogus
     * value a naive (now - start) subtraction would produce. */
    laptimer_init();
    gates_init();

    assert(gates_set(0U, 0.0f, 0.0f, 0.0f) == STATUS_OK);

    uint32_t week_ms = 604800000U;
    uint32_t prev_itow_ms = week_ms - 5U;
    uint32_t itow_ms = 5U; /* wrapped: 10 ms after prev_itow_ms */

    laptimer_update(1.0f, 0.0f, -1.0f, 0.0f, itow_ms, prev_itow_ms);
    assert(laptimer_is_running());

    uint32_t elapsed = laptimer_get_current_elapsed_ms(5000U);
    printf("post-rollover elapsed: %u ms\n", elapsed);

    /* Should be roughly 5000 ms minus the few ms consumed before the
     * interpolated crossing landed at ~itow 0, never anywhere near the
     * ~4.29e9 ms a raw unsigned underflow would produce. */
    assert(elapsed < 5100U);
}

static void test_final_sector_timed_on_lap(void)
{
    /* Sector gate a quarter-lap in (theta = pi/2), so the final sector
     * (sector gate -> finish line, 3/4 of the lap) is clearly longer than
     * sector 0 (start -> sector gate, 1/4 lap). The final sector is closed
     * by the finish-line crossing; regression: at lap completion its time
     * must be the ~3/4-lap value, not sector 0's stale ~1/4-lap value. */
    laptimer_init();
    gates_init();

    assert(gates_set(0U, CIRCLE_RADIUS_M, 0.0f, PI_F / 2.0f) == STATUS_OK);
    /* theta = pi/2: position (0, R), tangent heading atan2(cos,-sin) = pi. */
    assert(gates_set(1U, 0.0f, CIRCLE_RADIUS_M, PI_F) == STATUS_OK);

    float omega = SPEED_MPS / CIRCLE_RADIUS_M;
    float lap_ms_truth = ground_truth_lap_ms();
    float total_time_s = (lap_ms_truth / 1000.0f) * 1.1f; /* just past one lap */
    uint32_t nsamples = (uint32_t) (total_time_s / SAMPLE_DT_S) + 1U;
    uint32_t itow0_ms = 100000U;

    float prev_east = CIRCLE_RADIUS_M;
    float prev_north = 0.0f;
    uint32_t prev_itow_ms = itow0_ms;
    uint16_t prev_lap_count = 0U;
    uint32_t final_sector_ms = 0U;
    bool saw_lap = false;

    for (uint32_t i = 1U; i <= nsamples; i++) {
        float t = (float) i * SAMPLE_DT_S;
        float theta = omega * t;
        float sin_t, cos_t;
        test_sincosf(theta, &sin_t, &cos_t);
        float east = CIRCLE_RADIUS_M * cos_t;
        float north = CIRCLE_RADIUS_M * sin_t;
        uint32_t itow_ms = itow0_ms + (uint32_t) (t * 1000.0f + 0.5f);

        laptimer_update(east, north, prev_east, prev_north, itow_ms,
                         prev_itow_ms);

        uint16_t lap_count = laptimer_get_lap_count();
        if (lap_count != prev_lap_count) {
            /* Sector time as it stands the instant the lap completes. */
            final_sector_ms = laptimer_get_last_sector_ms();
            saw_lap = true;
            prev_lap_count = lap_count;
        }

        prev_east = east;
        prev_north = north;
        prev_itow_ms = itow_ms;
    }

    assert(saw_lap);
    float three_quarter_ms = lap_ms_truth * 0.75f;
    float err = test_absf((float) final_sector_ms - three_quarter_ms);
    printf("final sector: %u ms (truth %.1f ms, err %.2f ms)\n",
           final_sector_ms, (double) three_quarter_ms, (double) err);
    assert(err < 30.0f);
}

/* Drive the standard circle for `laps` laps from the start/finish line,
 * calling `on_sample` (if non-NULL) with laptimer_update()'s event mask
 * for every sample. Leaves the gates as the caller set them. */
static void drive_circle(float laps, void (*on_sample)(uint8_t events))
{
    float omega = SPEED_MPS / CIRCLE_RADIUS_M;
    float total_time_s = (ground_truth_lap_ms() / 1000.0f) * laps;
    uint32_t nsamples = (uint32_t) (total_time_s / SAMPLE_DT_S) + 1U;
    uint32_t itow0_ms = 200000U;

    float prev_east = CIRCLE_RADIUS_M;
    float prev_north = 0.0f;
    uint32_t prev_itow_ms = itow0_ms;

    for (uint32_t i = 1U; i <= nsamples; i++) {
        float t = (float) i * SAMPLE_DT_S;
        float sin_t, cos_t;
        test_sincosf(omega * t, &sin_t, &cos_t);
        float east = CIRCLE_RADIUS_M * cos_t;
        float north = CIRCLE_RADIUS_M * sin_t;
        uint32_t itow_ms = itow0_ms + (uint32_t) (t * 1000.0f + 0.5f);

        uint8_t events = laptimer_update(east, north, prev_east, prev_north,
                                          itow_ms, prev_itow_ms);
        if (on_sample != NULL) {
            on_sample(events);
        }

        prev_east = east;
        prev_north = north;
        prev_itow_ms = itow_ms;
    }
}

static uint32_t s_lap_events;
static uint32_t s_sector_events;
static uint32_t s_sector_events_with_lap; /* SECTOR and LAP in one sample */

static void count_events(uint8_t events)
{
    if ((events & LAPTIMER_EVT_LAP) != 0U) {
        s_lap_events++;
    }
    if ((events & LAPTIMER_EVT_SECTOR) != 0U) {
        s_sector_events++;
        if ((events & LAPTIMER_EVT_LAP) != 0U) {
            s_sector_events_with_lap++;
        }
    }
}

static void reset_event_counts(void)
{
    s_lap_events = 0U;
    s_sector_events = 0U;
    s_sector_events_with_lap = 0U;
}

static void test_update_reports_events(void)
{
    /* One sector gate at the half-way point, 2.2 laps: the event mask
     * must report exactly 2 laps and 4 sectors - each lap's mid-lap
     * sector, plus its final sector (sector gate -> finish), which
     * completes in the same sample as the lap itself. */
    laptimer_init();
    gates_init();
    assert(gates_set(0U, CIRCLE_RADIUS_M, 0.0f, PI_F / 2.0f) == STATUS_OK);
    assert(gates_set(1U, -CIRCLE_RADIUS_M, 0.0f, -PI_F / 2.0f) == STATUS_OK);

    reset_event_counts();
    drive_circle(2.2f, count_events);

    printf("events over 2.2 laps: %u lap, %u sector (%u with a lap)\n",
           s_lap_events, s_sector_events, s_sector_events_with_lap);
    assert(s_lap_events == 2U);
    assert(s_sector_events == 4U);
    assert(s_sector_events_with_lap == 2U);
    assert(laptimer_get_lap_count() == 2U);
}

static void test_no_sector_event_without_sector_gates(void)
{
    /* With only a start/finish line the "final sector" of a lap is the
     * lap itself - completing a lap must not also report a sector. */
    laptimer_init();
    gates_init();
    assert(gates_set(0U, CIRCLE_RADIUS_M, 0.0f, PI_F / 2.0f) == STATUS_OK);

    reset_event_counts();
    drive_circle(2.2f, count_events);

    assert(s_lap_events == 2U);
    assert(s_sector_events == 0U);
}

static void test_sector_gate_counts_once_per_lap(void)
{
    /* Cross a sector gate, back up over it, and cross it again within
     * the same lap (jitter while crawling over the line does the same):
     * the second forward crossing must not register, or every later
     * sector number that lap would be off by one. */
    laptimer_init();
    gates_init();
    /* Start/finish at the origin heading east; sector gate 20 m further
     * on, same heading. */
    assert(gates_set(0U, 0.0f, 0.0f, 0.0f) == STATUS_OK);
    assert(gates_set(1U, 20.0f, 0.0f, 0.0f) == STATUS_OK);

    uint32_t t = 1000U;
    uint8_t ev;

    ev = laptimer_update(1.0f, 0.0f, -1.0f, 0.0f, t + 10U, t); /* start */
    assert(laptimer_is_running());
    assert(ev == 0U); /* starting the first lap completes nothing */

    t += 2000U;
    ev = laptimer_update(21.0f, 0.0f, 19.0f, 0.0f, t + 10U, t); /* sector */
    assert((ev & LAPTIMER_EVT_SECTOR) != 0U);
    assert(laptimer_get_current_sector() == 1U);

    t += 500U;
    ev = laptimer_update(19.0f, 0.0f, 21.0f, 0.0f, t + 10U, t); /* reverse */
    assert(ev == 0U);

    t += 500U;
    ev = laptimer_update(21.0f, 0.0f, 19.0f, 0.0f, t + 10U, t); /* again */
    assert(ev == 0U);
    assert(laptimer_get_current_sector() == 1U);

    /* A new lap re-arms the gate. */
    t += 20000U;
    ev = laptimer_update(1.0f, 0.0f, -1.0f, 0.0f, t + 10U, t); /* finish */
    assert((ev & LAPTIMER_EVT_LAP) != 0U);
    assert(laptimer_get_current_sector() == 0U);

    t += 2000U;
    ev = laptimer_update(21.0f, 0.0f, 19.0f, 0.0f, t + 10U, t);
    assert((ev & LAPTIMER_EVT_SECTOR) != 0U);
    assert(laptimer_get_current_sector() == 1U);
}

static void test_init_restarts_timing(void)
{
    /* laptimer_init() is what the firmware calls when the start/finish
     * line is moved or removed: the lap in progress and the lap count
     * must both go, and the next crossing starts a fresh first lap. */
    laptimer_init();
    gates_init();
    assert(gates_set(0U, 0.0f, 0.0f, 0.0f) == STATUS_OK);

    uint32_t t = 5000U;
    (void) laptimer_update(1.0f, 0.0f, -1.0f, 0.0f, t + 10U, t);
    t += 30000U;
    uint8_t ev = laptimer_update(1.0f, 0.0f, -1.0f, 0.0f, t + 10U, t);
    assert((ev & LAPTIMER_EVT_LAP) != 0U);
    assert(laptimer_get_lap_count() == 1U);

    laptimer_init();
    assert(!laptimer_is_running());
    assert(laptimer_get_lap_count() == 0U);
    assert(laptimer_get_current_elapsed_ms(t + 1000U) == 0U);

    t += 3000U; /* well inside the old lap's debounce window */
    ev = laptimer_update(1.0f, 0.0f, -1.0f, 0.0f, t + 10U, t);
    assert(ev == 0U);
    assert(laptimer_is_running());
    assert(laptimer_get_lap_count() == 0U);
}

int main(void)
{
    test_update_reports_events();
    test_no_sector_event_without_sector_gates();
    test_sector_gate_counts_once_per_lap();
    test_init_restarts_timing();
    test_circle_replay_lap_times();
    test_sector_gate_within_lap();
    test_final_sector_timed_on_lap();
    test_itow_rollover_does_not_spike_elapsed();

    printf("test_laptimer: all tests passed\n");
    return 0;
}
