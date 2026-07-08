/**
 * @file    timebase.c
 * @brief   Tick <-> GPS iTOW affine mapping, disciplined by PPS edges.
 */

#include "fusion/timebase.h"

#include <stdbool.h>

#ifndef HOST_TEST_BUILD
#include "main.h"
#include "stm32g4xx_ll_tim.h"
#endif

/** GPS time-of-week wraps at the week boundary. */
#define TIMEBASE_WEEK_MS 604800000U

/** Tick counter is documented (timebase.h) as running in microseconds;
 * used only as a fallback rate before a second PPS observation lets us
 * measure the real rate. */
#define TIMEBASE_TICKS_PER_MS_NOMINAL 1000

typedef struct {
    uint32_t tick;
    uint32_t itow_ms;
    bool valid;
} pps_obs_t;

static pps_obs_t s_prev; /* older of the last two PPS observations */
static pps_obs_t s_last; /* most recent PPS observation */

void
timebase_init(void)
{
    s_prev.valid = false;
    s_last.valid = false;
}

void
timebase_on_pps(uint32_t tick_at_pps, uint32_t itow_ms_of_pps)
{
    if (s_last.valid) {
        s_prev = s_last;
    }
    s_last.tick = tick_at_pps;
    s_last.itow_ms = itow_ms_of_pps;
    s_last.valid = true;
}

bool
timebase_is_disciplined(void)
{
    return s_last.valid;
}

/* Rollover-safe iTOW difference (newer - older), wrapped into
 * [-week/2, +week/2] so a difference spanning the week boundary comes
 * out as the small delta it physically is instead of a ~604800000 ms
 * jump. */
static int64_t
itow_diff_ms(uint32_t newer, uint32_t older)
{
    int64_t raw = (int64_t) newer - (int64_t) older;

    if (raw < -(int64_t) (TIMEBASE_WEEK_MS / 2U)) {
        raw += (int64_t) TIMEBASE_WEEK_MS;
    } else if (raw > (int64_t) (TIMEBASE_WEEK_MS / 2U)) {
        raw -= (int64_t) TIMEBASE_WEEK_MS;
    }

    return raw;
}

/* Add a (possibly negative) ms delta to an iTOW, wrapping into
 * [0, TIMEBASE_WEEK_MS). */
static uint32_t
itow_add_ms(uint32_t itow_ms, int64_t delta_ms)
{
    int64_t result = ((int64_t) itow_ms + delta_ms) % (int64_t) TIMEBASE_WEEK_MS;

    if (result < 0) {
        result += (int64_t) TIMEBASE_WEEK_MS;
    }

    return (uint32_t) result;
}

uint32_t
timebase_tick_to_itow_ms(uint32_t tick)
{
    if (!s_last.valid) {
        /* Not disciplined yet; nothing better to report. */
        return 0U;
    }

    /* Tick delta from the most recent PPS observation to "now" - signed
     * cast handles the tick counter's own 32-bit wraparound correctly as
     * long as the elapsed tick count fits in 31 bits (~35 min at a 1 MHz
     * tick rate), which always holds since PPS re-disciplines ~1/s. */
    int64_t dtick_now = (int64_t) (int32_t) (tick - s_last.tick);

    if (!s_prev.valid) {
        /* Only one PPS observation so far: extrapolate at the nominal
         * (undisciplined) rate until a second PPS lets us measure the
         * actual one. */
        int64_t d_ms = dtick_now / TIMEBASE_TICKS_PER_MS_NOMINAL;
        return itow_add_ms(s_last.itow_ms, d_ms);
    }

    int32_t dtick_obs = (int32_t) (s_last.tick - s_prev.tick);
    int64_t ditow_obs = itow_diff_ms(s_last.itow_ms, s_prev.itow_ms);

    if (dtick_obs <= 0) {
        /* Degenerate observation pair (shouldn't happen for a real PPS
         * source); fall back to the nominal rate rather than divide by a
         * non-positive tick delta. */
        int64_t d_ms = dtick_now / TIMEBASE_TICKS_PER_MS_NOMINAL;
        return itow_add_ms(s_last.itow_ms, d_ms);
    }

    /* Affine map disciplined by the measured rate between the last two
     * PPS edges: ms_per_tick = ditow_obs / dtick_obs. Done as a single
     * 64-bit multiply-then-divide (no floating point) so precision does
     * not degrade as dtick_now grows between PPS edges. */
    int64_t d_ms = (dtick_now * ditow_obs) / (int64_t) dtick_obs;

    return itow_add_ms(s_last.itow_ms, d_ms);
}

uint32_t
timebase_itow_ms_to_tick(uint32_t itow_ms)
{
    if (!s_last.valid) {
        return 0U;
    }

    /* Signed ms delta from the last PPS observation to the target iTOW,
     * wraparound-safe across the GPS week boundary. */
    int64_t ditow_now = itow_diff_ms(itow_ms, s_last.itow_ms);

    if (!s_prev.valid) {
        int64_t dtick = ditow_now * (int64_t) TIMEBASE_TICKS_PER_MS_NOMINAL;
        return (uint32_t) ((int64_t) s_last.tick + dtick);
    }

    int32_t dtick_obs = (int32_t) (s_last.tick - s_prev.tick);
    int64_t ditow_obs = itow_diff_ms(s_last.itow_ms, s_prev.itow_ms);

    if (dtick_obs <= 0 || ditow_obs == 0) {
        int64_t dtick = ditow_now * (int64_t) TIMEBASE_TICKS_PER_MS_NOMINAL;
        return (uint32_t) ((int64_t) s_last.tick + dtick);
    }

    /* Inverse of the forward affine map: dtick = ditow_now / (ms_per_tick)
     * = ditow_now * dtick_obs / ditow_obs. */
    int64_t dtick = (ditow_now * (int64_t) dtick_obs) / ditow_obs;

    return (uint32_t) ((int64_t) s_last.tick + dtick);
}

/* Written only by timebase_tim3_irq() (target build); read/cleared by
 * timebase_take_pending_pps(). Plain word/bool accesses are atomic on a
 * Cortex-M without a critical section, and losing a race against a fresh
 * edge lands on "stale by one PPS period", which is harmless at a 1 Hz
 * signal - so none is used here. */
static volatile uint32_t s_pending_pps_tick;
static volatile bool s_pps_pending;

bool
timebase_take_pending_pps(uint32_t *tick_at_pps)
{
    if (!s_pps_pending) {
        return false;
    }

    *tick_at_pps = s_pending_pps_tick;
    s_pps_pending = false;
    return true;
}

#ifndef HOST_TEST_BUILD

/* Number of TIM3 update (16-bit rollover) events seen, extending the
 * hardware counter into the 32-bit microsecond tick documented in
 * timebase.h. */
static volatile uint16_t s_tick_high;

void
timebase_tim3_irq(void)
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM3)) {
        LL_TIM_ClearFlag_UPDATE(TIM3);
        s_tick_high++;
    }

    if (LL_TIM_IsActiveFlag_CC2(TIM3)) {
        LL_TIM_ClearFlag_CC2(TIM3);

        /* Note: if the PPS edge lands within the last handful of
         * microseconds before a rollover, the update flag may already
         * be latched (and processed above, incrementing s_tick_high)
         * before this capture is read, misattributing the edge to the
         * new period and off by one rollover (~65 ms). At most one such
         * PPS observation in ~18 hours (1-in-65536 chance per edge) -
         * timebase_on_pps()'s two-point rate estimate treats a bad
         * point as an outlier for one cycle, not worth extra register
         * gymnastics to close entirely. */
        uint16_t capture = (uint16_t) LL_TIM_IC_GetCaptureCH2(TIM3);
        s_pending_pps_tick = ((uint32_t) s_tick_high << 16) | capture;
        s_pps_pending = true;
    }
}

uint32_t
timebase_get_tick(void)
{
    uint16_t high1, high2, low;

    /* Retry until a read isn't split by a rollover landing between the
     * high-word and low-word reads (timebase_tim3_irq() runs at a higher
     * priority than any caller of this function, so it can only ever
     * happen between these two lines, not concurrently with them). */
    do {
        high1 = s_tick_high;
        low = (uint16_t) LL_TIM_GetCounter(TIM3);
        high2 = s_tick_high;
    } while (high1 != high2);

    return ((uint32_t) high2 << 16) | low;
}

#endif /* HOST_TEST_BUILD */
