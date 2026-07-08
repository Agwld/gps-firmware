/**
 * @file    laptimer.c
 * @brief   Lap/segment timing state machine - see laptimer.h.
 */

#include "laptimer/laptimer.h"

#include "board/board_config.h"
#include "laptimer/gates.h"

/* GPS itow_ms resets to 0 at the start of a new GPS week rather than
 * wrapping at UINT32_MAX, so a naive (now - start) subtraction underflows
 * to a huge value across the boundary. */
#define ITOW_WEEK_MS 604800000UL

typedef struct {
    bool running;
    uint32_t lap_start_itow_ms;
    uint32_t sector_start_itow_ms;
    uint32_t last_lap_ms;
    uint32_t last_sector_ms;
    uint16_t lap_count;
    uint8_t current_sector;
} laptimer_state_t;

static laptimer_state_t s_state;

/** @brief itow_ms delta from start to now, correct across a single GPS
 *         week rollover (laps/sectors are always far shorter than a
 *         week, so more than one rollover between samples can't happen). */
static uint32_t itow_elapsed_ms(uint32_t start_ms, uint32_t now_ms)
{
    if (now_ms >= start_ms) {
        return now_ms - start_ms;
    }
    return (ITOW_WEEK_MS - start_ms) + now_ms;
}

/** @brief Interpolate the itow_ms at which a crossing occurred, given the
 *         bracketing samples and the [0,1] fraction from
 *         gates_check_crossing(). */
static uint32_t itow_interpolate(uint32_t prev_itow_ms, uint32_t itow_ms,
                                  float frac)
{
    uint32_t dt = itow_elapsed_ms(prev_itow_ms, itow_ms);
    uint32_t crossing_ms = prev_itow_ms + (uint32_t) (frac * (float) dt);

    if (crossing_ms >= ITOW_WEEK_MS) {
        crossing_ms -= ITOW_WEEK_MS;
    }
    return crossing_ms;
}

void laptimer_init(void)
{
    s_state.running = false;
    s_state.lap_start_itow_ms = 0U;
    s_state.sector_start_itow_ms = 0U;
    s_state.last_lap_ms = 0U;
    s_state.last_sector_ms = 0U;
    s_state.lap_count = 0U;
    s_state.current_sector = 0U;
}

void laptimer_update(float east_m, float north_m, float prev_east_m,
                      float prev_north_m, uint32_t itow_ms,
                      uint32_t prev_itow_ms)
{
    float frac;

    if (gates_check_crossing(0U, prev_east_m, prev_north_m, east_m, north_m,
                              &frac)) {
        uint32_t crossing_ms
            = itow_interpolate(prev_itow_ms, itow_ms, frac);

        if (!s_state.running) {
            s_state.running = true;
            s_state.lap_start_itow_ms = crossing_ms;
            s_state.sector_start_itow_ms = crossing_ms;
            s_state.current_sector = 0U;
        } else {
            uint32_t elapsed = itow_elapsed_ms(s_state.lap_start_itow_ms,
                                                crossing_ms);

            /* Debounce: a noisy fix or a car crawling/line-hugging near
             * the start/finish can re-trigger a forward crossing well
             * inside one real lap; no legitimate lap is this short. */
            if (elapsed >= LAP_MIN_LAP_TIME_MS) {
                /* Crossing the finish line also closes the final sector
                 * (last sector gate -> finish). Time it here, before the
                 * sector state resets, so that sector is reported with its
                 * true duration rather than the previous sector's stale
                 * value; without this the final sector is never timed. */
                s_state.last_sector_ms = itow_elapsed_ms(
                    s_state.sector_start_itow_ms, crossing_ms);
                s_state.last_lap_ms = elapsed;
                s_state.lap_count++;
                s_state.lap_start_itow_ms = crossing_ms;
                s_state.sector_start_itow_ms = crossing_ms;
                s_state.current_sector = 0U;
            }
        }
    }

    if (!s_state.running) {
        return;
    }

    for (uint8_t i = 1U; i < LAP_MAX_GATES; i++) {
        if (gates_check_crossing(i, prev_east_m, prev_north_m, east_m,
                                  north_m, &frac)) {
            uint32_t crossing_ms
                = itow_interpolate(prev_itow_ms, itow_ms, frac);

            s_state.last_sector_ms = itow_elapsed_ms(
                s_state.sector_start_itow_ms, crossing_ms);
            s_state.sector_start_itow_ms = crossing_ms;
            s_state.current_sector++;
        }
    }
}

bool laptimer_is_running(void)
{
    return s_state.running;
}

uint32_t laptimer_get_current_elapsed_ms(uint32_t itow_ms_now)
{
    if (!s_state.running) {
        return 0U;
    }
    return itow_elapsed_ms(s_state.lap_start_itow_ms, itow_ms_now);
}

uint32_t laptimer_get_last_lap_ms(void)
{
    return s_state.last_lap_ms;
}

uint16_t laptimer_get_lap_count(void)
{
    return s_state.lap_count;
}

uint8_t laptimer_get_current_sector(void)
{
    return s_state.current_sector;
}

uint32_t laptimer_get_last_sector_ms(void)
{
    return s_state.last_sector_ms;
}
