/**
 * @file    laptimer.h
 * @brief   Lap/segment timing state machine built on gates.c. Pure C, no
 *          HAL - host testable.
 *
 * Consumes already-fused ENU position and GPS time-of-week (itow_ms); it
 * does not touch GPS or geodesy code directly. Driven at the fused-
 * position rate (~104 Hz) by whichever fusion module is upstream.
 */

#ifndef LAPTIMER_H
#define LAPTIMER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/** @brief Reset all timing state (lap count, running flag, clocks). Does
 *         not touch stored gates - call gates_init()/gates_clear_all()
 *         separately if a full reset is wanted. */
void laptimer_init(void);

/**
 * @brief Advance the timing state machine by one fused-position sample.
 *
 * Checks gate 0 (start/finish) and, once running, every sector gate for
 * a forward crossing between the previous and current sample. A gate-0
 * crossing starts the clock on the first lap, or completes/restarts the
 * lap on later crossings (subject to LAP_MIN_LAP_TIME_MS debounce - see
 * laptimer.c). Sector gate crossings complete/restart the current sector
 * without touching the lap clock.
 *
 * @param itow_ms      GPS time-of-week (ms) at the current sample.
 * @param prev_itow_ms GPS time-of-week (ms) at the previous sample.
 */
void laptimer_update(float east_m, float north_m, float prev_east_m,
                      float prev_north_m, uint32_t itow_ms,
                      uint32_t prev_itow_ms);

/** @brief True once the first start/finish crossing has started a lap. */
bool laptimer_is_running(void);

/**
 * @brief Elapsed time of the currently running lap, or 0 if not running.
 * @param itow_ms_now Current GPS time-of-week (ms), for the caller to
 *                     query between laptimer_update() calls (e.g. from
 *                     the CAN task building Lap_Status).
 */
uint32_t laptimer_get_current_elapsed_ms(uint32_t itow_ms_now);

/** @brief Duration of the last completed lap, ms (0 if none completed). */
uint32_t laptimer_get_last_lap_ms(void);

/** @brief Number of laps completed since the last laptimer_init(). */
uint16_t laptimer_get_lap_count(void);

/** @brief Index of the sector currently in progress (0 = the sector
 *         following the most recent start/finish or sector crossing). */
uint8_t laptimer_get_current_sector(void);

/** @brief Duration of the last completed sector, ms (0 if none yet). */
uint32_t laptimer_get_last_sector_ms(void);

#ifdef __cplusplus
}
#endif

#endif /* LAPTIMER_H */
