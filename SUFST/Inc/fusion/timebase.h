/**
 * @file    timebase.h
 * @brief   Affine mapping between a free-running microsecond tick counter
 *          and GPS time-of-week (iTOW), disciplined by PPS edges.
 *
 * The tick counter is TIM3 extended to 32 bits on target (microsecond
 * resolution); host tests just pass synthetic tick values in the same
 * units. Two PPS observations (tick, iTOW-of-that-second-boundary) give
 * both an offset *and* a measured rate (ms of GPS time per tick), so the
 * mapping tracks the tick clock's drift relative to GPS time rather than
 * assuming it is nominally exact - important since the tick source is a
 * free-running MCU timer, not a GPS-locked oscillator.
 */

#ifndef TIMEBASE_H
#define TIMEBASE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Reset the timebase to "undisciplined" (no PPS seen yet). */
void timebase_init(void);

/**
 * @brief Record a PPS observation: the tick value captured at a PPS edge
 *        and the iTOW (ms) of the second boundary that edge represents.
 *
 * Keeps the two most recent observations; the second and every
 * subsequent call refines the measured tick-to-ms rate used by
 * timebase_tick_to_itow_ms().
 */
void timebase_on_pps(uint32_t tick_at_pps, uint32_t itow_ms_of_pps);

/**
 * @brief Convert a tick reading to an estimated GPS iTOW (ms), extrapolated
 *        from the last one or two PPS observations.
 *
 * @return iTOW in ms, wrapped into [0, 604800000) (GPS week length) so a
 *         tick reading spanning a week boundary doesn't produce a huge
 *         jump or a negative-looking value.
 */
uint32_t timebase_tick_to_itow_ms(uint32_t tick);

/**
 * @brief Inverse of timebase_tick_to_itow_ms(): the tick value
 *        corresponding to a given GPS iTOW, extrapolated from the same
 *        disciplined affine mapping.
 *
 * Used to place a GPS fix's true epoch (known in iTOW) into the tick
 * domain that kf6.c's delayed-state history ring is keyed by.
 *
 * @return tick value, or 0 if undisciplined (no PPS observed yet) - same
 *         "nothing better to report" convention as the forward mapping.
 */
uint32_t timebase_itow_ms_to_tick(uint32_t itow_ms);

/**
 * @brief Has at least one PPS edge disciplined the mapping yet?
 *
 * Until this is true, timebase_itow_ms_to_tick() has no real reference
 * and returns 0, so callers that place a GPS fix into the tick domain
 * (kf6's delayed-state rewind) should hold off applying the correction
 * rather than anchor it to a bogus epoch.
 *
 * @return true once timebase_on_pps() has been called at least once.
 */
bool timebase_is_disciplined(void);

/**
 * @brief Fetch and clear the most recently captured PPS edge tick, if
 *        any edge has arrived since the last call.
 *
 * The ISR only knows the raw tick counter value at the edge - it has no
 * idea what GPS second that edge marks. The caller (gps_task, which
 * tracks the current NAV-PVT iTOW) is expected to pair the returned tick
 * with the nearest GPS second boundary and pass both to
 * timebase_on_pps() to actually discipline the mapping.
 *
 * @param tick_at_pps  [out] tick value of the captured PPS edge, valid
 *                     only if this function returns true.
 * @return true if a not-yet-consumed PPS edge was available.
 */
bool timebase_take_pending_pps(uint32_t *tick_at_pps);

#ifndef HOST_TEST_BUILD
/**
 * @brief TIM3 ISR entry point (called from the target's
 *        TIM3_IRQHandler): services the update interrupt (extends the
 *        hardware 16-bit counter into the 32-bit tick used throughout
 *        this module) and the CC2 interrupt (latches the PPS edge's
 *        tick value for timebase_take_pending_pps()). Target build
 *        only - touches TIM3 registers directly, so it isn't available
 *        (or needed) for host tests.
 */
void timebase_tim3_irq(void);

/** @brief Read the current 32-bit microsecond tick from TIM3 + the
 *         software-extended high word. Target build only. */
uint32_t timebase_get_tick(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* TIMEBASE_H */
