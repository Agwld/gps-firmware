/**
 * @file    gates.h
 * @brief   Gate storage and crossing detection in local ENU. Pure C, no
 *          HAL - host testable.
 *
 * A gate is a finite-width line segment: it passes through a stored ENU
 * point, oriented perpendicular to a stored heading (the direction the
 * car was facing when the gate was set). The car "crosses" a gate when
 * its position passes through that segment while moving in roughly the
 * stored heading's direction - i.e. forward through the start/finish or
 * a sector line, not backwards, and not somewhere far off to the side of
 * the track.
 */

#ifndef GATES_H
#define GATES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "sys/status.h"

/**
 * @brief Reset all gates to unset. Call once at start-up (persistence
 *        restore, if any, happens afterwards via repeated gates_set()).
 */
void gates_init(void);

/**
 * @brief Store/replace a gate.
 *
 * Setting index 0 (start/finish) also clears every other gate, matching
 * the steering-wheel button behaviour: a new start/finish line makes the
 * previous sector split meaningless.
 *
 * @param index       0 = start/finish, 1..LAP_MAX_GATES-1 = sector gates.
 * @param east_m      Gate point, ENU east, metres.
 * @param north_m     Gate point, ENU north, metres.
 * @param heading_rad Direction of travel through the gate, radians.
 * @return STATUS_OK, or STATUS_INVALID_ARG if index is out of range.
 */
status_t gates_set(uint8_t index, float east_m, float north_m,
                    float heading_rad);

/** @brief Unset every gate (steering-wheel "clear all" long-press). */
void gates_clear_all(void);

/**
 * @brief Unset a single gate (CAN_CMD_GATE_CLEAR with a specific slot,
 *        as opposed to the 0xFF "clear all" case gates_clear_all()
 *        handles).
 * @return STATUS_OK, or STATUS_INVALID_ARG if index is out of range.
 */
status_t gates_clear(uint8_t index);

/**
 * @brief Read back a stored gate (persistence, CAN readback).
 * @return STATUS_OK, STATUS_INVALID_ARG (bad index) or STATUS_NOT_READY
 *         (index in range but never set).
 */
status_t gates_get(uint8_t index, float *east_m, float *north_m,
                    float *heading_rad);

/**
 * @brief Test whether the vehicle crossed gate `index` moving from
 *        (prev_east, prev_north) to (cur_east, cur_north).
 *
 * Only forward crossings register: the displacement must have a positive
 * component along the gate's stored heading, so driving backwards over a
 * line (or across it while stationary/noise-jittering) never triggers.
 *
 * @param frac_out On a detected crossing, set to the linear-interpolation
 *                  fraction in [0,1] along prev->cur at which the
 *                  crossing occurred, for the caller to interpolate the
 *                  exact crossing time from the bracketing samples.
 * @return true if a forward crossing of the finite gate segment occurred.
 */
bool gates_check_crossing(uint8_t index, float prev_east, float prev_north,
                           float cur_east, float cur_north, float *frac_out);

#ifdef __cplusplus
}
#endif

#endif /* GATES_H */
