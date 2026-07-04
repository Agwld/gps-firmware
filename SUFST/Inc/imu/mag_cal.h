/**
 * @file    mag_cal.h
 * @brief   Magnetometer hard-iron/soft-iron calibration. Pure C, no HAL -
 *          host testable.
 *
 * This is the "basic" axis-aligned calibration, not a full 9-parameter
 * ellipsoid/skew fit: hard-iron offset is the per-axis midpoint of the
 * observed range, soft-iron correction is a per-axis scale (no
 * cross-axis skew term). It corrects the common case (constant bias
 * field + unequal per-axis gain) but not a magnetometer whose distortion
 * ellipsoid is rotated relative to its own axes. Good enough as a first
 * pass; a full ellipsoid fit is a natural follow-up if bench data shows
 * it's needed.
 *
 * Usage: mag_cal_start() at the beginning of a CAN-triggered calibration
 * pass (CAN_CMD_MAG_CAL_START), feed every raw sample seen while the
 * driver rotates the car through as much of the full sphere of
 * orientations as practical, then mag_cal_finish() on
 * CAN_CMD_MAG_CAL_STOP. The result is applied to every subsequent raw
 * reading via mag_cal_apply() before it reaches ahrs_update().
 */

#ifndef MAG_CAL_H
#define MAG_CAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "sys/status.h"

typedef struct {
    float min[3];
    float max[3];
    unsigned long sample_count;
} mag_cal_accumulator_t;

typedef struct {
    float bias[3];  /* hard-iron offset, subtract from the raw reading */
    float scale[3]; /* soft-iron per-axis scale, multiply after bias */
} mag_cal_result_t;

/** @brief Reset an accumulator to begin a fresh calibration pass. */
void mag_cal_start(mag_cal_accumulator_t *acc);

/** @brief Feed one raw (uncalibrated) magnetometer sample. */
void mag_cal_feed(mag_cal_accumulator_t *acc, float mx, float my,
                   float mz);

/**
 * @brief Compute bias/scale from the accumulated range.
 *
 * @return STATUS_OK, or STATUS_NOT_READY if too few samples were seen or
 *         the observed range on any axis is too small to calibrate from
 *         (the car wasn't actually rotated through enough orientations).
 */
status_t mag_cal_finish(const mag_cal_accumulator_t *acc,
                         mag_cal_result_t *out);

/** @brief Apply a finished calibration to one raw sample. */
void mag_cal_apply(const mag_cal_result_t *cal, float mx, float my,
                    float mz, float *out_x, float *out_y, float *out_z);

/** @brief An identity calibration (bias 0, scale 1) - the safe default
 *         before any calibration has ever been run or loaded from
 *         flash. */
void mag_cal_identity(mag_cal_result_t *out);

#ifdef __cplusplus
}
#endif

#endif /* MAG_CAL_H */
