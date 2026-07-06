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
 * Two ways to drive it:
 *
 *  - Continuous (default): a mag_cal_cont_t is fed every raw sample as
 *    the car drives. It sweeps the heading circle naturally, self-heals
 *    from magnetic transients by working in windows, raises a quality
 *    flag as coverage builds, and cross-checks the resulting heading
 *    against GPS course-over-ground to reach a "validated" state. No
 *    driver action needed.
 *
 *  - Manual pass: mag_cal_start() / mag_cal_feed() / mag_cal_finish()
 *    around a CAN-triggered pass (CAN_CMD_MAG_CAL_START/STOP), rotating
 *    the car through as much of a full sphere as practical. Kept as a
 *    "force a fresh calibration now" override.
 *
 * Either way the result is applied to every subsequent raw reading via
 * mag_cal_apply() before it reaches ahrs_update().
 */

#ifndef MAG_CAL_H
#define MAG_CAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

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

/* Broadcast in GPS_Mag.cal_status. Monotonic in "trust": a consumer that
 * only wants a heading it can rely on should gate on >= MAG_CAL_GOOD. */
typedef enum {
    MAG_CAL_UNCALIBRATED = 0, /* identity applied; heading not trustworthy */
    MAG_CAL_COLLECTING = 1,   /* a candidate exists, still sweeping */
    MAG_CAL_GOOD = 2,         /* heading-circle coverage + range sufficient */
    MAG_CAL_VALIDATED = 3,    /* also agrees with GPS course-over-ground */
} mag_cal_quality_t;

/* Heading circle split into sectors for coverage tracking; a window is
 * only trusted once most sectors have been driven through. */
#define MAG_CAL_HEADING_BINS 12

/* Continuous background calibrator (see file header). Fed every raw mag
 * sample; periodically finalises a swept window into `result`, which
 * mag_cal_apply() then uses. Plain data, no HAL - host testable. */
typedef struct {
    mag_cal_result_t result;      /* currently-applied calibration */
    mag_cal_quality_t quality;
    bool dirty;                   /* result changed since last save */

    mag_cal_accumulator_t window; /* the in-progress sweep */
    uint16_t bins_seen;           /* bitmask over MAG_CAL_HEADING_BINS */

    /* GPS-course cross-check: circular accumulation of (course - yaw),
     * capped so it forgets old blocks and stays adaptive. */
    float off_sin;
    float off_cos;
    unsigned long val_count;
} mag_cal_cont_t;

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

/* ---- Continuous background calibration --------------------------- */

/**
 * @brief Initialise a continuous calibrator from a starting calibration
 *        (typically the one restored from flash, or mag_cal_identity()).
 *
 * A non-identity `initial` starts at MAG_CAL_GOOD so a persisted
 * calibration is trusted immediately at boot; identity starts at
 * MAG_CAL_UNCALIBRATED.
 */
void mag_cal_cont_init(mag_cal_cont_t *c, const mag_cal_result_t *initial);

/**
 * @brief Feed one raw (uncalibrated) magnetometer sample.
 *
 * Extends the current window and, once the window has swept enough of
 * the heading circle with adequate range, finalises it as the new
 * applied calibration (raising quality and setting `dirty`) and starts a
 * fresh window. Self-heals: a transient disturbance taints at most one
 * window.
 */
void mag_cal_cont_feed(mag_cal_cont_t *c, float mx, float my, float mz);

/**
 * @brief Cross-check the fused heading against an independent heading
 *        reference (GPS course-over-ground), both in radians.
 *
 * Call only when the reference is trustworthy (car moving above the
 * course-valid speed). Accumulates the circular agreement between the
 * two; once tight and coverage is already good, promotes quality to
 * MAG_CAL_VALIDATED. Persistent disagreement demotes it back to GOOD.
 */
void mag_cal_cont_observe_heading(mag_cal_cont_t *c, float fused_yaw_rad,
                                   float course_rad);

/** @brief The calibration to apply right now (feeds mag_cal_apply()). */
const mag_cal_result_t *mag_cal_cont_result(const mag_cal_cont_t *c);

/** @brief Current quality level (for GPS_Mag.cal_status). */
mag_cal_quality_t mag_cal_cont_quality(const mag_cal_cont_t *c);

/**
 * @brief Test-and-clear the "result changed, worth persisting" flag.
 * @return true once after each meaningful calibration change; the caller
 *         (imu_task) rate-limits the actual flash write.
 */
bool mag_cal_cont_take_dirty(mag_cal_cont_t *c);

#ifdef __cplusplus
}
#endif

#endif /* MAG_CAL_H */
