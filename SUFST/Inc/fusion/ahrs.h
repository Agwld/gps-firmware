/**
 * @file    ahrs.h
 * @brief   Mahony 9-DOF complementary-filter attitude estimator.
 *
 * Standard Mahony (2008) explicit-complementary-filter AHRS: accel gives
 * a gravity-direction reference, mag gives a heading reference, gyro is
 * integrated and PI-corrected against the cross-product error between
 * the measured and quaternion-predicted reference vectors. Runs at the
 * IMU ODR (104 Hz).
 *
 * World-frame convention: this module's world frame is exactly the ENU
 * frame produced by geodesy.c / consumed by kf6.c - Z is up (opposite
 * gravity, i.e. what a stationary accelerometer's Z axis reads), X is
 * the horizontal projection of the magnetometer reference direction
 * (feed mx/my/mz as the East/North/Up components of the calibrated mag
 * reading so this lines up with East), and Y completes the right-handed
 * frame (Y = Z x X = North given X = East, Z = Up). kf6.h's body->ENU
 * rotation relies on this convention.
 */

#ifndef AHRS_H
#define AHRS_H

#ifdef __cplusplus
extern "C" {
#endif

/** Attitude quaternion, scalar-first (w, x, y, z). Shared with kf6.h,
 * which takes this as an opaque input type only (no functional coupling
 * to this module). */
typedef struct {
    float w, x, y, z;
} quat_t;

/** @brief Reset the filter to a level, zero-yaw quaternion with zero
 *         integral gyro-bias estimate. */
void ahrs_init(void);

/**
 * @brief Run one Mahony filter update.
 *
 * @param gx, gy, gz  gyro rate, rad/s, body frame.
 * @param ax, ay, az  accel, any consistent unit (this firmware uses g) -
 *                    only its direction is used, it is re-normalised
 *                    internally. Pass all-zero to skip the accel/mag
 *                    correction entirely and integrate gyro only (e.g.
 *                    while accel is known to be unreliable).
 * @param mx, my, mz  mag, any consistent unit (raw sensor counts are
 *                     fine) - only its direction is used. Pass all-zero
 *                     to fall back to 6-DOF (accel-only) correction.
 * @param dt          seconds since the previous call (nominally 1/104).
 * @param q_out       [out] updated attitude quaternion.
 */
void ahrs_update(float gx, float gy, float gz, float ax, float ay, float az,
                  float mx, float my, float mz, float dt, quat_t *q_out);

#ifdef __cplusplus
}
#endif

#endif /* AHRS_H */
