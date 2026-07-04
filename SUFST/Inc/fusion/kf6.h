/**
 * @file    kf6.h
 * @brief   6-state linear Kalman filter (local ENU position + velocity)
 *          with delayed-state ("out-of-sequence measurement") correction.
 *
 * State is [pos_e, pos_n, pos_u, vel_e, vel_n, vel_u] in the local ENU
 * frame set up by geodesy.c. Prediction runs at the IMU rate (~104 Hz)
 * from calibrated body-frame accel rotated into ENU by the AHRS
 * quaternion; correction comes from GPS NAV-PVT position/velocity and,
 * optionally, wheelspeed projected along the vehicle heading.
 *
 * GPS fixes describe the vehicle's state 20-60 ms in the past by the
 * time they're processed (F9P internal latency), which at racing speeds
 * is a metre-plus of travel - applying that correction to the *current*
 * state as if it were current would smear it across the wrong epoch. To
 * avoid that, every predict step is kept in a short history ring
 * (KF6_HISTORY_LEN entries); a correct_* call rewinds to the history
 * entry at the fix's true epoch, applies the correction there, then
 * replays every subsequent predict step so the returned "current" state
 * reflects the fix applied at the right point in time. No bias states:
 * with GPS updates every 50 ms, they wouldn't have time to matter.
 */

#ifndef KF6_H
#define KF6_H

#include <stdint.h>

#include "fusion/ahrs.h"

#ifdef __cplusplus
extern "C" {
#endif

/** History depth: covers the worst-case ~60 ms NAV-PVT latency at the
 * ~9.6 ms IMU predict period with margin (16 * 9.6 ms ~= 154 ms). */
#define KF6_HISTORY_LEN 16U

/** @brief Reset state to the origin at rest, covariance to an
 *         uninformative prior, and clear the delayed-state history. */
void kf6_init(void);

/**
 * @brief Predict step: propagate state by one IMU sample.
 *
 * @param tick   monotonic sample time in whatever units the caller uses
 *               consistently across every kf6_predict()/kf6_correct_*()
 *               call (e.g. a free-running microsecond tick) - this
 *               module assigns no absolute meaning to it beyond ordering
 *               and placing correction epochs within the history ring.
 * @param ax, ay, az   calibrated body-frame accel, m/s^2, as read by the
 *                     accelerometer (specific force - this function
 *                     removes gravity itself using @p q).
 * @param q      current attitude quaternion, body frame -> ENU (from
 *               ahrs.c; taken here purely as an opaque rotation input).
 * @param dt     seconds since the previous predict call.
 */
void kf6_predict(uint32_t tick, float ax, float ay, float az, quat_t q,
                  float dt);

/**
 * @brief Fuse a GPS position fix (ENU metres, e.g. from
 *        geodesy_to_enu()), rewinding to its true epoch first.
 *
 * @param fix_tick     tick value the fix corresponds to (same units as
 *                     kf6_predict()'s tick). If older than the entire
 *                     retained history window, the oldest entry is used
 *                     instead (best effort).
 * @param e_m, n_m, u_m   fix position, ENU metres.
 * @param sigma_pos_m     1-sigma position measurement noise, metres
 *                        (isotropic across all three axes).
 */
void kf6_correct_pos(uint32_t fix_tick, float e_m, float n_m, float u_m,
                      float sigma_pos_m);

/**
 * @brief Fuse a GPS velocity fix (ENU m/s), same rewind mechanism as
 *        kf6_correct_pos().
 */
void kf6_correct_vel(uint32_t fix_tick, float ve_mps, float vn_mps,
                      float vu_mps, float sigma_vel_mps);

/**
 * @brief Fuse a scalar wheelspeed measurement projected onto the vehicle
 *        heading, same rewind mechanism. Keeps velocity aided through
 *        GPS dropout (tunnels, tight corners with poor sky view) where
 *        NAV-PVT can't help.
 *
 * @param heading_rad   direction speed_mps was measured along, radians,
 *                      ENU convention (0 = East, positive toward North).
 */
void kf6_correct_speed(uint32_t fix_tick, float speed_mps,
                        float heading_rad, float sigma_speed_mps);

/** @brief Read the current fused state. Any output pointer may be NULL. */
void kf6_get_state(float *e_m, float *n_m, float *u_m, float *ve_mps,
                    float *vn_mps, float *vu_mps);

#ifdef __cplusplus
}
#endif

#endif /* KF6_H */
