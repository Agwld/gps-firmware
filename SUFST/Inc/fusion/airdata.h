/**
 * @file    airdata.h
 * @brief   Airspeed and wind from the pitot pressures plus the fused ground
 *          velocity. Pure C, no HAL - host testable.
 *
 * Airspeed comes from Bernoulli on the dynamic pressure q = p_total -
 * p_static (incompressible: v = sqrt(2 q / rho)), with air density from
 * the static pressure and a temperature by the ideal gas law.
 *
 * Two things a bare pitot gets wrong on a car are estimated here:
 *
 *  - The offset between the total and static channels (each absolute
 *    sensor has its own zero error - ~50 Pa on the P4-A, i.e. a phantom
 *    ~9 m/s). It is re-measured ("tared") every time the car stands
 *    still, when the true q is ~0. Wind at that moment is tared away too
 *    (a 5 m/s wind is ~15 Pa), so a stop in shelter gives the best zero.
 *
 *  - Wind and the installation scale factor. The pitot sits in the car's
 *    own flow field, so it reads k * |v_rel| for some k near 1, where
 *    v_rel = v_ground - wind is the air velocity relative to the car.
 *    With the ground velocity known from the GPS-aided filter, a small
 *    extended Kalman filter on [k, wind_e, wind_n] fits that model as the
 *    car drives round: the wind is only observable because the car points
 *    in many directions over a lap, and k because its speed varies.
 *
 * Output is for logging and analysis only; nothing here feeds back into
 * the position/velocity filter or lap timing.
 */

#ifndef AIRDATA_H
#define AIRDATA_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AIRDATA_DEFAULT_TEMP_C 20.0f /* when no usable temperature */

typedef struct {
    float p_total_mbar;  /* pitot total pressure, absolute */
    float p_static_mbar; /* static reference pressure, absolute */
    float temp_c;        /* air temperature for density */
    bool temp_valid;     /* false: use AIRDATA_DEFAULT_TEMP_C */
    float ve_mps;        /* fused ground velocity, ENU east */
    float vn_mps;        /* fused ground velocity, ENU north */
    bool ground_valid;   /* ve/vn are GPS-aided and current */
    float dt_s;          /* time since the previous update */
} airdata_input_t;

typedef struct {
    /* zero (tare) of the total - static difference */
    float q_offset_pa;
    float stationary_s;
    bool tared;

    /* EKF: x = [k, wind_e, wind_n] */
    float x[3];
    float p[3][3];

    /* latest results */
    float q_pa;          /* dynamic pressure after the tare */
    float density;       /* kg/m^3 */
    float indicated_mps; /* sqrt(2 q / rho), before the scale factor */
    bool temp_used;      /* density used the supplied temperature */
} airdata_t;

/** @brief Reset to untared, k = 1, no wind, uninformative uncertainty. */
void airdata_init(airdata_t *a);

/** @brief Process one pressure sample (call at the pitot frame rate). */
void airdata_update(airdata_t *a, const airdata_input_t *in);

/** @brief True airspeed estimate, m/s: indicated / k. 0 until tared. */
float airdata_airspeed_mps(const airdata_t *a);

/** @brief Estimated wind velocity (the air's motion over the ground),
 *         ENU m/s. */
void airdata_wind(const airdata_t *a, float *wind_e_mps, float *wind_n_mps);

/** @brief Estimated installation scale factor k (pitot reads k x true). */
float airdata_scale(const airdata_t *a);

/** @brief True once the wind and scale estimates have converged. */
bool airdata_wind_valid(const airdata_t *a);

/** @brief Mean of the channels selected by `mask` (bit i = channel i+1),
 *         e.g. several static ports averaged into one reference. Returns
 *         0 for an empty mask. */
float airdata_mean_channels(const float ch_mbar[4], uint8_t mask);

/** @brief Air density from static pressure and temperature (ideal gas). */
float airdata_density(float p_static_mbar, float temp_c);

#ifdef __cplusplus
}
#endif

#endif /* AIRDATA_H */
