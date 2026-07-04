/**
 * @file    ahrs.c
 * @brief   Mahony 9-DOF explicit-complementary-filter AHRS.
 */

#include "fusion/ahrs.h"

#include <math.h>
#include <stdbool.h>

/* Proportional/integral gains on the correction rate (rad/s per unit
 * cross-product error). Values follow the usual Mahony (2008) sizing for
 * a vehicle-mounted IMU: fast enough proportional term to track dynamic
 * accel/mag corrections within a lap, small integral term so gyro-bias
 * estimation settles over seconds rather than fighting genuine turns. */
#define AHRS_KP     1.0f
#define AHRS_KI     0.01f
#define AHRS_TWO_KP (2.0f * AHRS_KP)
#define AHRS_TWO_KI (2.0f * AHRS_KI)

typedef struct {
    float x, y, z;
} ahrs_vec3_t;

static quat_t s_q;
static ahrs_vec3_t s_bias; /* integral gyro-bias estimate, rad/s */

void
ahrs_init(void)
{
    s_q.w = 1.0f;
    s_q.x = 0.0f;
    s_q.y = 0.0f;
    s_q.z = 0.0f;

    s_bias.x = 0.0f;
    s_bias.y = 0.0f;
    s_bias.z = 0.0f;
}

void
ahrs_update(float gx, float gy, float gz, float ax, float ay, float az,
            float mx, float my, float mz, float dt, quat_t *q_out)
{
    float q0 = s_q.w, q1 = s_q.x, q2 = s_q.y, q3 = s_q.z;
    bool have_accel = !(ax == 0.0f && ay == 0.0f && az == 0.0f);

    if (have_accel) {
        float halfex, halfey, halfez;
        float recip_norm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);

        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;

        /* Estimated direction of gravity in the body frame, from the
         * current quaternion (world Z is up, so a level, stationary
         * accelerometer reads +g on its Z axis - see ahrs.h). */
        float halfvx = q1 * q3 - q0 * q2;
        float halfvy = q0 * q1 + q2 * q3;
        float halfvz = q0 * q0 - 0.5f + q3 * q3;

        halfex = ay * halfvz - az * halfvy;
        halfey = az * halfvx - ax * halfvz;
        halfez = ax * halfvy - ay * halfvx;

        bool have_mag = !(mx == 0.0f && my == 0.0f && mz == 0.0f);

        if (have_mag) {
            recip_norm = 1.0f / sqrtf(mx * mx + my * my + mz * mz);
            mx *= recip_norm;
            my *= recip_norm;
            mz *= recip_norm;

            float q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3;
            float q1q1 = q1 * q1, q1q2 = q1 * q2, q1q3 = q1 * q3;
            float q2q2 = q2 * q2, q2q3 = q2 * q3, q3q3 = q3 * q3;

            /* Reference magnetic field direction, expressed in the
             * world frame as a horizontal component bx (this *defines*
             * world X for this update - see ahrs.h) and vertical
             * component bz, then rotated back into the body frame as
             * halfwx/y/z for comparison against the measured mx/my/mz. */
            float hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) +
                                my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
            float hy = 2.0f * (mx * (q1q2 + q0q3) +
                                my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
            float bx = sqrtf(hx * hx + hy * hy);
            float bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) +
                                mz * (0.5f - q1q1 - q2q2));

            float halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
            float halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
            float halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

            halfex += my * halfwz - mz * halfwy;
            halfey += mz * halfwx - mx * halfwz;
            halfez += mx * halfwy - my * halfwx;
        }

        s_bias.x += AHRS_TWO_KI * halfex * dt;
        s_bias.y += AHRS_TWO_KI * halfey * dt;
        s_bias.z += AHRS_TWO_KI * halfez * dt;

        gx += s_bias.x + AHRS_TWO_KP * halfex;
        gy += s_bias.y + AHRS_TWO_KP * halfey;
        gz += s_bias.z + AHRS_TWO_KP * halfez;
    } else {
        /* No new correction this step; still apply the last integral
         * bias estimate so gyro-only integration is not left undamped
         * for the duration of the accel dropout. */
        gx += s_bias.x;
        gy += s_bias.y;
        gz += s_bias.z;
    }

    /* Integrate rate of change of quaternion: dq/dt = 0.5 * q * omega. */
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    float qa = q0, qb = q1, qc = q2;
    q0 += -qb * gx - qc * gy - q3 * gz;
    q1 += qa * gx + qc * gz - q3 * gy;
    q2 += qa * gy - qb * gz + q3 * gx;
    q3 += qa * gz + qb * gy - qc * gx;

    float recip_norm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    s_q.w = q0 * recip_norm;
    s_q.x = q1 * recip_norm;
    s_q.y = q2 * recip_norm;
    s_q.z = q3 * recip_norm;

    *q_out = s_q;
}
