/**
 * @file    test_ahrs.c
 * @brief   Host tests for fusion/ahrs.c.
 */

#include "fusion/ahrs.h"

#include <math.h>
#include <stdio.h>

static int s_failures = 0;

#define CHECK_NEAR(desc, actual, expected, tol)                             \
    do {                                                                    \
        float diff = (float) fabs((double) (actual) - (double) (expected)); \
        if (diff > (tol)) {                                                 \
            fprintf(stderr,                                                \
                    "FAIL: %s: got %f, expected %f (tol %f, diff %f)\n",    \
                    (desc), (double) (actual), (double) (expected),         \
                    (double) (tol), (double) diff);                        \
            s_failures++;                                                   \
        }                                                                    \
    } while (0)

typedef struct {
    float x, y, z;
} vec3_t;

/* Independent quaternion-vector rotation (v' = q * v * q^-1), deliberately
 * not sharing any formula with ahrs.c's internal halfv/halfw shortcuts, so
 * these tests exercise the filter's actual convergence behaviour rather
 * than just re-checking its own algebra. */
static vec3_t
rotate(quat_t q, vec3_t v)
{
    float tx = 2.0f * (q.y * v.z - q.z * v.y);
    float ty = 2.0f * (q.z * v.x - q.x * v.z);
    float tz = 2.0f * (q.x * v.y - q.y * v.x);

    vec3_t r;
    r.x = v.x + q.w * tx + (q.y * tz - q.z * ty);
    r.y = v.y + q.w * ty + (q.z * tx - q.x * tz);
    r.z = v.z + q.w * tz + (q.x * ty - q.y * tx);
    return r;
}

static quat_t
conjugate(quat_t q)
{
    quat_t r = {q.w, -q.x, -q.y, -q.z};
    return r;
}

#define IMU_DT (1.0f / 104.0f)

/* Pure gyro integration (no accel/mag correction) must reproduce the
 * closed-form quaternion for constant angular rate about a single axis:
 * q(t) = (cos(theta/2), 0, 0, sin(theta/2)) for a rotation of theta about
 * Z, independent of any of the accel/mag correction machinery. */
static void
test_pure_gyro_integration_matches_closed_form(void)
{
    ahrs_init();

    const float rate_rad_s = 1.0f; /* 1 rad/s about Z */
    const int steps = 200;         /* ~1.92 s -> theta ~ 1.923 rad */

    quat_t q = {0};
    for (int i = 0; i < steps; i++) {
        ahrs_update(0.0f, 0.0f, rate_rad_s, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, IMU_DT, &q);
    }

    float theta = rate_rad_s * (float) steps * IMU_DT;
    float expected_w = cosf(theta / 2.0f);
    float expected_z = sinf(theta / 2.0f);

    CHECK_NEAR("pure gyro yaw: w", q.w, expected_w, 1e-3f);
    CHECK_NEAR("pure gyro yaw: x", q.x, 0.0f, 1e-3f);
    CHECK_NEAR("pure gyro yaw: y", q.y, 0.0f, 1e-3f);
    CHECK_NEAR("pure gyro yaw: z", q.z, expected_z, 1e-3f);
}

/* Starting from an arbitrary tilted attitude (reached by pure gyro
 * integration, itself verified above), feeding a constant level-accel
 * reading (0,0,1) with no gyro/mag input must converge the filter's
 * notion of "up" (world Z expressed in the body frame, computed via an
 * independent rotation, not ahrs.c's internal shortcut) onto that
 * reading. */
static void
test_accel_only_converges_to_level(void)
{
    ahrs_init();

    quat_t q = {0};
    /* Tilt ~46 deg about X via pure gyro integration to seed a non-level
     * starting attitude. */
    for (int i = 0; i < 100; i++) {
        ahrs_update(0.8f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    IMU_DT, &q);
    }

    vec3_t up_before = rotate(conjugate(q), (vec3_t) {0.0f, 0.0f, 1.0f});
    /* Sanity: the seeding tilt actually moved "up" away from body Z. */
    CHECK_NEAR("seed tilt moved up vector",
               (fabsf(up_before.y) > 0.3f) ? 1.0f : 0.0f, 1.0f, 0.0f);

    for (int i = 0; i < 800; i++) {
        ahrs_update(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
                    IMU_DT, &q);
    }

    vec3_t up_after = rotate(conjugate(q), (vec3_t) {0.0f, 0.0f, 1.0f});
    CHECK_NEAR("accel-only levelling: x", up_after.x, 0.0f, 1e-2f);
    CHECK_NEAR("accel-only levelling: y", up_after.y, 0.0f, 1e-2f);
    CHECK_NEAR("accel-only levelling: z", up_after.z, 1.0f, 1e-2f);
}

/* Magnetometer correction must resolve a yaw error that accel alone
 * cannot see. Construct a physical scenario with a known true yaw
 * (q_true, a pure rotation about Z) and a body-frame mag reading
 * consistent with a fixed horizontal world field under that true
 * attitude; start the filter's internal estimate at identity (wrong by
 * exactly q_true) and check it converges onto q_true. */
static void
test_mag_correction_resolves_yaw(void)
{
    ahrs_init();

    float theta_true = 40.0f * (float) M_PI / 180.0f;
    quat_t q_true = {cosf(theta_true / 2.0f), 0.0f, 0.0f,
                      sinf(theta_true / 2.0f)};

    /* Fixed world horizontal field (1, 0, 0), expressed in the body frame
     * under the true attitude: rotate world->body via conjugate(q_true). */
    vec3_t mag_body = rotate(conjugate(q_true), (vec3_t) {1.0f, 0.0f, 0.0f});

    quat_t q = {0};
    for (int i = 0; i < 800; i++) {
        ahrs_update(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, mag_body.x,
                    mag_body.y, mag_body.z, IMU_DT, &q);
    }

    /* Quaternions double-cover rotations (q and -q are the same attitude);
     * compare via the dot product magnitude rather than component-wise. */
    float dot = q.w * q_true.w + q.x * q_true.x + q.y * q_true.y +
                q.z * q_true.z;
    CHECK_NEAR("mag yaw convergence: |dot(q, q_true)|", fabsf(dot), 1.0f,
               1e-2f);
}

/* A steady gyro bias with the sensor genuinely stationary (level accel,
 * fixed mag, zero true angular rate fed in as a constant offset) must be
 * absorbed by the integral term: the attitude should settle rather than
 * drift away indefinitely. */
static void
test_gyro_bias_is_absorbed(void)
{
    ahrs_init();

    const float bias = 0.05f; /* rad/s, constant simulated gyro-bias */
    quat_t q = {0};

    /* Run long enough for the integral term (Ki small) to wind up. */
    for (int i = 0; i < 6000; i++) {
        ahrs_update(bias, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f,
                    IMU_DT, &q);
    }

    vec3_t up_mid = rotate(conjugate(q), (vec3_t) {0.0f, 0.0f, 1.0f});

    for (int i = 0; i < 1000; i++) {
        ahrs_update(bias, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f,
                    IMU_DT, &q);
    }

    vec3_t up_end = rotate(conjugate(q), (vec3_t) {0.0f, 0.0f, 1.0f});

    /* Once settled, further running shouldn't keep moving "up" around -
     * a bias that wasn't being cancelled would show continued drift. */
    CHECK_NEAR("settled attitude stable despite gyro bias: x", up_end.x,
               up_mid.x, 5e-2f);
    CHECK_NEAR("settled attitude stable despite gyro bias: y", up_end.y,
               up_mid.y, 5e-2f);
    CHECK_NEAR("settled attitude stable despite gyro bias: z", up_end.z,
               up_mid.z, 5e-2f);
}

int
main(void)
{
    test_pure_gyro_integration_matches_closed_form();
    test_accel_only_converges_to_level();
    test_mag_correction_resolves_yaw();
    test_gyro_bias_is_absorbed();

    if (s_failures == 0) {
        printf("test_ahrs: all tests passed\n");
        return 0;
    }

    fprintf(stderr, "test_ahrs: %d failure(s)\n", s_failures);
    return 1;
}
