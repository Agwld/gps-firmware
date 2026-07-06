/**
 * @file    test_mag_cal.c
 * @brief   Host tests for imu/mag_cal.c.
 */

#include "imu/mag_cal.h"

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

#define MAG_CAL_TEST_N_THETA 40
#define MAG_CAL_TEST_N_PHI   20

static const float k_true_bias[3] = {10.0f, -5.0f, 20.0f};
static const float k_true_gain[3] = {1.0f, 1.5f, 0.7f}; /* sensor's own gain */
static const float k_field_ut = 50.0f;

/* A dense theta/phi grid (including the poles, phi=0 and phi=pi) so the
 * min/max estimator this calibration relies on actually sees the true
 * extremes of each axis, not just a sparse/uneven sample of the sphere. */
static void
sample_raw(int i, int j, float *raw_x, float *raw_y, float *raw_z)
{
    float theta = (float) i / (float) MAG_CAL_TEST_N_THETA * 2.0f *
                  (float) M_PI;
    float phi =
        (float) j / (float) (MAG_CAL_TEST_N_PHI - 1) * (float) M_PI;

    float x = k_field_ut * sinf(phi) * cosf(theta);
    float y = k_field_ut * sinf(phi) * sinf(theta);
    float z = k_field_ut * cosf(phi);

    /* Sensor reads the true field distorted by its own gain, plus a
     * constant hard-iron offset. */
    *raw_x = x * k_true_gain[0] + k_true_bias[0];
    *raw_y = y * k_true_gain[1] + k_true_bias[1];
    *raw_z = z * k_true_gain[2] + k_true_bias[2];
}

/* Simulate a magnetometer with a known hard-iron bias and per-axis
 * soft-iron gain, fed a full sphere of orientations of a fixed-magnitude
 * true field. Calibration must recover the bias and normalise the
 * per-axis range back to equal. */
static void
test_recovers_known_bias_and_scale(void)
{
    mag_cal_accumulator_t acc;
    mag_cal_start(&acc);

    for (int i = 0; i < MAG_CAL_TEST_N_THETA; i++) {
        for (int j = 0; j < MAG_CAL_TEST_N_PHI; j++) {
            float raw_x, raw_y, raw_z;
            sample_raw(i, j, &raw_x, &raw_y, &raw_z);
            mag_cal_feed(&acc, raw_x, raw_y, raw_z);
        }
    }

    mag_cal_result_t cal;
    status_t st = mag_cal_finish(&acc, &cal);
    if (st != STATUS_OK) {
        fprintf(stderr, "FAIL: mag_cal_finish did not return STATUS_OK\n");
        s_failures++;
        return;
    }

    CHECK_NEAR("recovered bias x", cal.bias[0], k_true_bias[0], 0.5f);
    CHECK_NEAR("recovered bias y", cal.bias[1], k_true_bias[1], 0.5f);
    CHECK_NEAR("recovered bias z", cal.bias[2], k_true_bias[2], 0.5f);

    /* After calibration, the same sweep of raw samples should map back
     * to (near-)equal per-axis range, centred on zero. */
    float cmin[3] = {1e9f, 1e9f, 1e9f};
    float cmax[3] = {-1e9f, -1e9f, -1e9f};
    for (int i = 0; i < MAG_CAL_TEST_N_THETA; i++) {
        for (int j = 0; j < MAG_CAL_TEST_N_PHI; j++) {
            float raw_x, raw_y, raw_z;
            sample_raw(i, j, &raw_x, &raw_y, &raw_z);

            float cx, cy, cz;
            mag_cal_apply(&cal, raw_x, raw_y, raw_z, &cx, &cy, &cz);

            float v[3] = {cx, cy, cz};
            for (int a = 0; a < 3; a++) {
                if (v[a] < cmin[a]) {
                    cmin[a] = v[a];
                }
                if (v[a] > cmax[a]) {
                    cmax[a] = v[a];
                }
            }
        }
    }

    float range0 = cmax[0] - cmin[0];
    float range1 = cmax[1] - cmin[1];
    float range2 = cmax[2] - cmin[2];
    CHECK_NEAR("calibrated range x == y", range0, range1, 1.0f);
    CHECK_NEAR("calibrated range y == z", range1, range2, 1.0f);
    CHECK_NEAR("calibrated midpoint x", 0.5f * (cmax[0] + cmin[0]), 0.0f,
               0.5f);
}

/* Too few samples, or samples that never rotated (near-zero range),
 * must be rejected rather than producing a bogus calibration. */
static void
test_rejects_insufficient_data(void)
{
    mag_cal_accumulator_t acc;
    mag_cal_result_t cal;

    mag_cal_start(&acc);
    for (int i = 0; i < 5; i++) {
        mag_cal_feed(&acc, 10.0f, 10.0f, 10.0f);
    }
    if (mag_cal_finish(&acc, &cal) == STATUS_OK) {
        fprintf(stderr, "FAIL: accepted calibration from too few samples\n");
        s_failures++;
    }

    mag_cal_start(&acc);
    for (int i = 0; i < 200; i++) {
        mag_cal_feed(&acc, 10.0f, 10.0f, 10.0f); /* never moves */
    }
    if (mag_cal_finish(&acc, &cal) == STATUS_OK) {
        fprintf(stderr, "FAIL: accepted calibration with zero range\n");
        s_failures++;
    }
}

static void
test_identity_is_a_no_op(void)
{
    mag_cal_result_t cal;
    mag_cal_identity(&cal);

    float x, y, z;
    mag_cal_apply(&cal, 3.0f, -4.0f, 5.0f, &x, &y, &z);

    CHECK_NEAR("identity: x", x, 3.0f, 1e-6f);
    CHECK_NEAR("identity: y", y, -4.0f, 1e-6f);
    CHECK_NEAR("identity: z", z, 5.0f, 1e-6f);
}

/* Simulate a level car yawing through `revs` full turns, `pts` samples
 * each: the horizontal field traces a circle in the vehicle x/y plane
 * (distorted by hard-iron bias + per-axis gain), while the vertical axis
 * sees a constant field - exactly the case that a full-sphere min/max
 * can't finish but the continuous horizontal calibrator must. */
static void
feed_level_revolutions(mag_cal_cont_t *c, const float bias[3],
                       float gain_x, float gain_y, int pts, int revs)
{
    const float H = 40.0f; /* horizontal field magnitude, uT */
    const float Z = 30.0f; /* vertical component (constant when level) */
    for (int r = 0; r < revs; r++) {
        for (int i = 0; i < pts; i++) {
            float psi = (float) i / (float) pts * 2.0f * (float) M_PI;
            float raw_x = H * cosf(psi) * gain_x + bias[0];
            float raw_y = H * sinf(psi) * gain_y + bias[1];
            float raw_z = Z + bias[2];
            mag_cal_cont_feed(c, raw_x, raw_y, raw_z);
        }
    }
}

/* Driving a level car in circles must reach GOOD and recover the
 * horizontal hard-iron bias, even though the vertical axis never moves. */
static void
test_cont_good_from_level_driving(void)
{
    const float bias[3] = {8.0f, -6.0f, 15.0f};
    mag_cal_result_t identity;
    mag_cal_identity(&identity);

    mag_cal_cont_t c;
    mag_cal_cont_init(&c, &identity);
    if (mag_cal_cont_quality(&c) != MAG_CAL_UNCALIBRATED) {
        fprintf(stderr, "FAIL: identity seed should start UNCALIBRATED\n");
        s_failures++;
    }

    feed_level_revolutions(&c, bias, 1.0f, 1.3f, 200, 2);

    if (mag_cal_cont_quality(&c) < MAG_CAL_GOOD) {
        fprintf(stderr, "FAIL: level driving did not reach GOOD\n");
        s_failures++;
    }
    if (!mag_cal_cont_take_dirty(&c)) {
        fprintf(stderr, "FAIL: reaching GOOD should mark dirty\n");
        s_failures++;
    }
    if (mag_cal_cont_take_dirty(&c)) {
        fprintf(stderr, "FAIL: dirty should clear after being taken\n");
        s_failures++;
    }

    const mag_cal_result_t *cal = mag_cal_cont_result(&c);
    CHECK_NEAR("cont recovered bias x", cal->bias[0], bias[0], 1.0f);
    CHECK_NEAR("cont recovered bias y", cal->bias[1], bias[1], 1.0f);
    /* Vertical axis uncalibratable from level driving: left at identity. */
    CHECK_NEAR("cont vertical bias untouched", cal->bias[2], 0.0f, 1e-6f);
    CHECK_NEAR("cont vertical scale untouched", cal->scale[2], 1.0f, 1e-6f);

    /* Horizontal scales equalise the per-axis gain (gy was 1.3x gx). */
    float cx0, cy0, cz0, cx1, cy1, cz1;
    mag_cal_apply(cal, 40.0f + bias[0], bias[1], 0.0f, &cx0, &cy0, &cz0);
    mag_cal_apply(cal, bias[0], 40.0f * 1.3f + bias[1], 0.0f, &cx1, &cy1,
                  &cz1);
    CHECK_NEAR("cont equalised x/y radius", cx0, cy1, 1.0f);
}

/* A non-identity calibration restored from flash is trusted immediately. */
static void
test_cont_flash_seed_starts_good(void)
{
    mag_cal_result_t seed = {{5.0f, 5.0f, 5.0f}, {1.1f, 1.1f, 1.1f}};
    mag_cal_cont_t c;
    mag_cal_cont_init(&c, &seed);
    if (mag_cal_cont_quality(&c) != MAG_CAL_GOOD) {
        fprintf(stderr, "FAIL: flash-seeded cal should start GOOD\n");
        s_failures++;
    }
    if (mag_cal_cont_take_dirty(&c)) {
        fprintf(stderr, "FAIL: a fresh seed should not be dirty\n");
        s_failures++;
    }
}

/* Course cross-check: a heading that tracks course with a constant offset
 * reaches VALIDATED; a heading uncorrelated with course never does. */
static void
test_cont_course_validation(void)
{
    const float bias[3] = {8.0f, -6.0f, 15.0f};
    mag_cal_result_t identity;
    mag_cal_identity(&identity);

    mag_cal_cont_t good;
    mag_cal_cont_init(&good, &identity);
    feed_level_revolutions(&good, bias, 1.0f, 1.3f, 200, 2);

    /* Constant 0.5 rad offset (declination + mounting) -> validates. */
    for (int i = 0; i < 200; i++) {
        float yaw = (float) i * 0.11f;
        mag_cal_cont_observe_heading(&good, yaw, yaw + 0.5f);
    }
    if (mag_cal_cont_quality(&good) != MAG_CAL_VALIDATED) {
        fprintf(stderr, "FAIL: constant-offset heading should VALIDATE\n");
        s_failures++;
    }

    /* Uncorrelated heading vs course -> stays GOOD, never validates. */
    mag_cal_cont_t bad;
    mag_cal_cont_init(&bad, &identity);
    feed_level_revolutions(&bad, bias, 1.0f, 1.3f, 200, 2);
    for (int i = 0; i < 500; i++) {
        float yaw = fmodf((float) i * 0.7f, 2.0f * (float) M_PI);
        float course = fmodf((float) i * 2.3f, 2.0f * (float) M_PI);
        mag_cal_cont_observe_heading(&bad, yaw, course);
    }
    if (mag_cal_cont_quality(&bad) == MAG_CAL_VALIDATED) {
        fprintf(stderr, "FAIL: uncorrelated heading must not VALIDATE\n");
        s_failures++;
    }
}

int
main(void)
{
    test_recovers_known_bias_and_scale();
    test_rejects_insufficient_data();
    test_identity_is_a_no_op();
    test_cont_good_from_level_driving();
    test_cont_flash_seed_starts_good();
    test_cont_course_validation();

    if (s_failures == 0) {
        printf("test_mag_cal: all tests passed\n");
        return 0;
    }

    fprintf(stderr, "test_mag_cal: %d failure(s)\n", s_failures);
    return 1;
}
