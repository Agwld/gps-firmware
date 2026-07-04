/**
 * @file    test_kf6.c
 * @brief   Host tests for fusion/kf6.c.
 */

#include "fusion/kf6.h"

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

#define IMU_DT (1.0f / 104.0f)
#define GRAVITY 9.80665f

static const quat_t LEVEL_Q = {1.0f, 0.0f, 0.0f, 0.0f};

/* A body-frame accel reading that, once gravity is removed, is exactly
 * zero net acceleration for a level vehicle (accelerometer reads +g up
 * at rest or at any constant velocity). */
static void
predict_zero_accel(uint32_t tick)
{
    kf6_predict(tick, 0.0f, 0.0f, GRAVITY, LEVEL_Q, IMU_DT);
}

/* Pure dead reckoning: seed a velocity via a confident GPS velocity fix,
 * then predict with zero net accel for N steps and check position tracks
 * v * t exactly (no process noise realisation in this deterministic
 * test - only its mean, which should follow the constant-velocity
 * kinematics exactly). */
static void
test_predict_tracks_constant_velocity(void)
{
    kf6_init();

    kf6_correct_vel(0U, 20.0f, 0.0f, 0.0f, 0.01f);

    const int steps = 50;
    for (int i = 1; i <= steps; i++) {
        predict_zero_accel((uint32_t) i);
    }

    float e, n, u, ve, vn, vu;
    kf6_get_state(&e, &n, &u, &ve, &vn, &vu);

    float expected_e = 20.0f * (float) steps * IMU_DT;
    CHECK_NEAR("constant velocity: east position", e, expected_e, 1e-3f);
    CHECK_NEAR("constant velocity: north position", n, 0.0f, 1e-3f);
    CHECK_NEAR("constant velocity: east velocity", ve, 20.0f, 1e-3f);
}

/* An immediate (undelayed) position fix, with the filter's prior far
 * less confident than the measurement, should pull the state close to
 * the fix. */
static void
test_immediate_position_correction(void)
{
    kf6_init();

    kf6_correct_pos(0U, 50.0f, -10.0f, 2.0f, 1.0f);

    float e, n, u;
    kf6_get_state(&e, &n, &u, NULL, NULL, NULL);

    CHECK_NEAR("immediate correction: east", e, 50.0f, 1.0f);
    CHECK_NEAR("immediate correction: north", n, -10.0f, 1.0f);
    CHECK_NEAR("immediate correction: up", u, 2.0f, 1.0f);
}

/*
 * The delayed-state rewind, exercised end to end.
 *
 * Scenario: the filter has accumulated a pure position bias (as if from
 * unmodelled drift) while truth moves at a constant 20 m/s east. A GPS
 * fix arrives describing the *true* position from 10 predict steps ago
 * (~96 ms of latency at 104 Hz - within the F9P's typical 20-60 ms plus
 * margin). Applying that fix correctly must:
 *   (a) remove the bias from the CURRENT state, landing near the true
 *       *current* position (truth propagated forward by the 10 steps'
 *       worth of travel since the fix's epoch) - not the fix's own
 *       (now-stale) position, and
 *   (b) leave velocity close to unchanged - not exactly untouched, since
 *       10 steps of consistent dynamics legitimately build up a small
 *       pos/vel cross-covariance (a real KF property, from F P F^T) that
 *       a position correction partially projects onto velocity too; the
 *       bias here is kept small enough (2 m, a plausible drift over
 *       ~100 ms) that this effect stays within the test's tolerance.
 *
 * A naive implementation that applied the same measurement directly to
 * the current state (ignoring its age) would instead land near the
 * fix's stale position - short by v * latency (~1.9 m here) - which is
 * exactly what check (a) below distinguishes from the correct answer.
 */
static void
test_delayed_correction_rewinds_and_replays(void)
{
    kf6_init();

    const float v_true = 20.0f; /* m/s east, truth throughout */
    const float bias_m = 2.0f; /* filter's accumulated east-position error */

    /* Seed velocity and inject the position bias before the simulated
     * trajectory starts (tick 0, no history yet - direct correction).
     * Sigma here is deliberately moderate (not near-zero): an
     * unrealistically tight seed would leave the filter's covariance too
     * small for the later, genuinely accurate fix to move it - exactly
     * the correct KF behaviour for an overconfident prior, but not what
     * this test is trying to exercise (a normal-confidence prior meeting
     * a normal-confidence correction). */
    kf6_correct_vel(0U, v_true, 0.0f, 0.0f, 2.0f);
    kf6_correct_pos(0U, bias_m, 0.0f, 0.0f, 2.0f);

    const int fix_step = 10;  /* GPS fix describes truth as of this step */
    const int total_steps = 20; /* current time: 10 steps later */

    for (int i = 1; i <= total_steps; i++) {
        predict_zero_accel((uint32_t) i);
    }

    /* Sanity: before correction, the filter is indeed off by ~bias_m
     * (confirms the scenario actually set up a divergent filter, not a
     * no-op). */
    float e_before;
    kf6_get_state(&e_before, NULL, NULL, NULL, NULL, NULL);
    float true_pos_now = v_true * (float) total_steps * IMU_DT;
    CHECK_NEAR("pre-correction state carries the injected bias",
               e_before - true_pos_now, bias_m, 1e-2f);

    /* GPS fix: the TRUE (unbiased) position at fix_step's epoch. */
    float true_pos_at_fix = v_true * (float) fix_step * IMU_DT;
    kf6_correct_pos((uint32_t) fix_step, true_pos_at_fix, 0.0f, 0.0f, 0.05f);

    float e_after, ve_after;
    kf6_get_state(&e_after, NULL, NULL, &ve_after, NULL, NULL);

    /* (a): current state lands near the true CURRENT position, not the
     * stale fix's own position. */
    CHECK_NEAR("rewound correction matches true current position", e_after,
               true_pos_now, 0.5f);

    /* Distinguish from the naive (non-rewound) failure mode explicitly:
     * that would have landed near true_pos_at_fix, which is v_true *
     * (total_steps - fix_step) * IMU_DT away from the correct answer -
     * make sure we are NOT there. */
    float naive_wrong_answer = true_pos_at_fix;
    float distance_from_naive =
        (float) fabs((double) e_after - (double) naive_wrong_answer);
    if (distance_from_naive < 0.5f) {
        fprintf(stderr,
                "FAIL: rewind looks like a no-op - state matches the "
                "stale fix (%.3f) rather than the replayed current "
                "position (%.3f)\n",
                (double) naive_wrong_answer, (double) true_pos_now);
        s_failures++;
    }

    /* (b): velocity, which was never wrong, should be undisturbed. */
    CHECK_NEAR("rewound correction leaves velocity intact", ve_after,
               v_true, 0.5f);
}

/* A wheelspeed correction along a known heading should adjust velocity
 * projected onto that heading without perturbing the perpendicular
 * component when the heading is axis-aligned. */
static void
test_speed_correction_projects_onto_heading(void)
{
    kf6_init();

    kf6_correct_vel(0U, 0.0f, 0.0f, 0.0f, 2.0f);

    kf6_correct_speed(0U, 15.0f, 0.0f /* heading = East */, 0.2f);

    float ve, vn;
    kf6_get_state(NULL, NULL, NULL, &ve, &vn, NULL);

    CHECK_NEAR("speed correction: east velocity moves toward speed", ve,
               15.0f, 1.0f);
    CHECK_NEAR("speed correction: north velocity undisturbed", vn, 0.0f,
               1e-3f);
}

int
main(void)
{
    test_predict_tracks_constant_velocity();
    test_immediate_position_correction();
    test_delayed_correction_rewinds_and_replays();
    test_speed_correction_projects_onto_heading();

    if (s_failures == 0) {
        printf("test_kf6: all tests passed\n");
        return 0;
    }

    fprintf(stderr, "test_kf6: %d failure(s)\n", s_failures);
    return 1;
}
