/**
 * @file    test_airdata.c
 * @brief   Host tests for fusion/airdata.c: density, channel averaging,
 *          Bernoulli airspeed, standstill tare, and the wind / scale EKF
 *          converging on a simulated session.
 */

#include "fusion/airdata.h"

#include <math.h>
#include <stdio.h>

static int s_failures = 0;

#define CHECK_NEAR(desc, actual, expected, tol)                             \
    do {                                                                    \
        double diff_ = fabs((double) (actual) - (double) (expected));      \
        if (diff_ > (double) (tol)) {                                       \
            fprintf(stderr, "FAIL: %s: got %f, expected %f (tol %f)\n",     \
                    (desc), (double) (actual), (double) (expected),         \
                    (double) (tol));                                        \
            s_failures++;                                                   \
        }                                                                    \
    } while (0)

#define CHECK(desc, cond)                                                   \
    do {                                                                    \
        if (!(cond)) {                                                      \
            fprintf(stderr, "FAIL: %s\n", (desc));                          \
            s_failures++;                                                   \
        }                                                                    \
    } while (0)

#define PI_F 3.14159265358979f
#define DT_S 0.01f /* P4-A frame period */

/* Deterministic pseudo-noise in [-1, 1]. */
static uint32_t s_rng = 12345U;
static float
noise(void)
{
    s_rng = s_rng * 1664525U + 1013904223U;
    return (float) (s_rng >> 8) / (float) (1U << 23) - 1.0f;
}

/* The P4-A reports in 0.01 mbar (1 Pa) steps. */
static float
quantise_mbar(float mbar)
{
    return floorf(mbar * 100.0f + 0.5f) / 100.0f;
}

static void
test_density_and_mean(void)
{
    CHECK_NEAR("density at 1013.25 mbar, 15 C", airdata_density(1013.25f, 15.0f),
               1.225f, 0.001f);
    CHECK_NEAR("density at 1000 mbar, 20 C", airdata_density(1000.0f, 20.0f),
               1.1885f, 0.001f);
    CHECK_NEAR("absent sensor falls back to standard air",
               airdata_density(0.0f, 20.0f), 1.225f, 1e-6f);

    float ch[4] = {999.9f, 999.4f, 1003.0f, 999.5f};
    CHECK_NEAR("mean of CH1, CH2, CH4", airdata_mean_channels(ch, 0x0BU),
               (999.9f + 999.4f + 999.5f) / 3.0f, 1e-4f);
    CHECK_NEAR("single channel", airdata_mean_channels(ch, 0x04U), 1003.0f,
               1e-4f);
    CHECK_NEAR("empty mask", airdata_mean_channels(ch, 0x00U), 0.0f, 1e-6f);
}

/* One update with explicit pressures and ground velocity. */
static void
step(airdata_t *a, float p_total, float p_static, float ve, float vn,
     bool ground_valid)
{
    airdata_input_t in = {p_total, p_static, 20.0f, true, ve, vn,
                          ground_valid, DT_S};
    airdata_update(a, &in);
}

static void
test_tare_removes_channel_offset(void)
{
    airdata_t a;
    airdata_init(&a);

    /* Total channel reads 0.5 mbar (50 Pa) high at rest: untared, that's a
     * phantom ~9 m/s. */
    const float offset = 0.5f;
    for (int i = 0; i < 50; i++) { /* 0.5 s: not tared yet */
        step(&a, 1000.0f + offset, 1000.0f, 0.0f, 0.0f, true);
    }
    CHECK("not tared after 0.5 s", !a.tared);
    CHECK_NEAR("no airspeed reported before the tare",
               airdata_airspeed_mps(&a), 0.0f, 1e-6f);

    for (int i = 0; i < 500; i++) { /* 5 s stationary */
        step(&a, 1000.0f + offset, 1000.0f, 0.0f, 0.0f, true);
    }
    CHECK("tared after a 5 s stop", a.tared);
    CHECK_NEAR("tared airspeed at rest", airdata_airspeed_mps(&a), 0.0f, 0.5f);

    /* Now moving: 250 Pa of real dynamic pressure on top of the offset. */
    step(&a, 1000.0f + offset + 2.5f, 1000.0f, 20.0f, 0.0f, true);
    float expected = sqrtf(2.0f * 250.0f / airdata_density(1000.0f, 20.0f));
    /* Indicated (Bernoulli) airspeed: the corrected output also divides
     * by the scale factor, which this first moving sample already nudges. */
    CHECK_NEAR("airspeed with the offset removed", a.indicated_mps, expected,
               0.05f);

    /* No tare without a valid ground velocity (can't tell it's stopped). */
    airdata_t b;
    airdata_init(&b);
    for (int i = 0; i < 500; i++) {
        step(&b, 1000.5f, 1000.0f, 0.0f, 0.0f, false);
    }
    CHECK("no tare without GPS-aided velocity", !b.tared);
}

/* A whole session: 5 s parked (sheltered, no wind) to tare, then laps of a
 * 60 m-radius circle with the speed swinging between 12 and 26 m/s, in a
 * steady wind, with a pitot that over-reads by k_true, channel offsets,
 * 1 Pa quantisation and gusty noise. The EKF must recover wind and k. */
static void
test_wind_and_scale_converge(void)
{
    const float k_true = 1.07f;
    const float wind_e = 3.0f, wind_n = -2.0f;
    const float p_amb = 1001.3f;
    const float rho = airdata_density(p_amb, 20.0f);
    /* Per-channel zero errors (mbar): total, then the three statics. */
    const float off_total = 0.42f;
    const float off_static[3] = {0.08f, -0.45f, -0.15f};

    airdata_t a;
    airdata_init(&a);

    float theta = 0.0f;
    const float radius = 60.0f;
    const int steps = (int) (420.0f / DT_S); /* 7 minutes */

    bool parked_ok = true;
    for (int i = 0; i < steps; i++) {
        float t = (float) i * DT_S;
        bool parked = t < 5.0f;

        float speed = parked ? 0.0f : 19.0f + 7.0f * sinf(0.21f * t);
        theta += speed / radius * DT_S;
        /* Counter-clockwise: velocity is the circle's tangent. */
        float ve = -speed * sinf(theta);
        float vn = speed * cosf(theta);

        float we = parked ? 0.0f : wind_e;
        float wn = parked ? 0.0f : wind_n;
        float rel = sqrtf((ve - we) * (ve - we) + (vn - wn) * (vn - wn));
        float indicated = k_true * rel * (1.0f + 0.02f * noise());
        float q_pa = 0.5f * rho * indicated * indicated;

        float ch[4];
        ch[0] = quantise_mbar(p_amb + off_static[0]);
        ch[1] = quantise_mbar(p_amb + off_static[1]);
        ch[2] = quantise_mbar(p_amb + off_total + q_pa / 100.0f);
        ch[3] = quantise_mbar(p_amb + off_static[2]);

        airdata_input_t in = {ch[2], airdata_mean_channels(ch, 0x0BU), 20.0f,
                              true, ve, vn, true, DT_S};
        airdata_update(&a, &in);

        if (parked && t > 4.0f && airdata_airspeed_mps(&a) > 0.5f) {
            parked_ok = false;
        }
    }

    CHECK("airspeed ~0 while parked once tared", parked_ok);
    CHECK("wind estimate converged", airdata_wind_valid(&a));

    float we_est, wn_est;
    airdata_wind(&a, &we_est, &wn_est);
    printf("wind (%.2f, %.2f) m/s (truth %.2f, %.2f), k %.3f (truth %.3f)\n",
           (double) we_est, (double) wn_est, (double) wind_e, (double) wind_n,
           (double) airdata_scale(&a), (double) k_true);
    CHECK_NEAR("wind east", we_est, wind_e, 0.4f);
    CHECK_NEAR("wind north", wn_est, wind_n, 0.4f);
    CHECK_NEAR("scale factor", airdata_scale(&a), k_true, 0.02f);

    /* The corrected airspeed now matches the true air-relative speed. */
    float ve = 20.0f, vn = 0.0f;
    float rel = sqrtf((ve - wind_e) * (ve - wind_e) + (vn - wind_n) * (vn - wind_n));
    float q_pa = 0.5f * rho * (k_true * rel) * (k_true * rel);
    float ch[4] = {p_amb + off_static[0], p_amb + off_static[1],
                   p_amb + off_total + q_pa / 100.0f, p_amb + off_static[2]};
    airdata_input_t in = {ch[2], airdata_mean_channels(ch, 0x0BU), 20.0f, true,
                          ve, vn, true, DT_S};
    airdata_update(&a, &in);
    CHECK_NEAR("corrected airspeed", airdata_airspeed_mps(&a), rel, 0.3f);
}

/* With no ground velocity the wind must not move (nothing to fit to). */
static void
test_no_fit_without_ground_velocity(void)
{
    airdata_t a;
    airdata_init(&a);
    for (int i = 0; i < 500; i++) { /* tare */
        step(&a, 1000.0f, 1000.0f, 0.0f, 0.0f, true);
    }
    for (int i = 0; i < 3000; i++) {
        step(&a, 1003.0f, 1000.0f, 0.0f, 0.0f, false);
    }
    float we, wn;
    airdata_wind(&a, &we, &wn);
    CHECK_NEAR("wind east unchanged", we, 0.0f, 1e-6f);
    CHECK_NEAR("wind north unchanged", wn, 0.0f, 1e-6f);
    CHECK("not reported as converged", !airdata_wind_valid(&a));
}

int
main(void)
{
    test_density_and_mean();
    test_tare_removes_channel_offset();
    test_wind_and_scale_converge();
    test_no_fit_without_ground_velocity();

    if (s_failures == 0) {
        printf("test_airdata: all tests passed\n");
        return 0;
    }
    fprintf(stderr, "test_airdata: %d failure(s)\n", s_failures);
    return 1;
}
