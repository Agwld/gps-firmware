/**
 * @file    test_geodesy.c
 * @brief   Host tests for fusion/geodesy.c.
 */

#include "fusion/geodesy.h"

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

/* Origin used throughout: near Silverstone circuit, altitude ~153 m. */
#define ORIGIN_LAT_DEG 52.0786
#define ORIGIN_LON_DEG (-1.0169)
#define ORIGIN_HEIGHT_M 153.0f

static void
test_origin_returns_zero(void)
{
    geodesy_set_origin(ORIGIN_LAT_DEG, ORIGIN_LON_DEG, ORIGIN_HEIGHT_M);

    float e, n, u;
    geodesy_to_enu(ORIGIN_LAT_DEG, ORIGIN_LON_DEG, ORIGIN_HEIGHT_M, &e, &n,
                   &u);

    CHECK_NEAR("origin east", e, 0.0f, 1e-4f);
    CHECK_NEAR("origin north", n, 0.0f, 1e-4f);
    CHECK_NEAR("origin up", u, 0.0f, 1e-4f);
}

/*
 * Target point derived independently (via a standalone script implementing
 * the same WGS84 meridian/normal radius-of-curvature formulas by hand,
 * *not* by calling this module) as the origin displaced by exactly
 * east=+50 m, north=+30 m, up=+2 m:
 *
 *   lat1 = 52.07886961724705 deg
 *   lon1 = -1.0161706869899274 deg
 *   h1   = 155.0 m
 *
 * geodesy_to_enu() should recover (50, 30, 2) within ~0.5 m per the spec's
 * tolerance for this scale of offset (in practice the WGS84-radius flat
 * plane approximation plus float rounding gives sub-millimetre agreement
 * here, since the offset was derived using the identical closed-form
 * formulas the module implements).
 */
static void
test_known_offset(void)
{
    geodesy_set_origin(ORIGIN_LAT_DEG, ORIGIN_LON_DEG, ORIGIN_HEIGHT_M);

    double lat1 = 52.07886961724705;
    double lon1 = -1.0161706869899274;
    float h1 = 155.0f;

    float e, n, u;
    geodesy_to_enu(lat1, lon1, h1, &e, &n, &u);

    CHECK_NEAR("known offset east", e, 50.0f, 0.5f);
    CHECK_NEAR("known offset north", n, 30.0f, 0.5f);
    CHECK_NEAR("known offset up", u, 2.0f, 1e-4f);
}

/*
 * A second, smaller offset (10 m east only) at the same origin, checking
 * that a pure-longitude displacement doesn't leak into the north axis.
 */
static void
test_pure_east_offset(void)
{
    geodesy_set_origin(ORIGIN_LAT_DEG, ORIGIN_LON_DEG, ORIGIN_HEIGHT_M);

    /* +10 m east at this latitude: m_per_deg_lon ~ 68557.7, so
     * dlon = 10 / 68557.7 ~ 1.458626e-4 deg. */
    double lon1 = ORIGIN_LON_DEG + 1.458626e-4;

    float e, n, u;
    geodesy_to_enu(ORIGIN_LAT_DEG, lon1, ORIGIN_HEIGHT_M, &e, &n, &u);

    CHECK_NEAR("pure east offset east", e, 10.0f, 0.5f);
    CHECK_NEAR("pure east offset north", n, 0.0f, 0.5f);
    CHECK_NEAR("pure east offset up", u, 0.0f, 1e-4f);
}

/* geodesy_from_enu() must invert geodesy_to_enu() exactly (both use the
 * same per-origin linear scale factors, so this isn't testing the WGS84
 * math twice - it's testing that the inverse was wired up correctly). */
static void
test_from_enu_is_inverse(void)
{
    geodesy_set_origin(ORIGIN_LAT_DEG, ORIGIN_LON_DEG, ORIGIN_HEIGHT_M);

    double lat1 = 52.07886961724705;
    double lon1 = -1.0161706869899274;
    float h1 = 155.0f;

    float e, n, u;
    geodesy_to_enu(lat1, lon1, h1, &e, &n, &u);

    double lat2;
    double lon2;
    float h2;
    geodesy_from_enu(e, n, u, &lat2, &lon2, &h2);

    CHECK_NEAR("from_enu inverse: lat", lat2, lat1, 1e-9);
    CHECK_NEAR("from_enu inverse: lon", lon2, lon1, 1e-9);
    CHECK_NEAR("from_enu inverse: height", h2, h1, 1e-4f);
}

int
main(void)
{
    test_origin_returns_zero();
    test_known_offset();
    test_pure_east_offset();
    test_from_enu_is_inverse();

    if (s_failures == 0) {
        printf("test_geodesy: all tests passed\n");
        return 0;
    }

    fprintf(stderr, "test_geodesy: %d failure(s)\n", s_failures);
    return 1;
}
