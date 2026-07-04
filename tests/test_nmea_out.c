/**
 * @file    test_nmea_out.c
 * @brief   Host tests for nmea_out.c: exact golden-string checks against
 *          hand-verified checksums, plus buffer-too-small handling.
 *
 * Golden strings were derived by hand from the conversion formulas in
 * nmea_out.c (documented per-field in nmea_out.h) and cross-checked with
 * an independent script computing the XOR checksum only - the mechanical,
 * spec-defined part - never by running the formatter and copying its
 * output.
 */

#include <stdio.h>
#include <string.h>

#include "gps/ubx.h"
#include "nmea/nmea_out.h"

static int g_failures = 0;

#define CHECK_STR_EQ(desc, actual, expected)                                 \
    do {                                                                     \
        if (strcmp((actual), (expected)) != 0) {                             \
            printf("FAIL %s\n  expected: %s\n  actual:   %s\n", (desc),      \
                   (expected), (actual));                                    \
            g_failures++;                                                    \
        } else {                                                             \
            printf("PASS %s\n", (desc));                                     \
        }                                                                    \
    } while (0)

#define CHECK_SIZE_EQ(desc, actual, expected)                                \
    do {                                                                     \
        if ((size_t)(actual) != (size_t)(expected)) {                        \
            printf("FAIL %s\n  expected len: %zu\n  actual len:   %zu\n",     \
                   (desc), (size_t)(expected), (size_t)(actual));            \
            g_failures++;                                                    \
        } else {                                                             \
            printf("PASS %s\n", (desc));                                     \
        }                                                                    \
    } while (0)

/*
 * Fixture 1: London-ish, 3D+RTK-capable fields but plain GPS fix
 * (carrSoln = 0), lat 51.5074000 N, lon 0.1278000 W.
 *
 * Hand-verified derivations:
 *   lat  51.5074000 deg -> 51 deg + 0.5074000*60 = 30.4440 min
 *                       -> "5130.4440,N"
 *   lon   0.1278000 deg -> 0 deg   + 0.1278000*60 =  7.6680 min
 *                       -> "00007.6680,W" (negative -> W)
 *   time  itow_ms=123456780 -> centisec = (123456780 % 1000)/10 = 78
 *                       -> "123456.78"
 *   date  15/06/2026        -> "150626"
 *   fix quality: gnssFixOK=1, carrSoln=0 -> 1
 *   HDOP: pdop_1e2=150      -> "1.50"
 *   alt:  hmsl_mm=45678     -> (45678+50)/100 = 457 -> "45.7"
 *   speed knots: gspeed_mms=4630 -> 4630*9/4630 = 9.000 exactly
 *   speed km/h:  4630*9/2500      = 16.668 exactly
 *   course: head_motion_1e5=9050000 -> 90.50 deg -> "090.5"
 */
static ubx_nav_pvt_t make_fixture_1(void)
{
    ubx_nav_pvt_t pvt;
    memset(&pvt, 0, sizeof(pvt));

    pvt.itow_ms = 123456780U;
    pvt.year = 2026U;
    pvt.month = 6U;
    pvt.day = 15U;
    pvt.hour = 12U;
    pvt.min = 34U;
    pvt.sec = 56U;
    pvt.valid = 0x07U;
    pvt.fix_type = 3U;
    pvt.flags = 0x01U; /* gnssFixOK=1, carrSoln=0 */
    pvt.num_sv = 12U;
    pvt.lat_1e7 = 515074000;
    pvt.lon_1e7 = -1278000;
    pvt.height_mm = 46500;
    pvt.hmsl_mm = 45678;
    pvt.hacc_mm = 1000U;
    pvt.vacc_mm = 1500U;
    pvt.gspeed_mms = 4630;
    pvt.head_motion_1e5 = 9050000;
    pvt.sacc_mms = 100U;
    pvt.headacc_1e5 = 100000U;
    pvt.pdop_1e2 = 150U;

    return pvt;
}

/*
 * Fixture 2: Sydney-ish, no fix, all-zero time/date/speed - exercises
 * the S/E hemisphere branches, status 'V', fix quality 0 and the
 * all-zero numeric formatting paths.
 *
 * Hand-verified derivations:
 *   lat -33.8688000 deg -> 33 deg + 0.8688000*60 = 52.1280 min
 *                       -> "3352.1280,S"
 *   lon 151.2093000 deg -> 151 deg + 0.2093000*60 = 12.5580 min
 *                       -> "15112.5580,E"
 */
static ubx_nav_pvt_t make_fixture_2(void)
{
    ubx_nav_pvt_t pvt;
    memset(&pvt, 0, sizeof(pvt));

    pvt.itow_ms = 0U;
    pvt.year = 2000U;
    pvt.month = 1U;
    pvt.day = 1U;
    pvt.hour = 0U;
    pvt.min = 0U;
    pvt.sec = 0U;
    pvt.valid = 0U;
    pvt.fix_type = 0U;
    pvt.flags = 0x00U; /* gnssFixOK=0 -> no fix */
    pvt.num_sv = 0U;
    pvt.lat_1e7 = -338688000;
    pvt.lon_1e7 = 1512093000;
    pvt.hmsl_mm = 0;
    pvt.gspeed_mms = 0;
    pvt.head_motion_1e5 = 0;
    pvt.pdop_1e2 = 0U;

    return pvt;
}

static void test_fixture_1(void)
{
    ubx_nav_pvt_t pvt = make_fixture_1();
    char buf[128];
    size_t len;

    len = nmea_format_gga(&pvt, buf, sizeof(buf));
    CHECK_SIZE_EQ("fixture1 GGA length", len, 71U);
    CHECK_STR_EQ(
        "fixture1 GGA",
        buf,
        "$GPGGA,123456.78,5130.4440,N,00007.6680,W,1,12,1.50,45.7,M,0.0,M,,"
        "*4B\r\n");

    len = nmea_format_rmc(&pvt, buf, sizeof(buf));
    CHECK_SIZE_EQ("fixture1 RMC length", len, 69U);
    CHECK_STR_EQ(
        "fixture1 RMC",
        buf,
        "$GPRMC,123456.78,A,5130.4440,N,00007.6680,W,9.000,090.5,150626,,"
        "*26\r\n");

    len = nmea_format_vtg(&pvt, buf, sizeof(buf));
    CHECK_SIZE_EQ("fixture1 VTG length", len, 44U);
    CHECK_STR_EQ("fixture1 VTG", buf,
                 "$GPVTG,090.5,T,090.5,M,9.000,N,16.668,K*78\r\n");
}

static void test_fixture_2(void)
{
    ubx_nav_pvt_t pvt = make_fixture_2();
    char buf[128];
    size_t len;

    len = nmea_format_gga(&pvt, buf, sizeof(buf));
    CHECK_SIZE_EQ("fixture2 GGA length", len, 70U);
    CHECK_STR_EQ(
        "fixture2 GGA", buf,
        "$GPGGA,000000.00,3352.1280,S,15112.5580,E,0,00,0.00,0.0,M,0.0,M,,"
        "*72\r\n");

    len = nmea_format_rmc(&pvt, buf, sizeof(buf));
    CHECK_SIZE_EQ("fixture2 RMC length", len, 69U);
    CHECK_STR_EQ(
        "fixture2 RMC", buf,
        "$GPRMC,000000.00,V,3352.1280,S,15112.5580,E,0.000,000.0,010100,,"
        "*3B\r\n");

    len = nmea_format_vtg(&pvt, buf, sizeof(buf));
    CHECK_SIZE_EQ("fixture2 VTG length", len, 43U);
    CHECK_STR_EQ("fixture2 VTG", buf,
                 "$GPVTG,000.0,T,000.0,M,0.000,N,0.000,K*4E\r\n");
}

/* Buffer-too-small: fixture 1's GGA needs exactly 72 bytes (71 visible
 * chars + NUL). One byte short must fail cleanly (return 0); exactly
 * enough must succeed. */
static void test_buffer_too_small(void)
{
    ubx_nav_pvt_t pvt = make_fixture_1();
    char buf[128];
    size_t len;

    memset(buf, 'X', sizeof(buf));
    len = nmea_format_gga(&pvt, buf, 71U); /* one byte short of 72 */
    CHECK_SIZE_EQ("GGA too-small buffer returns 0", len, 0U);

    len = nmea_format_gga(&pvt, buf, 72U); /* exactly enough */
    CHECK_SIZE_EQ("GGA exact-size buffer succeeds", len, 71U);

    len = nmea_format_rmc(&pvt, buf, 5U); /* far too small */
    CHECK_SIZE_EQ("RMC far-too-small buffer returns 0", len, 0U);

    len = nmea_format_vtg(&pvt, buf, 0U); /* zero-size buffer */
    CHECK_SIZE_EQ("VTG zero-size buffer returns 0", len, 0U);
}

int main(void)
{
    test_fixture_1();
    test_fixture_2();
    test_buffer_too_small();

    if (g_failures == 0) {
        printf("All tests passed.\n");
        return 0;
    }

    printf("%d test(s) failed.\n", g_failures);
    return 1;
}
