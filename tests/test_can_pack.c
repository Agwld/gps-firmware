/**
 * @file    test_can_pack.c
 * @brief   Host round-trip tests for canbus/can_pack.c.
 *
 * For every message: pack known engineering values -> bytes -> unpack ->
 * assert values recovered within the scaling resolution's rounding error
 * (1.5x the LSB scale). For GPS_Position and GPS_Velocity, additionally
 * assert the exact byte layout against hand-computed expected arrays, to
 * catch endianness/bit-position mistakes.
 */

#include <stdio.h>
#include <string.h>

#include "canbus/can_defs.h"

static int g_failures = 0;

#define CHECK(cond, fmt, ...)                                          \
    do {                                                               \
        if (!(cond)) {                                                 \
            printf("FAIL %s:%d: " fmt "\n", __func__, __LINE__,        \
                   ##__VA_ARGS__);                                     \
            g_failures++;                                              \
        }                                                              \
    } while (0)

static double fabs_d(double x)
{
    return (x < 0.0) ? -x : x;
}

static float fabs_f(float x)
{
    return (x < 0.0f) ? -x : x;
}

/* ------------------------------------------------------------------ */
/* 0x6B0 GPS_Position                                                  */
/* ------------------------------------------------------------------ */

static void test_gps_position_roundtrip(void)
{
    can_gps_position_t in = { .lat_deg = 51.5074000, .lon_deg = -0.1278000 };
    uint8_t buf[8];
    can_gps_position_t out;

    CHECK(can_pack_gps_position(&in, buf) == STATUS_OK, "pack failed");
    CHECK(can_unpack_gps_position(buf, &out) == STATUS_OK, "unpack failed");

    double tol = 1.5e-7; /* 1.5x LSB (1e-7 deg) */
    CHECK(fabs_d(out.lat_deg - in.lat_deg) <= tol, "lat %.9f vs %.9f",
          out.lat_deg, in.lat_deg);
    CHECK(fabs_d(out.lon_deg - in.lon_deg) <= tol, "lon %.9f vs %.9f",
          out.lon_deg, in.lon_deg);
}

static void test_gps_position_bytes(void)
{
    /* lat=51.5074, lon=-0.1278 -> raw_lat=515074000 (0x1EB367D0),
     * raw_lon=-1278000 (0xFFEC7FD0), little-endian i32 each. */
    can_gps_position_t in = { .lat_deg = 51.5074000, .lon_deg = -0.1278000 };
    uint8_t buf[8];
    const uint8_t expect[8] = { 0xD0, 0x67, 0xB3, 0x1E,
                                 0xD0, 0x7F, 0xEC, 0xFF };

    CHECK(can_pack_gps_position(&in, buf) == STATUS_OK, "pack failed");
    CHECK(memcmp(buf, expect, 8) == 0, "byte mismatch");
}

/* ------------------------------------------------------------------ */
/* 0x6B1 GPS_Velocity                                                  */
/* ------------------------------------------------------------------ */

static void test_gps_velocity_roundtrip(void)
{
    can_gps_velocity_t in = { .speed_mps = 42.15f,
                               .course_deg = 271.30f,
                               .alt_m = 123.4f,
                               .fix_type = 3,
                               .num_sv = 12,
                               .counter = 7 };
    uint8_t buf[8];
    can_gps_velocity_t out;

    CHECK(can_pack_gps_velocity(&in, buf) == STATUS_OK, "pack failed");
    CHECK(can_unpack_gps_velocity(buf, &out) == STATUS_OK, "unpack failed");

    CHECK(fabs_f(out.speed_mps - in.speed_mps) <= 0.015f, "speed %f vs %f",
          out.speed_mps, in.speed_mps);
    CHECK(fabs_f(out.course_deg - in.course_deg) <= 0.015f,
          "course %f vs %f", out.course_deg, in.course_deg);
    CHECK(fabs_f(out.alt_m - in.alt_m) <= 0.15f, "alt %f vs %f", out.alt_m,
          in.alt_m);
    CHECK(out.fix_type == in.fix_type, "fix_type %u vs %u", out.fix_type,
          in.fix_type);
    CHECK(out.num_sv == in.num_sv, "num_sv %u vs %u", out.num_sv,
          in.num_sv);
    CHECK(out.counter == in.counter, "counter %u vs %u", out.counter,
          in.counter);
}

static void test_gps_velocity_bytes(void)
{
    /* speed=42.15 -> 4215 (0x1077), course=271.30 -> 27130 (0x69FA),
     * alt=123.4 -> 1234 (0x04D2), fix=3/numSV=12 -> 0xC3, counter=7. */
    can_gps_velocity_t in = { .speed_mps = 42.15f,
                               .course_deg = 271.30f,
                               .alt_m = 123.4f,
                               .fix_type = 3,
                               .num_sv = 12,
                               .counter = 7 };
    uint8_t buf[8];
    const uint8_t expect[8] = { 0x77, 0x10, 0xFA, 0x69,
                                 0xD2, 0x04, 0xC3, 0x07 };

    CHECK(can_pack_gps_velocity(&in, buf) == STATUS_OK, "pack failed");
    CHECK(memcmp(buf, expect, 8) == 0, "byte mismatch");
}

/* ------------------------------------------------------------------ */
/* 0x6B2 GPS_Attitude                                                  */
/* ------------------------------------------------------------------ */

static void test_gps_attitude_roundtrip(void)
{
    can_gps_attitude_t in = { .yaw_deg = 359.99f,
                               .pitch_deg = -12.34f,
                               .roll_deg = 8.75f,
                               .fusion_status = 2,
                               .counter = 200 };
    uint8_t buf[8];
    can_gps_attitude_t out;

    CHECK(can_pack_gps_attitude(&in, buf) == STATUS_OK, "pack failed");
    CHECK(can_unpack_gps_attitude(buf, &out) == STATUS_OK, "unpack failed");

    CHECK(fabs_f(out.yaw_deg - in.yaw_deg) <= 0.015f, "yaw %f vs %f",
          out.yaw_deg, in.yaw_deg);
    CHECK(fabs_f(out.pitch_deg - in.pitch_deg) <= 0.015f, "pitch %f vs %f",
          out.pitch_deg, in.pitch_deg);
    CHECK(fabs_f(out.roll_deg - in.roll_deg) <= 0.015f, "roll %f vs %f",
          out.roll_deg, in.roll_deg);
    CHECK(out.fusion_status == in.fusion_status, "fusion_status");
    CHECK(out.counter == in.counter, "counter");
}

/* ------------------------------------------------------------------ */
/* 0x6B3 Lap_Status                                                    */
/* ------------------------------------------------------------------ */

static void test_lap_status_roundtrip(void)
{
    can_lap_status_t in = { .lap = 5,
                             .running_time_ms = 123456,
                             .sector = 2,
                             .flags = 0x81 };
    uint8_t buf[8];
    can_lap_status_t out;

    CHECK(can_pack_lap_status(&in, buf) == STATUS_OK, "pack failed");
    CHECK(can_unpack_lap_status(buf, &out) == STATUS_OK, "unpack failed");

    CHECK(out.lap == in.lap, "lap");
    CHECK(out.running_time_ms == in.running_time_ms, "running_time_ms");
    CHECK(out.sector == in.sector, "sector");
    CHECK(out.flags == in.flags, "flags");
}

/* ------------------------------------------------------------------ */
/* 0x6B4 Lap_Event                                                     */
/* ------------------------------------------------------------------ */

static void test_lap_event_roundtrip(void)
{
    can_lap_event_t in = { .type = CAN_LAP_EVENT_TM2,
                            .lap = 3,
                            .time_ms = 987654321U,
                            .counter = 42 };
    uint8_t buf[8];
    can_lap_event_t out;

    CHECK(can_pack_lap_event(&in, buf) == STATUS_OK, "pack failed");
    CHECK(can_unpack_lap_event(buf, &out) == STATUS_OK, "unpack failed");

    CHECK(out.type == in.type, "type");
    CHECK(out.lap == in.lap, "lap");
    CHECK(out.time_ms == in.time_ms, "time_ms");
    CHECK(out.counter == in.counter, "counter");
}

/* ------------------------------------------------------------------ */
/* 0x6B5 GPS_Quality                                                   */
/* ------------------------------------------------------------------ */

static void test_gps_quality_roundtrip(void)
{
    uint8_t flags = (uint8_t)(CAN_GPS_QUALITY_FIX_OK |
                               (2U << CAN_GPS_QUALITY_CARR_SOLN_SHIFT));
    can_gps_quality_t in = { .hacc_mm = 850.0f,
                              .sacc_mm_s = 120.3f,
                              .pdop = 1.87f,
                              .flags = flags };
    uint8_t buf[8];
    can_gps_quality_t out;

    CHECK(can_pack_gps_quality(&in, buf) == STATUS_OK, "pack failed");
    CHECK(can_unpack_gps_quality(buf, &out) == STATUS_OK, "unpack failed");

    CHECK(fabs_f(out.hacc_mm - in.hacc_mm) <= 1.5f, "hacc %f vs %f",
          out.hacc_mm, in.hacc_mm);
    CHECK(fabs_f(out.sacc_mm_s - in.sacc_mm_s) <= 0.15f, "sacc %f vs %f",
          out.sacc_mm_s, in.sacc_mm_s);
    CHECK(fabs_f(out.pdop - in.pdop) <= 0.015f, "pdop %f vs %f", out.pdop,
          in.pdop);
    CHECK(out.flags == in.flags, "flags 0x%02X vs 0x%02X", out.flags,
          in.flags);
}

/* ------------------------------------------------------------------ */
/* 0x6B6 GPS_IMU_Accel                                                 */
/* ------------------------------------------------------------------ */

static void test_gps_imu_accel_roundtrip(void)
{
    can_gps_imu_accel_t in = { .ax_mg = -980.0f,
                                .ay_mg = 15.0f,
                                .az_mg = 1002.0f,
                                .counter = 99 };
    uint8_t buf[8];
    can_gps_imu_accel_t out;

    CHECK(can_pack_gps_imu_accel(&in, buf) == STATUS_OK, "pack failed");
    CHECK(can_unpack_gps_imu_accel(buf, &out) == STATUS_OK,
          "unpack failed");

    CHECK(fabs_f(out.ax_mg - in.ax_mg) <= 1.5f, "ax %f vs %f", out.ax_mg,
          in.ax_mg);
    CHECK(fabs_f(out.ay_mg - in.ay_mg) <= 1.5f, "ay %f vs %f", out.ay_mg,
          in.ay_mg);
    CHECK(fabs_f(out.az_mg - in.az_mg) <= 1.5f, "az %f vs %f", out.az_mg,
          in.az_mg);
    CHECK(out.counter == in.counter, "counter");
}

/* ------------------------------------------------------------------ */
/* 0x6B7 GPS_IMU_Gyro                                                  */
/* ------------------------------------------------------------------ */

static void test_gps_imu_gyro_roundtrip(void)
{
    can_gps_imu_gyro_t in = { .gx_dps = -120.5f,
                               .gy_dps = 3.02f,
                               .gz_dps = 640.0f,
                               .counter = 250 };
    uint8_t buf[8];
    can_gps_imu_gyro_t out;

    CHECK(can_pack_gps_imu_gyro(&in, buf) == STATUS_OK, "pack failed");
    CHECK(can_unpack_gps_imu_gyro(buf, &out) == STATUS_OK, "unpack failed");

    CHECK(fabs_f(out.gx_dps - in.gx_dps) <= 0.09375f, "gx %f vs %f",
          out.gx_dps, in.gx_dps);
    CHECK(fabs_f(out.gy_dps - in.gy_dps) <= 0.09375f, "gy %f vs %f",
          out.gy_dps, in.gy_dps);
    CHECK(fabs_f(out.gz_dps - in.gz_dps) <= 0.09375f, "gz %f vs %f",
          out.gz_dps, in.gz_dps);
    CHECK(out.counter == in.counter, "counter");
}

/* ------------------------------------------------------------------ */
/* 0x6B8 GPS_Temp                                                      */
/* ------------------------------------------------------------------ */

static void test_gps_temp_roundtrip(void)
{
    can_gps_temp_t in = { .mcp9800_temp_c = 23.45f,
                           .imu_temp_c = 31.20f,
                           .mcu_temp_c = -5.60f };
    uint8_t buf[8];
    can_gps_temp_t out;

    CHECK(can_pack_gps_temp(&in, buf) == STATUS_OK, "pack failed");
    CHECK(can_unpack_gps_temp(buf, &out) == STATUS_OK, "unpack failed");

    CHECK(fabs_f(out.mcp9800_temp_c - in.mcp9800_temp_c) <= 0.015f,
          "mcp9800 %f vs %f", out.mcp9800_temp_c, in.mcp9800_temp_c);
    CHECK(fabs_f(out.imu_temp_c - in.imu_temp_c) <= 0.015f,
          "imu %f vs %f", out.imu_temp_c, in.imu_temp_c);
    CHECK(fabs_f(out.mcu_temp_c - in.mcu_temp_c) <= 0.015f,
          "mcu %f vs %f", out.mcu_temp_c, in.mcu_temp_c);
}

/* ------------------------------------------------------------------ */
/* 0x6B9 GPS_Status                                                    */
/* ------------------------------------------------------------------ */

static void test_gps_status_roundtrip(void)
{
    can_gps_status_t in = { .uptime_s = 54321,
                             .fault_bits = 0x00A5,
                             .gps_retry_count = 3,
                             .imu_retry_count = 1,
                             .cpu_load_pct = 47 };
    uint8_t buf[8];
    can_gps_status_t out;

    CHECK(can_pack_gps_status(&in, buf) == STATUS_OK, "pack failed");
    CHECK(can_unpack_gps_status(buf, &out) == STATUS_OK, "unpack failed");

    CHECK(out.uptime_s == in.uptime_s, "uptime_s");
    CHECK(out.fault_bits == in.fault_bits, "fault_bits");
    CHECK(out.gps_retry_count == in.gps_retry_count, "gps_retry_count");
    CHECK(out.imu_retry_count == in.imu_retry_count, "imu_retry_count");
    CHECK(out.cpu_load_pct == in.cpu_load_pct, "cpu_load_pct");
}

/* ------------------------------------------------------------------ */
/* 0x6BA GPS_Mag                                                       */
/* ------------------------------------------------------------------ */

static void test_gps_mag_roundtrip(void)
{
    can_gps_mag_t in = { .mx_ut = 22.3f,
                          .my_ut = -8.1f,
                          .mz_ut = -41.7f,
                          .cal_status = 2 };
    uint8_t buf[8];
    can_gps_mag_t out;

    CHECK(can_pack_gps_mag(&in, buf) == STATUS_OK, "pack failed");
    CHECK(can_unpack_gps_mag(buf, &out) == STATUS_OK, "unpack failed");

    CHECK(fabs_f(out.mx_ut - in.mx_ut) <= 0.15f, "mx %f vs %f", out.mx_ut,
          in.mx_ut);
    CHECK(fabs_f(out.my_ut - in.my_ut) <= 0.15f, "my %f vs %f", out.my_ut,
          in.my_ut);
    CHECK(fabs_f(out.mz_ut - in.mz_ut) <= 0.15f, "mz %f vs %f", out.mz_ut,
          in.mz_ut);
    CHECK(out.cal_status == in.cal_status, "cal_status");
}

/* ------------------------------------------------------------------ */
/* 0x6BF GPS_Command                                                   */
/* ------------------------------------------------------------------ */

static void test_gps_command_roundtrip(void)
{
    can_gps_command_t in = { .cmd = CAN_CMD_NMEA_CFG, .arg0 = 10, .arg1 = 1 };
    uint8_t buf[8];
    can_gps_command_t out;

    CHECK(can_pack_gps_command(&in, buf) == STATUS_OK, "pack failed");
    CHECK(can_unpack_gps_command(buf, &out) == STATUS_OK, "unpack failed");

    CHECK(out.cmd == in.cmd, "cmd");
    CHECK(out.arg0 == in.arg0, "arg0");
    CHECK(out.arg1 == in.arg1, "arg1");
}

/* ------------------------------------------------------------------ */
/* NULL-argument guards (spot check)                                  */
/* ------------------------------------------------------------------ */

static void test_null_args(void)
{
    uint8_t buf[8] = { 0 };
    can_gps_position_t p;

    CHECK(can_pack_gps_position(NULL, buf) == STATUS_INVALID_ARG,
          "pack NULL in");
    CHECK(can_pack_gps_position(&p, NULL) == STATUS_INVALID_ARG,
          "pack NULL out");
    CHECK(can_unpack_gps_position(NULL, &p) == STATUS_INVALID_ARG,
          "unpack NULL in");
    CHECK(can_unpack_gps_position(buf, NULL) == STATUS_INVALID_ARG,
          "unpack NULL out");
}

int main(void)
{
    test_gps_position_roundtrip();
    test_gps_position_bytes();
    test_gps_velocity_roundtrip();
    test_gps_velocity_bytes();
    test_gps_attitude_roundtrip();
    test_lap_status_roundtrip();
    test_lap_event_roundtrip();
    test_gps_quality_roundtrip();
    test_gps_imu_accel_roundtrip();
    test_gps_imu_gyro_roundtrip();
    test_gps_temp_roundtrip();
    test_gps_status_roundtrip();
    test_gps_mag_roundtrip();
    test_gps_command_roundtrip();
    test_null_args();

    if (g_failures == 0) {
        printf("PASS: all test_can_pack cases\n");
        return 0;
    }

    printf("FAIL: %d check(s) failed\n", g_failures);
    return 1;
}
