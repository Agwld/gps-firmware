/**
 * @file    can_pack.c
 * @brief   Pack/unpack for the GPS node's CAN-S messages (see can_defs.h).
 *
 * Pure integer/bit arithmetic - no printf, no libc float formatting.
 * Float/double are only ever used for the multiply-by-scale-factor step;
 * everything else is explicit byte shifts so the wire layout is identical
 * regardless of host/target endianness (this file is host-testable).
 */

#include "canbus/can_defs.h"

#include <stddef.h>

/* ------------------------------------------------------------------ */
/* Little-endian byte read/write helpers                              */
/* ------------------------------------------------------------------ */

static void wr_u8(uint8_t *p, uint8_t v)
{
    p[0] = v;
}

static void wr_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
}

static void wr_i16(uint8_t *p, int16_t v)
{
    wr_u16(p, (uint16_t)v);
}

static void wr_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
    p[2] = (uint8_t)((v >> 16) & 0xFFU);
    p[3] = (uint8_t)((v >> 24) & 0xFFU);
}

static void wr_i32(uint8_t *p, int32_t v)
{
    wr_u32(p, (uint32_t)v);
}

static uint8_t rd_u8(const uint8_t *p)
{
    return p[0];
}

static uint16_t rd_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int16_t rd_i16(const uint8_t *p)
{
    return (int16_t)rd_u16(p);
}

static uint32_t rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int32_t rd_i32(const uint8_t *p)
{
    return (int32_t)rd_u32(p);
}

/* ------------------------------------------------------------------ */
/* Rounding + saturation helpers                                      */
/* ------------------------------------------------------------------ */

/* Round-half-away-from-zero; avoids a libm lroundf() dependency. */
static int64_t round_f(float x)
{
    return (int64_t)(x >= 0.0f ? x + 0.5f : x - 0.5f);
}

static int64_t round_d(double x)
{
    return (int64_t)(x >= 0.0 ? x + 0.5 : x - 0.5);
}

static uint16_t sat_u16(int64_t v)
{
    if (v < 0) {
        return 0U;
    }
    if (v > 0xFFFF) {
        return 0xFFFFU;
    }
    return (uint16_t)v;
}

static int16_t sat_i16(int64_t v)
{
    if (v < INT16_MIN) {
        return INT16_MIN;
    }
    if (v > INT16_MAX) {
        return INT16_MAX;
    }
    return (int16_t)v;
}

static int32_t sat_i32(int64_t v)
{
    if (v < INT32_MIN) {
        return INT32_MIN;
    }
    if (v > INT32_MAX) {
        return INT32_MAX;
    }
    return (int32_t)v;
}

static uint8_t sat_u4(uint8_t v)
{
    return (v > 0x0FU) ? 0x0FU : v;
}

/* ------------------------------------------------------------------ */
/* 0x6B0 GPS_Position                                                  */
/* ------------------------------------------------------------------ */

status_t can_pack_gps_position(const can_gps_position_t *in,
                                uint8_t out[8])
{
    if (in == NULL || out == NULL) {
        return STATUS_INVALID_ARG;
    }

    int32_t raw_lat = sat_i32(round_d(in->lat_deg * 1e7));
    int32_t raw_lon = sat_i32(round_d(in->lon_deg * 1e7));

    wr_i32(&out[0], raw_lat);
    wr_i32(&out[4], raw_lon);

    return STATUS_OK;
}

status_t can_unpack_gps_position(const uint8_t in[8],
                                  can_gps_position_t *out)
{
    if (in == NULL || out == NULL) {
        return STATUS_INVALID_ARG;
    }

    out->lat_deg = (double)rd_i32(&in[0]) * 1e-7;
    out->lon_deg = (double)rd_i32(&in[4]) * 1e-7;

    return STATUS_OK;
}

/* ------------------------------------------------------------------ */
/* 0x6B1 GPS_Velocity                                                  */
/* ------------------------------------------------------------------ */

status_t can_pack_gps_velocity(const can_gps_velocity_t *in,
                                uint8_t out[8])
{
    if (in == NULL || out == NULL) {
        return STATUS_INVALID_ARG;
    }

    uint16_t raw_speed = sat_u16(round_f(in->speed_mps * 100.0f));
    uint16_t raw_course = sat_u16(round_f(in->course_deg * 100.0f));
    int16_t raw_alt = sat_i16(round_f(in->alt_m * 10.0f));
    uint8_t fix = sat_u4(in->fix_type);
    uint8_t sv = sat_u4(in->num_sv);

    wr_u16(&out[0], raw_speed);
    wr_u16(&out[2], raw_course);
    wr_i16(&out[4], raw_alt);
    wr_u8(&out[6], (uint8_t)(fix | (uint8_t)(sv << 4)));
    wr_u8(&out[7], in->counter);

    return STATUS_OK;
}

status_t can_unpack_gps_velocity(const uint8_t in[8],
                                  can_gps_velocity_t *out)
{
    if (in == NULL || out == NULL) {
        return STATUS_INVALID_ARG;
    }

    out->speed_mps = (float)rd_u16(&in[0]) * 0.01f;
    out->course_deg = (float)rd_u16(&in[2]) * 0.01f;
    out->alt_m = (float)rd_i16(&in[4]) * 0.1f;
    out->fix_type = (uint8_t)(rd_u8(&in[6]) & 0x0FU);
    out->num_sv = (uint8_t)((rd_u8(&in[6]) >> 4) & 0x0FU);
    out->counter = rd_u8(&in[7]);

    return STATUS_OK;
}

/* ------------------------------------------------------------------ */
/* 0x6B2 GPS_Attitude                                                  */
/* ------------------------------------------------------------------ */

status_t can_pack_gps_attitude(const can_gps_attitude_t *in,
                                uint8_t out[8])
{
    if (in == NULL || out == NULL) {
        return STATUS_INVALID_ARG;
    }

    uint16_t raw_yaw = sat_u16(round_f(in->yaw_deg * 100.0f));
    int16_t raw_pitch = sat_i16(round_f(in->pitch_deg * 100.0f));
    int16_t raw_roll = sat_i16(round_f(in->roll_deg * 100.0f));

    wr_u16(&out[0], raw_yaw);
    wr_i16(&out[2], raw_pitch);
    wr_i16(&out[4], raw_roll);
    wr_u8(&out[6], in->fusion_status);
    wr_u8(&out[7], in->counter);

    return STATUS_OK;
}

status_t can_unpack_gps_attitude(const uint8_t in[8],
                                  can_gps_attitude_t *out)
{
    if (in == NULL || out == NULL) {
        return STATUS_INVALID_ARG;
    }

    out->yaw_deg = (float)rd_u16(&in[0]) * 0.01f;
    out->pitch_deg = (float)rd_i16(&in[2]) * 0.01f;
    out->roll_deg = (float)rd_i16(&in[4]) * 0.01f;
    out->fusion_status = rd_u8(&in[6]);
    out->counter = rd_u8(&in[7]);

    return STATUS_OK;
}

/* ------------------------------------------------------------------ */
/* 0x6B3 Lap_Status                                                    */
/* ------------------------------------------------------------------ */

status_t can_pack_lap_status(const can_lap_status_t *in, uint8_t out[8])
{
    if (in == NULL || out == NULL) {
        return STATUS_INVALID_ARG;
    }

    wr_u16(&out[0], in->lap);
    wr_u32(&out[2], in->running_time_ms);
    wr_u8(&out[6], in->sector);
    wr_u8(&out[7], in->flags);

    return STATUS_OK;
}

status_t can_unpack_lap_status(const uint8_t in[8], can_lap_status_t *out)
{
    if (in == NULL || out == NULL) {
        return STATUS_INVALID_ARG;
    }

    out->lap = rd_u16(&in[0]);
    out->running_time_ms = rd_u32(&in[2]);
    out->sector = rd_u8(&in[6]);
    out->flags = rd_u8(&in[7]);

    return STATUS_OK;
}

/* ------------------------------------------------------------------ */
/* 0x6B4 Lap_Event                                                     */
/* ------------------------------------------------------------------ */

status_t can_pack_lap_event(const can_lap_event_t *in, uint8_t out[8])
{
    if (in == NULL || out == NULL) {
        return STATUS_INVALID_ARG;
    }

    wr_u8(&out[0], in->type);
    wr_u16(&out[1], in->lap);
    wr_u32(&out[3], in->time_ms);
    wr_u8(&out[7], in->counter);

    return STATUS_OK;
}

status_t can_unpack_lap_event(const uint8_t in[8], can_lap_event_t *out)
{
    if (in == NULL || out == NULL) {
        return STATUS_INVALID_ARG;
    }

    out->type = rd_u8(&in[0]);
    out->lap = rd_u16(&in[1]);
    out->time_ms = rd_u32(&in[3]);
    out->counter = rd_u8(&in[7]);

    return STATUS_OK;
}

/* ------------------------------------------------------------------ */
/* 0x6B5 GPS_Quality                                                   */
/* ------------------------------------------------------------------ */

status_t can_pack_gps_quality(const can_gps_quality_t *in, uint8_t out[8])
{
    if (in == NULL || out == NULL) {
        return STATUS_INVALID_ARG;
    }

    uint16_t raw_hacc = sat_u16(round_f(in->hacc_mm));
    uint16_t raw_sacc = sat_u16(round_f(in->sacc_mm_s * 10.0f));
    uint16_t raw_pdop = sat_u16(round_f(in->pdop * 100.0f));

    wr_u16(&out[0], raw_hacc);
    wr_u16(&out[2], raw_sacc);
    wr_u16(&out[4], raw_pdop);
    wr_u8(&out[6], in->flags);
    wr_u8(&out[7], 0U); /* reserved, DLC 7 */

    return STATUS_OK;
}

status_t can_unpack_gps_quality(const uint8_t in[8],
                                 can_gps_quality_t *out)
{
    if (in == NULL || out == NULL) {
        return STATUS_INVALID_ARG;
    }

    out->hacc_mm = (float)rd_u16(&in[0]);
    out->sacc_mm_s = (float)rd_u16(&in[2]) * 0.1f;
    out->pdop = (float)rd_u16(&in[4]) * 0.01f;
    out->flags = rd_u8(&in[6]);

    return STATUS_OK;
}

/* ------------------------------------------------------------------ */
/* 0x6B6 GPS_IMU_Accel                                                 */
/* ------------------------------------------------------------------ */

status_t can_pack_gps_imu_accel(const can_gps_imu_accel_t *in,
                                 uint8_t out[8])
{
    if (in == NULL || out == NULL) {
        return STATUS_INVALID_ARG;
    }

    wr_i16(&out[0], sat_i16(round_f(in->ax_mg)));
    wr_i16(&out[2], sat_i16(round_f(in->ay_mg)));
    wr_i16(&out[4], sat_i16(round_f(in->az_mg)));
    wr_u8(&out[6], in->counter);

    return STATUS_OK;
}

status_t can_unpack_gps_imu_accel(const uint8_t in[8],
                                   can_gps_imu_accel_t *out)
{
    if (in == NULL || out == NULL) {
        return STATUS_INVALID_ARG;
    }

    out->ax_mg = (float)rd_i16(&in[0]);
    out->ay_mg = (float)rd_i16(&in[2]);
    out->az_mg = (float)rd_i16(&in[4]);
    out->counter = rd_u8(&in[6]);

    return STATUS_OK;
}

/* ------------------------------------------------------------------ */
/* 0x6B7 GPS_IMU_Gyro                                                  */
/* ------------------------------------------------------------------ */

status_t can_pack_gps_imu_gyro(const can_gps_imu_gyro_t *in,
                                uint8_t out[8])
{
    if (in == NULL || out == NULL) {
        return STATUS_INVALID_ARG;
    }

    wr_i16(&out[0], sat_i16(round_f(in->gx_dps * 16.0f))); /* 1/0.0625 */
    wr_i16(&out[2], sat_i16(round_f(in->gy_dps * 16.0f)));
    wr_i16(&out[4], sat_i16(round_f(in->gz_dps * 16.0f)));
    wr_u8(&out[6], in->counter);

    return STATUS_OK;
}

status_t can_unpack_gps_imu_gyro(const uint8_t in[8],
                                  can_gps_imu_gyro_t *out)
{
    if (in == NULL || out == NULL) {
        return STATUS_INVALID_ARG;
    }

    out->gx_dps = (float)rd_i16(&in[0]) * 0.0625f;
    out->gy_dps = (float)rd_i16(&in[2]) * 0.0625f;
    out->gz_dps = (float)rd_i16(&in[4]) * 0.0625f;
    out->counter = rd_u8(&in[6]);

    return STATUS_OK;
}

/* ------------------------------------------------------------------ */
/* 0x6B8 GPS_Temp                                                      */
/* ------------------------------------------------------------------ */

status_t can_pack_gps_temp(const can_gps_temp_t *in, uint8_t out[8])
{
    if (in == NULL || out == NULL) {
        return STATUS_INVALID_ARG;
    }

    wr_i16(&out[0], sat_i16(round_f(in->mcp9800_temp_c * 100.0f)));
    wr_i16(&out[2], sat_i16(round_f(in->imu_temp_c * 100.0f)));
    wr_i16(&out[4], sat_i16(round_f(in->mcu_temp_c * 100.0f)));

    return STATUS_OK;
}

status_t can_unpack_gps_temp(const uint8_t in[8], can_gps_temp_t *out)
{
    if (in == NULL || out == NULL) {
        return STATUS_INVALID_ARG;
    }

    out->mcp9800_temp_c = (float)rd_i16(&in[0]) * 0.01f;
    out->imu_temp_c = (float)rd_i16(&in[2]) * 0.01f;
    out->mcu_temp_c = (float)rd_i16(&in[4]) * 0.01f;

    return STATUS_OK;
}

/* ------------------------------------------------------------------ */
/* 0x6B9 GPS_Status                                                    */
/* ------------------------------------------------------------------ */

status_t can_pack_gps_status(const can_gps_status_t *in, uint8_t out[8])
{
    if (in == NULL || out == NULL) {
        return STATUS_INVALID_ARG;
    }

    wr_u16(&out[0], in->uptime_s);
    wr_u16(&out[2], in->fault_bits);
    wr_u8(&out[4], in->gps_retry_count);
    wr_u8(&out[5], in->imu_retry_count);
    wr_u8(&out[6], in->cpu_load_pct);

    return STATUS_OK;
}

status_t can_unpack_gps_status(const uint8_t in[8], can_gps_status_t *out)
{
    if (in == NULL || out == NULL) {
        return STATUS_INVALID_ARG;
    }

    out->uptime_s = rd_u16(&in[0]);
    out->fault_bits = rd_u16(&in[2]);
    out->gps_retry_count = rd_u8(&in[4]);
    out->imu_retry_count = rd_u8(&in[5]);
    out->cpu_load_pct = rd_u8(&in[6]);

    return STATUS_OK;
}

/* ------------------------------------------------------------------ */
/* 0x6BA GPS_Mag                                                       */
/* ------------------------------------------------------------------ */

status_t can_pack_gps_mag(const can_gps_mag_t *in, uint8_t out[8])
{
    if (in == NULL || out == NULL) {
        return STATUS_INVALID_ARG;
    }

    wr_i16(&out[0], sat_i16(round_f(in->mx_ut * 10.0f)));
    wr_i16(&out[2], sat_i16(round_f(in->my_ut * 10.0f)));
    wr_i16(&out[4], sat_i16(round_f(in->mz_ut * 10.0f)));
    wr_u8(&out[6], in->cal_status);

    return STATUS_OK;
}

status_t can_unpack_gps_mag(const uint8_t in[8], can_gps_mag_t *out)
{
    if (in == NULL || out == NULL) {
        return STATUS_INVALID_ARG;
    }

    out->mx_ut = (float)rd_i16(&in[0]) * 0.1f;
    out->my_ut = (float)rd_i16(&in[2]) * 0.1f;
    out->mz_ut = (float)rd_i16(&in[4]) * 0.1f;
    out->cal_status = rd_u8(&in[6]);

    return STATUS_OK;
}

/* ------------------------------------------------------------------ */
/* 0x6BF GPS_Command (RX)                                              */
/* ------------------------------------------------------------------ */

status_t can_pack_gps_command(const can_gps_command_t *in, uint8_t out[8])
{
    if (in == NULL || out == NULL) {
        return STATUS_INVALID_ARG;
    }

    wr_u8(&out[0], in->cmd);
    wr_u8(&out[1], in->arg0);
    wr_u8(&out[2], in->arg1);

    return STATUS_OK;
}

status_t can_unpack_gps_command(const uint8_t in[8],
                                 can_gps_command_t *out)
{
    if (in == NULL || out == NULL) {
        return STATUS_INVALID_ARG;
    }

    out->cmd = rd_u8(&in[0]);
    out->arg0 = rd_u8(&in[1]);
    out->arg1 = rd_u8(&in[2]);

    return STATUS_OK;
}

/* ------------------------------------------------------------------ */
/* 0x6BB GPS_Frame_Origin                                              */
/* ------------------------------------------------------------------ */

status_t can_pack_gps_frame_origin(const can_gps_frame_origin_t *in,
                                    uint8_t out[8])
{
    if (in == NULL || out == NULL) {
        return STATUS_INVALID_ARG;
    }

    wr_i32(&out[0], sat_i32(round_d(in->lat_deg * 1e7)));
    wr_i32(&out[4], sat_i32(round_d(in->lon_deg * 1e7)));

    return STATUS_OK;
}

status_t can_unpack_gps_frame_origin(const uint8_t in[8],
                                      can_gps_frame_origin_t *out)
{
    if (in == NULL || out == NULL) {
        return STATUS_INVALID_ARG;
    }

    out->lat_deg = (double)rd_i32(&in[0]) * 1e-7;
    out->lon_deg = (double)rd_i32(&in[4]) * 1e-7;

    return STATUS_OK;
}

/* ------------------------------------------------------------------ */
/* 0x6BC GPS_Gate                                                      */
/* ------------------------------------------------------------------ */

status_t can_pack_gps_gate(const can_gps_gate_t *in, uint8_t out[8])
{
    if (in == NULL || out == NULL) {
        return STATUS_INVALID_ARG;
    }

    wr_u8(&out[0], in->index);
    wr_u8(&out[1], in->flags);
    wr_i16(&out[2], sat_i16(round_f(in->east_m * 10.0f)));
    wr_i16(&out[4], sat_i16(round_f(in->north_m * 10.0f)));
    wr_u16(&out[6], sat_u16(round_f(in->heading_deg * 10.0f)));

    return STATUS_OK;
}

status_t can_unpack_gps_gate(const uint8_t in[8], can_gps_gate_t *out)
{
    if (in == NULL || out == NULL) {
        return STATUS_INVALID_ARG;
    }

    out->index = rd_u8(&in[0]);
    out->flags = rd_u8(&in[1]);
    out->east_m = (float)rd_i16(&in[2]) * 0.1f;
    out->north_m = (float)rd_i16(&in[4]) * 0.1f;
    out->heading_deg = (float)rd_u16(&in[6]) * 0.1f;

    return STATUS_OK;
}
