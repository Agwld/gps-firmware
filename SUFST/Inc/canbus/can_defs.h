/**
 * @file    can_defs.h
 * @brief   CAN-S bus matrix for the GPS node: message IDs, decoded
 *          (engineering-unit) payload structs, and pack/unpack prototypes.
 *
 * Wire format is always little-endian ("Intel") with 11-bit standard IDs,
 * matching tools/GPS.dbc exactly - the two are kept consistent by
 * tests/test_can_pack.c. Every message with a `counter` field carries it
 * as the last byte, incrementing per-send, so the receiver can detect
 * dropped frames.
 *
 * Euler + raw IMU (not quaternions) is a deliberate choice: quaternions
 * fill the whole 8-byte frame with no room for status, and every
 * consumer converts to heading/pitch/roll anyway.
 */

#ifndef CAN_DEFS_H
#define CAN_DEFS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "sys/status.h"

/* ------------------------------------------------------------------ */
/* Message IDs (11-bit standard), base 0x6B0                          */
/* ------------------------------------------------------------------ */

#define CAN_ID_GPS_POSITION  0x6B0U /* 20 Hz */
#define CAN_ID_GPS_VELOCITY  0x6B1U /* 20 Hz */
#define CAN_ID_GPS_ATTITUDE  0x6B2U /* 50 Hz */
#define CAN_ID_LAP_STATUS    0x6B3U /* 10 Hz */
#define CAN_ID_LAP_EVENT     0x6B4U /* event */
#define CAN_ID_GPS_QUALITY   0x6B5U /* 5 Hz */
#define CAN_ID_GPS_IMU_ACCEL 0x6B6U /* 100 Hz */
#define CAN_ID_GPS_IMU_GYRO  0x6B7U /* 100 Hz */
#define CAN_ID_GPS_TEMP      0x6B8U /* 1 Hz */
#define CAN_ID_GPS_STATUS    0x6B9U /* 1 Hz */
#define CAN_ID_GPS_MAG       0x6BAU /* 10 Hz */
#define CAN_ID_GPS_FRAME_ORIGIN 0x6BBU /* 1 Hz - ENU origin lat/lon */
#define CAN_ID_GPS_GATE      0x6BCU /* ~5 Hz aggregate, round-robin per slot */
#define CAN_ID_GPS_TIME      0x6BDU /* 1 Hz - GPS iTOW + UTC clock */
#define CAN_ID_GPS_COMMAND   0x6BFU /* RX, event */

/* Frame lengths (bytes) - match tools/GPS.dbc `BO_` length fields */
#define CAN_DLC_GPS_POSITION  8U
#define CAN_DLC_GPS_VELOCITY  8U
#define CAN_DLC_GPS_ATTITUDE  8U
#define CAN_DLC_LAP_STATUS    8U
#define CAN_DLC_LAP_EVENT     8U
#define CAN_DLC_GPS_QUALITY   7U
#define CAN_DLC_GPS_IMU_ACCEL 7U
#define CAN_DLC_GPS_IMU_GYRO  7U
#define CAN_DLC_GPS_TEMP      6U
#define CAN_DLC_GPS_STATUS    7U
#define CAN_DLC_GPS_MAG       7U
#define CAN_DLC_GPS_FRAME_ORIGIN 8U
#define CAN_DLC_GPS_GATE      8U
#define CAN_DLC_GPS_TIME      8U
#define CAN_DLC_GPS_COMMAND   3U

/* ------------------------------------------------------------------ */
/* Shared field enums                                                  */
/* ------------------------------------------------------------------ */

/* Lap_Event.type - what `time_ms` (and, for TM2, `lap`) represents */
#define CAN_LAP_EVENT_LAP    0U /* lap complete: time_ms = lap time */
#define CAN_LAP_EVENT_SECTOR 1U /* sector crossed: time_ms = sector time */
#define CAN_LAP_EVENT_TM2    2U /* button/EXTINT time-mark: time_ms = iTOW */

/* GPS_Quality.flags bit layout */
#define CAN_GPS_QUALITY_FIX_OK          (1U << 0)
#define CAN_GPS_QUALITY_CARR_SOLN_SHIFT 1U
#define CAN_GPS_QUALITY_CARR_SOLN_MASK                                      \
    (0x3U << CAN_GPS_QUALITY_CARR_SOLN_SHIFT)
#define CAN_GPS_QUALITY_DIFF_SOLN       (1U << 3)
/* carrSoln: 0 = none, 1 = float RTK, 2 = fixed RTK */

/* GPS_Command.cmd sub-commands (RX only; node listens on CAN_ID_GPS_COMMAND).
 * byte0 = cmd, byte1 = arg0, byte2 = arg1; unused trailing bytes ignored. */
#define CAN_CMD_GATE_SET      0x01U /* arg0 = slot (0=start/finish,1-7=sector);
                                      * gate is placed at the node's current
                                      * fused position/heading */
#define CAN_CMD_GATE_CLEAR    0x02U /* arg0 = slot, 0xFFU = clear all */
#define CAN_CMD_MAG_CAL_START 0x10U
#define CAN_CMD_MAG_CAL_STOP  0x11U
#define CAN_CMD_CONFIG_SAVE   0x20U
#define CAN_CMD_NMEA_CFG      0x30U /* arg0 = rate Hz, arg1 = enable (0/1) */

/* ------------------------------------------------------------------ */
/* Decoded (engineering-unit) payload structs                         */
/* ------------------------------------------------------------------ */

/* 0x6B0 GPS_Position - fused position. double survives the round trip
 * through the i32 1e-7 deg encoding exactly enough (LSB << double ULP). */
typedef struct {
    double lat_deg; /* raw i32, LSB = 1e-7 deg */
    double lon_deg; /* raw i32, LSB = 1e-7 deg */
} can_gps_position_t;

/* 0x6B1 GPS_Velocity */
typedef struct {
    float speed_mps;  /* raw u16, LSB = 0.01 m/s, clamped [0, 655.35] */
    float course_deg; /* raw u16, LSB = 0.01 deg, clamped [0, 655.35] */
    float alt_m;      /* raw i16, LSB = 0.1 m */
    uint8_t fix_type;  /* 0 none, 2 2D, 3 3D, 4 GNSS+DR (ubx.h fix_type);
                         * packed into 4 bits, clamped [0, 15] */
    uint8_t num_sv;    /* packed into 4 bits, clamped [0, 15] */
    uint8_t counter;   /* increments per send */
} can_gps_velocity_t;

/* 0x6B2 GPS_Attitude - Mahony AHRS Euler output */
typedef struct {
    float yaw_deg;         /* raw u16, LSB = 0.01 deg, wrapped [0, 360) */
    float pitch_deg;       /* raw i16, LSB = 0.01 deg */
    float roll_deg;        /* raw i16, LSB = 0.01 deg */
    uint8_t fusion_status; /* implementation-defined AHRS health/mode byte */
    uint8_t counter;       /* increments per send */
} can_gps_attitude_t;

/* 0x6B3 Lap_Status */
typedef struct {
    uint16_t lap;              /* raw as-is */
    uint32_t running_time_ms;  /* raw as-is, since current lap start */
    uint8_t sector;            /* raw as-is */
    uint8_t flags;             /* implementation-defined status bits, e.g.
                                 * gate-armed / GPS-suspended / stationary */
} can_lap_status_t;

/* 0x6B4 Lap_Event */
typedef struct {
    uint8_t type;      /* CAN_LAP_EVENT_* */
    uint16_t lap;       /* raw as-is */
    uint32_t time_ms;   /* meaning depends on `type`, see CAN_LAP_EVENT_* */
    uint8_t counter;    /* increments per send */
} can_lap_event_t;

/* 0x6B5 GPS_Quality */
typedef struct {
    float hacc_mm;   /* raw u16, LSB = 1 mm, clamped [0, 65535] */
    float sacc_mm_s; /* raw u16, LSB = 0.1 mm/s, clamped [0, 6553.5] */
    float pdop;      /* raw u16, LSB = 0.01, clamped [0, 655.35] */
    uint8_t flags;   /* CAN_GPS_QUALITY_* bits */
} can_gps_quality_t;

/* 0x6B6 GPS_IMU_Accel - calibrated, body frame, +-16 g FS */
typedef struct {
    float ax_mg; /* raw i16, LSB = 1 mg */
    float ay_mg; /* raw i16, LSB = 1 mg */
    float az_mg; /* raw i16, LSB = 1 mg */
    uint8_t counter; /* increments per send */
} can_gps_imu_accel_t;

/* 0x6B7 GPS_IMU_Gyro - calibrated, body frame. LSB = 0.0625 dps (1/16),
 * range +-2047.94 dps, covering the LSM6DSO32's +-2000 dps FS with margin -
 * the plan's originally-specified 0.02 dps/LSB saturated at +-655.34 dps,
 * well inside the sensor's actual range, so it was widened here. */
typedef struct {
    float gx_dps; /* raw i16, LSB = 0.0625 dps */
    float gy_dps; /* raw i16, LSB = 0.0625 dps */
    float gz_dps; /* raw i16, LSB = 0.0625 dps */
    uint8_t counter; /* increments per send */
} can_gps_imu_gyro_t;

/* 0x6B8 GPS_Temp */
typedef struct {
    float mcp9800_temp_c; /* raw i16, LSB = 0.01 C, board ambient */
    float imu_temp_c;     /* raw i16, LSB = 0.01 C, LSM6DSO32 die temp */
    float mcu_temp_c;     /* raw i16, LSB = 0.01 C, STM32G431 internal */
} can_gps_temp_t;

/* 0x6B9 GPS_Status */
typedef struct {
    uint16_t uptime_s;        /* raw as-is, since boot */
    uint16_t fault_bits;      /* implementation-defined fault event-group */
    uint8_t gps_retry_count;  /* raw as-is */
    uint8_t imu_retry_count;  /* raw as-is */
    uint8_t cpu_load_pct;     /* raw as-is, [0, 100] */
} can_gps_status_t;

/* 0x6BA GPS_Mag - diagnostic only, not used by AHRS correction directly.
 * uT*10 is used (not raw sensor LSB) so the frame is self-describing on
 * the bus independent of IIS2MDC gain/config. */
typedef struct {
    float mx_ut;         /* raw i16, LSB = 0.1 uT */
    float my_ut;         /* raw i16, LSB = 0.1 uT */
    float mz_ut;         /* raw i16, LSB = 0.1 uT */
    uint8_t cal_status;  /* 0 uncalibrated, 1 calibrating, 2 calibrated */
} can_gps_mag_t;

/* 0x6BB GPS_Frame_Origin - the ENU origin the gates/track are relative
 * to, as absolute lat/lon. Broadcast so a dash can place the ENU gates
 * (and its own track breadcrumb) in a common frame; re-sent every boot
 * since the origin is re-anchored at each power-up's first fix. */
typedef struct {
    double lat_deg; /* raw i32, LSB = 1e-7 deg */
    double lon_deg; /* raw i32, LSB = 1e-7 deg */
} can_gps_frame_origin_t;

/* 0x6BC GPS_Gate - one lap gate, round-robined a slot at a time. Position
 * is ENU metres relative to GPS_Frame_Origin (i16 0.1 m, +-3276.7 m spans
 * any FS track). `index` is a plain data field (which slot this is), not a
 * DBC multiplexor - every frame has the same layout. */
typedef struct {
    uint8_t index;      /* 0 = start/finish, 1..7 = sector */
    uint8_t flags;      /* bit0 = valid (0 => slot empty/cleared) */
    float east_m;       /* raw i16, LSB = 0.1 m */
    float north_m;      /* raw i16, LSB = 0.1 m */
    float heading_deg;  /* raw u16, LSB = 0.1 deg, [0, 360) */
} can_gps_gate_t;

#define CAN_GPS_GATE_FLAG_VALID (1U << 0)

/* 0x6BD GPS_Time - GPS time of week + UTC wall clock, so other nodes can
 * timestamp their own data in the same time domain as Lap_Event (iTOW)
 * and drive a human-readable clock. The UTC *date* is deliberately not
 * carried (8 bytes won't fit it alongside iTOW at full ms precision, and
 * the MoTeC already receives it via NMEA RMC); note UTC h/m/s is NOT
 * simply iTOW mod 1 day - GPS time leads UTC by the current leap-second
 * count, which is exactly why both are broadcast. */
typedef struct {
    uint32_t itow_ms;  /* raw as-is, GPS time of week */
    uint8_t utc_hour;  /* raw as-is, [0, 23] */
    uint8_t utc_min;   /* raw as-is, [0, 59] */
    uint8_t utc_sec;   /* raw as-is, [0, 60] (60 during a leap second) */
    uint8_t flags;     /* CAN_GPS_TIME_FLAG_* validity bits */
} can_gps_time_t;

#define CAN_GPS_TIME_FLAG_UTC_VALID      (1U << 0) /* NAV-PVT validTime */
#define CAN_GPS_TIME_FLAG_FULLY_RESOLVED (1U << 1) /* no sub-s ambiguity */

/* 0x6BF GPS_Command (RX) */
typedef struct {
    uint8_t cmd;  /* CAN_CMD_* */
    uint8_t arg0;
    uint8_t arg1;
} can_gps_command_t;

/* ------------------------------------------------------------------ */
/* Pack / unpack                                                      */
/* ------------------------------------------------------------------ */

status_t can_pack_gps_position(const can_gps_position_t *in,
                                uint8_t out[8]);
status_t can_unpack_gps_position(const uint8_t in[8],
                                  can_gps_position_t *out);

status_t can_pack_gps_velocity(const can_gps_velocity_t *in,
                                uint8_t out[8]);
status_t can_unpack_gps_velocity(const uint8_t in[8],
                                  can_gps_velocity_t *out);

status_t can_pack_gps_attitude(const can_gps_attitude_t *in,
                                uint8_t out[8]);
status_t can_unpack_gps_attitude(const uint8_t in[8],
                                  can_gps_attitude_t *out);

status_t can_pack_lap_status(const can_lap_status_t *in, uint8_t out[8]);
status_t can_unpack_lap_status(const uint8_t in[8], can_lap_status_t *out);

status_t can_pack_lap_event(const can_lap_event_t *in, uint8_t out[8]);
status_t can_unpack_lap_event(const uint8_t in[8], can_lap_event_t *out);

status_t can_pack_gps_quality(const can_gps_quality_t *in, uint8_t out[8]);
status_t can_unpack_gps_quality(const uint8_t in[8],
                                 can_gps_quality_t *out);

status_t can_pack_gps_imu_accel(const can_gps_imu_accel_t *in,
                                 uint8_t out[8]);
status_t can_unpack_gps_imu_accel(const uint8_t in[8],
                                   can_gps_imu_accel_t *out);

status_t can_pack_gps_imu_gyro(const can_gps_imu_gyro_t *in,
                                uint8_t out[8]);
status_t can_unpack_gps_imu_gyro(const uint8_t in[8],
                                  can_gps_imu_gyro_t *out);

status_t can_pack_gps_temp(const can_gps_temp_t *in, uint8_t out[8]);
status_t can_unpack_gps_temp(const uint8_t in[8], can_gps_temp_t *out);

status_t can_pack_gps_status(const can_gps_status_t *in, uint8_t out[8]);
status_t can_unpack_gps_status(const uint8_t in[8], can_gps_status_t *out);

status_t can_pack_gps_mag(const can_gps_mag_t *in, uint8_t out[8]);
status_t can_unpack_gps_mag(const uint8_t in[8], can_gps_mag_t *out);

status_t can_pack_gps_command(const can_gps_command_t *in, uint8_t out[8]);
status_t can_unpack_gps_command(const uint8_t in[8],
                                 can_gps_command_t *out);

status_t can_pack_gps_frame_origin(const can_gps_frame_origin_t *in,
                                    uint8_t out[8]);
status_t can_unpack_gps_frame_origin(const uint8_t in[8],
                                      can_gps_frame_origin_t *out);

status_t can_pack_gps_gate(const can_gps_gate_t *in, uint8_t out[8]);
status_t can_unpack_gps_gate(const uint8_t in[8], can_gps_gate_t *out);

status_t can_pack_gps_time(const can_gps_time_t *in, uint8_t out[8]);
status_t can_unpack_gps_time(const uint8_t in[8], can_gps_time_t *out);

#ifdef __cplusplus
}
#endif

#endif /* CAN_DEFS_H */
