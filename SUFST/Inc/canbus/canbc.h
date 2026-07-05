/**
 * @file    canbc.h
 * @brief   Mutex-guarded broadcast state: every task publishes its latest
 *          values here; canbc_task.c reads a consistent snapshot and
 *          packs/sends it on its own schedule. Decouples producers
 *          (imu_task, gps_task, sys_task) from the CAN bus entirely -
 *          none of them touch FDCAN directly.
 *
 * Pattern follows vcu's canbc.c: copy-in setters (one per logical group
 * of fields, so a partial update from one producer can't be seen half
 * from a stale snapshot) and a single copy-out snapshot getter, both
 * mutex-protected but never holding the lock across anything but the
 * memcpy/field assignment itself.
 */

#ifndef CANBC_H
#define CANBC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "board/board_config.h" /* LAP_MAX_GATES */

/* One gate's broadcastable state (ENU, relative to the current frame
 * origin) for the ~2 Hz GPS_Gate round-robin to the in-car dash. */
typedef struct {
    float east_m;
    float north_m;
    float heading_rad;
    uint8_t valid;
} canbc_gate_t;

typedef struct {
    /* fused position (imu_task, from kf6 + geodesy origin) */
    double lat_deg;
    double lon_deg;

    /* fused velocity/course/altitude (imu_task) */
    float speed_mps;
    float course_deg;
    float alt_m;
    uint8_t fix_type;
    uint8_t num_sv;

    /* AHRS attitude (imu_task) */
    float yaw_deg;
    float pitch_deg;
    float roll_deg;
    uint8_t fusion_status;

    /* raw calibrated IMU (imu_task) */
    float ax_mg, ay_mg, az_mg;
    float gx_dps, gy_dps, gz_dps;

    /* magnetometer (imu_task) */
    float mx_ut, my_ut, mz_ut;
    uint8_t mag_cal_status;

    /* GPS accuracy/RTK status (gps_task) */
    float hacc_mm;
    float sacc_mm_s;
    float pdop;
    uint8_t quality_flags;

    /* lap timing (imu_task, from laptimer.c) */
    uint16_t lap;
    uint32_t running_time_ms;
    uint8_t sector;
    uint8_t lap_flags;

    /* temperatures (sys_task) */
    float mcp9800_temp_c;
    float imu_temp_c;
    float mcu_temp_c;

    /* system status (sys_task) */
    uint16_t uptime_s;
    uint16_t fault_bits;
    uint8_t gps_retry_count;
    uint8_t imu_retry_count;
    uint8_t cpu_load_pct;

    /* ENU frame origin + lap gates (imu_task) - broadcast slowly so the
     * in-car dash can draw the start/finish and sector lines. Origin is
     * absolute lat/lon (i32 1e-7); origin_valid clears until the first
     * fix anchors the frame. */
    int32_t origin_lat_1e7;
    int32_t origin_lon_1e7;
    uint8_t origin_valid;
    canbc_gate_t gates[LAP_MAX_GATES];
} canbc_state_t;

/** @brief Zero the shared state and create its mutex. Call once from
 *         app_init() before any task starts. */
void canbc_state_init(void);

void canbc_state_set_position(double lat_deg, double lon_deg);
void canbc_state_set_velocity(float speed_mps, float course_deg,
                               float alt_m, uint8_t fix_type,
                               uint8_t num_sv);
void canbc_state_set_attitude(float yaw_deg, float pitch_deg,
                               float roll_deg, uint8_t fusion_status);
void canbc_state_set_imu_accel(float ax_mg, float ay_mg, float az_mg);
void canbc_state_set_imu_gyro(float gx_dps, float gy_dps, float gz_dps);
void canbc_state_set_mag(float mx_ut, float my_ut, float mz_ut,
                          uint8_t cal_status);
void canbc_state_set_quality(float hacc_mm, float sacc_mm_s, float pdop,
                              uint8_t quality_flags);
void canbc_state_set_lap(uint16_t lap, uint32_t running_time_ms,
                          uint8_t sector, uint8_t lap_flags);
void canbc_state_set_temp(float mcp9800_temp_c, float imu_temp_c,
                           float mcu_temp_c);
void canbc_state_set_status(uint16_t uptime_s, uint16_t fault_bits,
                             uint8_t gps_retry_count,
                             uint8_t imu_retry_count, uint8_t cpu_load_pct);

/** @brief Publish the ENU frame origin (absolute lat/lon, i32 1e-7 deg)
 *         so the dash can anchor the broadcast ENU gates. */
void canbc_state_set_origin(int32_t lat_1e7, int32_t lon_1e7);

/** @brief Publish one gate slot's ENU position/heading (valid=0 marks an
 *         empty slot, so a cleared gate is broadcast as removed). */
void canbc_state_set_gate(uint8_t index, float east_m, float north_m,
                           float heading_rad, uint8_t valid);

/** @brief Copy out a consistent snapshot of every field for the
 *         broadcast task to pack and send without holding the lock
 *         across the CAN transmit calls. */
void canbc_state_get_snapshot(canbc_state_t *out);

#ifdef __cplusplus
}
#endif

#endif /* CANBC_H */
