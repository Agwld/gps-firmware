/**
 * @file    canbc.c
 * @brief   Mutex-guarded broadcast state (see canbc.h).
 */

#include "canbus/canbc.h"

#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"

static canbc_state_t s_state;
static StaticSemaphore_t s_mutex_buf;
static SemaphoreHandle_t s_mutex;

static void
lock(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
}

static void
unlock(void)
{
    xSemaphoreGive(s_mutex);
}

void
canbc_state_init(void)
{
    memset(&s_state, 0, sizeof(s_state));
    s_mutex = xSemaphoreCreateMutexStatic(&s_mutex_buf);
}

void
canbc_state_set_position(double lat_deg, double lon_deg)
{
    lock();
    s_state.lat_deg = lat_deg;
    s_state.lon_deg = lon_deg;
    unlock();
}

void
canbc_state_set_velocity(float speed_mps, float course_deg, float alt_m,
                          uint8_t fix_type, uint8_t num_sv)
{
    lock();
    s_state.speed_mps = speed_mps;
    s_state.course_deg = course_deg;
    s_state.alt_m = alt_m;
    s_state.fix_type = fix_type;
    s_state.num_sv = num_sv;
    unlock();
}

void
canbc_state_set_attitude(float yaw_deg, float pitch_deg, float roll_deg,
                          uint8_t fusion_status)
{
    lock();
    s_state.yaw_deg = yaw_deg;
    s_state.pitch_deg = pitch_deg;
    s_state.roll_deg = roll_deg;
    s_state.fusion_status = fusion_status;
    unlock();
}

void
canbc_state_set_imu_accel(float ax_mg, float ay_mg, float az_mg)
{
    lock();
    s_state.ax_mg = ax_mg;
    s_state.ay_mg = ay_mg;
    s_state.az_mg = az_mg;
    unlock();
}

void
canbc_state_set_imu_gyro(float gx_dps, float gy_dps, float gz_dps)
{
    lock();
    s_state.gx_dps = gx_dps;
    s_state.gy_dps = gy_dps;
    s_state.gz_dps = gz_dps;
    unlock();
}

void
canbc_state_set_mag(float mx_ut, float my_ut, float mz_ut,
                     uint8_t cal_status)
{
    lock();
    s_state.mx_ut = mx_ut;
    s_state.my_ut = my_ut;
    s_state.mz_ut = mz_ut;
    s_state.mag_cal_status = cal_status;
    unlock();
}

void
canbc_state_set_quality(float hacc_mm, float sacc_mm_s, float pdop,
                         uint8_t quality_flags)
{
    lock();
    s_state.hacc_mm = hacc_mm;
    s_state.sacc_mm_s = sacc_mm_s;
    s_state.pdop = pdop;
    s_state.quality_flags = quality_flags;
    unlock();
}

void
canbc_state_set_time(uint32_t itow_ms, uint8_t utc_hour, uint8_t utc_min,
                      uint8_t utc_sec, uint8_t time_flags)
{
    lock();
    s_state.itow_ms = itow_ms;
    s_state.utc_hour = utc_hour;
    s_state.utc_min = utc_min;
    s_state.utc_sec = utc_sec;
    s_state.time_flags = time_flags;
    unlock();
}

void
canbc_state_set_lap(uint16_t lap, uint32_t running_time_ms, uint8_t sector,
                     uint8_t lap_flags)
{
    lock();
    s_state.lap = lap;
    s_state.running_time_ms = running_time_ms;
    s_state.sector = sector;
    s_state.lap_flags = lap_flags;
    unlock();
}

void
canbc_state_set_temp(float mcp9800_temp_c, float imu_temp_c,
                      float mcu_temp_c)
{
    lock();
    s_state.mcp9800_temp_c = mcp9800_temp_c;
    s_state.imu_temp_c = imu_temp_c;
    s_state.mcu_temp_c = mcu_temp_c;
    unlock();
}

void
canbc_state_set_status(uint16_t uptime_s, uint16_t fault_bits,
                        uint8_t gps_retry_count, uint8_t imu_retry_count,
                        uint8_t cpu_load_pct)
{
    lock();
    s_state.uptime_s = uptime_s;
    s_state.fault_bits = fault_bits;
    s_state.gps_retry_count = gps_retry_count;
    s_state.imu_retry_count = imu_retry_count;
    s_state.cpu_load_pct = cpu_load_pct;
    unlock();
}

void
canbc_state_set_origin(int32_t lat_1e7, int32_t lon_1e7)
{
    lock();
    s_state.origin_lat_1e7 = lat_1e7;
    s_state.origin_lon_1e7 = lon_1e7;
    s_state.origin_valid = 1U;
    unlock();
}

void
canbc_state_set_gate(uint8_t index, float east_m, float north_m,
                     float heading_rad, uint8_t valid)
{
    if (index >= LAP_MAX_GATES) {
        return;
    }
    lock();
    s_state.gates[index].east_m = east_m;
    s_state.gates[index].north_m = north_m;
    s_state.gates[index].heading_rad = heading_rad;
    s_state.gates[index].valid = valid;
    unlock();
}

void
canbc_state_get_snapshot(canbc_state_t *out)
{
    lock();
    *out = s_state;
    unlock();
}
