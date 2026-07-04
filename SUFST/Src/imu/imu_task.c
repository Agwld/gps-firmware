/**
 * @file    imu_task.c
 * @brief   IMU sample loop: read -> AHRS -> KF predict/correct -> lap
 *          timing -> publish. Owns gates.c/laptimer.c/kf6.c/ahrs.c and
 *          the geodesy ENU origin - no other task touches them.
 *
 * SPI reads are blocking HAL calls rather than the DMA pipeline the plan
 * describes - a reasonable first cut given a 104 Hz / ~14-byte transfer
 * budget, but a lower-CPU-overhead DMA+EXTI version is a natural
 * follow-up once this is bench-verified to actually work.
 */

#include "imu/imu_task.h"

#include <math.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "main.h"

#include "board/board_config.h"
#include "canbus/can_defs.h"
#include "canbus/canbc.h"
#include "fusion/ahrs.h"
#include "fusion/geodesy.h"
#include "fusion/kf6.h"
#include "fusion/timebase.h"
#include "gps/ubx.h"
#include "imu/lsm6dso32.h"
#include "imu/mag_cal.h"
#include "laptimer/gates.h"
#include "laptimer/laptimer.h"
#include "persist/flash_store.h"
#include "sys/app.h"

#define IMU_MAG_DECIMATION 1U /* sensor-hub updates the mag once per ODR cycle */
#define GEODESY_ORIGIN_MIN_HACC_MM 5000U /* don't anchor the origin on a poor fix */

static TaskHandle_t s_imu_task_handle;

/* Active magnetometer calibration (loaded from flash at boot, replaced
 * when a CAN-triggered calibration pass finishes) and, while a pass is
 * in progress, the accumulator collecting it. imu_task owns both since
 * it's the only task with raw mag samples each cycle. */
static mag_cal_result_t s_mag_cal;
static mag_cal_accumulator_t s_mag_cal_acc;
static bool s_mag_cal_in_progress;

static void
apply_mag_cal_commands(void)
{
    app_cmd_t cmd;
    while (xQueueReceive(g_mag_cal_cmd_queue, &cmd, 0) == pdTRUE) {
        if (cmd.cmd == CAN_CMD_MAG_CAL_START) {
            mag_cal_start(&s_mag_cal_acc);
            s_mag_cal_in_progress = true;
        } else if (cmd.cmd == CAN_CMD_MAG_CAL_STOP) {
            s_mag_cal_in_progress = false;
            mag_cal_result_t new_cal;
            if (mag_cal_finish(&s_mag_cal_acc, &new_cal) == STATUS_OK) {
                s_mag_cal = new_cal;
                flash_store_save_mag_cal(&s_mag_cal);
            }
            /* On failure (not enough rotation/samples), keep the
             * previous calibration rather than replacing it with
             * garbage. */
        }
    }
}

typedef struct {
    float x, y, z;
} vec3_t;

static const float k_mount[3][3] = IMU_MOUNT_MATRIX;

static vec3_t
apply_mount(float x, float y, float z)
{
    vec3_t v;
    v.x = k_mount[0][0] * x + k_mount[0][1] * y + k_mount[0][2] * z;
    v.y = k_mount[1][0] * x + k_mount[1][1] * y + k_mount[1][2] * z;
    v.z = k_mount[2][0] * x + k_mount[2][1] * y + k_mount[2][2] * z;
    return v;
}

/* Rotate v by quaternion q (body -> world, ahrs.h's convention);
 * duplicated in kf6.c and test_ahrs.c too - a generic rotation formula,
 * not shared module state, so each keeps its own copy. */
static vec3_t
quat_rotate(quat_t q, vec3_t v)
{
    float tx = 2.0f * (q.y * v.z - q.z * v.y);
    float ty = 2.0f * (q.z * v.x - q.x * v.z);
    float tz = 2.0f * (q.x * v.y - q.y * v.x);

    vec3_t r;
    r.x = v.x + q.w * tx + (q.y * tz - q.z * ty);
    r.y = v.y + q.w * ty + (q.z * tx - q.x * tz);
    r.z = v.z + q.w * tz + (q.x * ty - q.y * tx);
    return r;
}

static void
quat_to_euler_deg(quat_t q, float *yaw_deg, float *pitch_deg,
                   float *roll_deg)
{
    float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    *roll_deg = atan2f(sinr_cosp, cosr_cosp) * (180.0f / (float) M_PI);

    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    sinp = (sinp > 1.0f) ? 1.0f : ((sinp < -1.0f) ? -1.0f : sinp);
    *pitch_deg = asinf(sinp) * (180.0f / (float) M_PI);

    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    float yaw = atan2f(siny_cosp, cosy_cosp) * (180.0f / (float) M_PI);
    *yaw_deg = (yaw < 0.0f) ? (yaw + 360.0f) : yaw;
}

static void
apply_gate_commands(void)
{
    app_cmd_t cmd;
    float e, n, ve, vn;

    while (xQueueReceive(g_gate_cmd_queue, &cmd, 0) == pdTRUE) {
        if (cmd.cmd == CAN_CMD_GATE_SET) {
            kf6_get_state(&e, &n, NULL, &ve, &vn, NULL);
            float heading_rad = atan2f(vn, ve); /* travel direction */
            gates_set(cmd.arg0, e, n, heading_rad);
        } else if (cmd.cmd == CAN_CMD_GATE_CLEAR) {
            if (cmd.arg0 == 0xFFU) {
                gates_clear_all();
            } else {
                gates_clear(cmd.arg0);
            }
        }
    }
}

/* Updated at GPS rate (20 Hz) by handle_gps_fix(); read every loop
 * iteration (104 Hz) when publishing GPS_Velocity alongside the fused
 * speed/course, which do update every iteration. */
static uint8_t s_fix_type = 0U;
static uint8_t s_num_sv = 0U;

static void
handle_gps_fix(void)
{
    ubx_nav_pvt_t pvt;
    if (xQueueReceive(g_gps_pvt_queue, &pvt, 0) != pdTRUE) {
        return;
    }

    static bool s_origin_set = false;
    double lat_deg = (double) pvt.lat_1e7 * 1e-7;
    double lon_deg = (double) pvt.lon_1e7 * 1e-7;
    float height_m = (float) pvt.hmsl_mm * 0.001f;

    if (!s_origin_set) {
        if (pvt.hacc_mm > GEODESY_ORIGIN_MIN_HACC_MM) {
            return; /* wait for a better fix before anchoring the frame */
        }
        geodesy_set_origin(lat_deg, lon_deg, height_m);
        s_origin_set = true;
        app_set_events(SYS_EVT_ORIGIN_SET);
        return; /* this fix defines (0,0,0); nothing to correct against yet */
    }

    float e_m, n_m, u_m;
    geodesy_to_enu(lat_deg, lon_deg, height_m, &e_m, &n_m, &u_m);

    uint32_t fix_tick = timebase_itow_ms_to_tick(pvt.itow_ms);
    float sigma_pos_m = (float) pvt.hacc_mm * 0.001f;
    if (sigma_pos_m < 0.5f) {
        sigma_pos_m = 0.5f; /* floor: hAcc alone understates real error */
    }
    kf6_correct_pos(fix_tick, e_m, n_m, u_m, sigma_pos_m);

    float ve_mps = (float) pvt.vel_e_mms * 0.001f;
    float vn_mps = (float) pvt.vel_n_mms * 0.001f;
    float vu_mps = -(float) pvt.vel_d_mms * 0.001f;
    float sigma_vel_mps = (float) pvt.sacc_mms * 0.001f;
    if (sigma_vel_mps < 0.1f) {
        sigma_vel_mps = 0.1f;
    }
    kf6_correct_vel(fix_tick, ve_mps, vn_mps, vu_mps, sigma_vel_mps);

    s_fix_type = pvt.fix_type;
    s_num_sv = pvt.num_sv;
}

static void
handle_wheelspeed(quat_t q)
{
    float speed_mps;
    if (xQueueReceive(g_wheelspeed_queue, &speed_mps, 0) != pdTRUE) {
        return;
    }

    vec3_t forward_world = quat_rotate(q, (vec3_t) {1.0f, 0.0f, 0.0f});
    float heading_rad = atan2f(forward_world.y, forward_world.x);

    kf6_correct_speed(timebase_get_tick(), speed_mps, heading_rad, 0.3f);
}

void
imu_task_main(void *argument)
{
    (void) argument;

    s_imu_task_handle = xTaskGetCurrentTaskHandle();

    if (lsm6dso32_init() != STATUS_OK) {
        app_set_events(SYS_EVT_IMU_FAULT);
    } else {
        app_set_events(SYS_EVT_IMU_READY);
        if (lsm6dso32_mag_init() == STATUS_OK) {
            app_set_events(SYS_EVT_MAG_READY);
        } else {
            app_set_events(SYS_EVT_MAG_FAULT);
        }
    }

    ahrs_init();
    kf6_init();
    gates_init();
    laptimer_init();
    s_mag_cal_in_progress = false;
    flash_store_init(&s_mag_cal); /* restores gates too; falls back to
                                    * identity calibration if none saved */

    uint32_t prev_tick = timebase_get_tick();
    float prev_e = 0.0f, prev_n = 0.0f;
    uint32_t prev_itow_ms = 0U;
    uint32_t mag_decim = 0U;

    for (;;) {
        /* DRDY notification with a watchdog timeout: keep the loop
         * alive (at nominal rate) even if the interrupt is ever missed,
         * rather than stalling fusion indefinitely. */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20U));

        uint32_t tick = timebase_get_tick();
        float dt = (float) (int32_t) (tick - prev_tick) * 1e-6f;
        if (dt <= 0.0f || dt > 0.1f) {
            dt = 1.0f / (float) IMU_ODR_HZ;
        }
        prev_tick = tick;

        lsm6dso32_sample_t s;
        if (lsm6dso32_read(&s) != STATUS_OK) {
            continue;
        }

        vec3_t a = apply_mount(s.ax_g, s.ay_g, s.az_g);
        vec3_t g = apply_mount(s.gx_dps, s.gy_dps, s.gz_dps);

        apply_mag_cal_commands();

        float mx = 0.0f, my = 0.0f, mz = 0.0f;
        if ((mag_decim++ % IMU_MAG_DECIMATION) == 0U) {
            lsm6dso32_mag_sample_t m;
            if (lsm6dso32_read_mag(&m) == STATUS_OK && m.valid) {
                vec3_t mv = apply_mount(m.mx_ut, m.my_ut, m.mz_ut);

                if (s_mag_cal_in_progress) {
                    mag_cal_feed(&s_mag_cal_acc, mv.x, mv.y, mv.z);
                }
                mag_cal_apply(&s_mag_cal, mv.x, mv.y, mv.z, &mx, &my, &mz);

                canbc_state_set_mag(mx, my, mz,
                                     s_mag_cal_in_progress ? 1U : 2U);
            }
        }

        quat_t q;
        ahrs_update(g.x * ((float) M_PI / 180.0f),
                    g.y * ((float) M_PI / 180.0f),
                    g.z * ((float) M_PI / 180.0f), a.x, a.y, a.z, mx, my,
                    mz, dt, &q);

        kf6_predict(tick, a.x * IMU_GRAVITY_MPS2, a.y * IMU_GRAVITY_MPS2,
                    a.z * IMU_GRAVITY_MPS2, q, dt);

        handle_gps_fix();
        handle_wheelspeed(q);
        apply_gate_commands();

        float e_m, n_m, u_m, ve_mps, vn_mps;
        kf6_get_state(&e_m, &n_m, &u_m, &ve_mps, &vn_mps, NULL);

        double fused_lat_deg, fused_lon_deg;
        float fused_height_m;
        geodesy_from_enu(e_m, n_m, u_m, &fused_lat_deg, &fused_lon_deg,
                          &fused_height_m);
        canbc_state_set_position(fused_lat_deg, fused_lon_deg);

        float fused_speed_mps = sqrtf(ve_mps * ve_mps + vn_mps * vn_mps);
        float fused_course_deg =
            atan2f(ve_mps, vn_mps) * (180.0f / (float) M_PI);
        if (fused_course_deg < 0.0f) {
            fused_course_deg += 360.0f;
        }
        canbc_state_set_velocity(fused_speed_mps, fused_course_deg,
                                  fused_height_m, s_fix_type, s_num_sv);

        uint32_t itow_ms = timebase_tick_to_itow_ms(tick);
        uint16_t lap_before = laptimer_get_lap_count();
        uint8_t sector_before = laptimer_get_current_sector();

        laptimer_update(e_m, n_m, prev_e, prev_n, itow_ms, prev_itow_ms);

        if (laptimer_get_lap_count() != lap_before) {
            can_lap_event_t evt = {CAN_LAP_EVENT_LAP, lap_before + 1U,
                                    laptimer_get_last_lap_ms(), 0U};
            xQueueSend(g_lap_event_queue, &evt, 0);
        }
        if (laptimer_get_current_sector() != sector_before) {
            can_lap_event_t evt = {CAN_LAP_EVENT_SECTOR,
                                    laptimer_get_lap_count(),
                                    laptimer_get_last_sector_ms(), 0U};
            xQueueSend(g_lap_event_queue, &evt, 0);
        }

        uint8_t lap_flags = laptimer_is_running() ? 0x01U : 0x00U;
        canbc_state_set_lap(laptimer_get_lap_count(),
                             laptimer_get_current_elapsed_ms(itow_ms),
                             laptimer_get_current_sector(), lap_flags);

        float yaw_deg, pitch_deg, roll_deg;
        quat_to_euler_deg(q, &yaw_deg, &pitch_deg, &roll_deg);
        canbc_state_set_attitude(yaw_deg, pitch_deg, roll_deg, 0U);

        canbc_state_set_imu_accel(a.x * 1000.0f, a.y * 1000.0f,
                                   a.z * 1000.0f);
        canbc_state_set_imu_gyro(g.x, g.y, g.z);

        prev_e = e_m;
        prev_n = n_m;
        prev_itow_ms = itow_ms;
    }
}

void
HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != IMU_INT_PIN) {
        return;
    }
    BaseType_t hp_woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_imu_task_handle, &hp_woken);
    portYIELD_FROM_ISR(hp_woken);
}
