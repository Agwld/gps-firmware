/**
 * @file    imu_task.c
 * @brief   IMU sample loop: read -> AHRS -> KF predict/correct -> lap
 *          timing -> publish. Owns gates.c/laptimer.c/kf6.c/ahrs.c and
 *          the geodesy ENU origin - no other task touches them.
 *
 * IMU SPI reads (lsm6dso32.c) are DMA-backed: this task blocks on a
 * completion semaphore while each burst clocks out, yielding the CPU to
 * other tasks instead of spinning in a blocking HAL call. The loop is
 * still driven by the DRDY notification below; the DMA just removes the
 * transfer time from this task's busy budget.
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

/* Magnetometer calibration runs continuously in the background: every
 * raw sample is fed in as the car drives, the quality flag climbs as the
 * heading circle fills, and it's cross-checked against GPS course. Seeded
 * from flash at boot. imu_task owns it - it's the only task with raw mag
 * samples each cycle. s_last_mag_save rate-limits flash writes. */
static mag_cal_cont_t s_mag_cont;
static TickType_t s_last_mag_save;

/* Gates persisted as absolute lat/lon (flash_store), loaded at boot but
 * only resolvable to ENU once the first fix sets the frame origin - so
 * they're held here and placed in gates.c at origin time, and the origin
 * is never known before then. */
static flash_store_gate_t s_stored_gates[LAP_MAX_GATES];
static bool s_frame_origin_set;

/* deg -> i32 1e-7 deg (u-blox convention), round half away from zero. */
static int32_t
deg_to_1e7(double deg)
{
    double scaled = deg * 1e7;
    return (int32_t) (scaled >= 0.0 ? scaled + 0.5 : scaled - 0.5);
}

/* Calibration is continuous, so the old START/STOP pass is repurposed:
 * START forces a from-scratch rebuild (throw the current cal away and
 * re-sweep - use it if the magnetic environment changed, e.g. new
 * hardware bolted near the sensor); STOP is a no-op, kept so old senders
 * don't get NAKed. */
static void
apply_mag_cal_commands(void)
{
    app_cmd_t cmd;
    while (xQueueReceive(g_mag_cal_cmd_queue, &cmd, 0) == pdTRUE) {
        if (cmd.cmd == CAN_CMD_MAG_CAL_START) {
            mag_cal_result_t identity;
            mag_cal_identity(&identity);
            mag_cal_cont_init(&s_mag_cont, &identity);
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

/* Persist a live-set gate as absolute lat/lon (reproducible next boot,
 * unlike the per-boot ENU) and publish it for the ~2 Hz dash broadcast.
 * Best-effort on flash: if the page is full the gate still lives in RAM
 * and on the bus; a CONFIG_SAVE compaction (sys_task, when stationary)
 * reclaims space. */
static void
set_and_persist_gate(uint8_t index, float e, float n, float heading_rad)
{
    gates_set(index, e, n, heading_rad);

    /* Setting a new start/finish wipes the old sector split (gates.c does
     * this in RAM); mirror it on flash and on the broadcast so a reboot,
     * or the dash, doesn't resurrect sectors from the previous layout. */
    if (index == 0U) {
        (void) flash_store_save_gates_cleared_all();
        for (uint8_t i = 1U; i < LAP_MAX_GATES; i++) {
            canbc_state_set_gate(i, 0.0f, 0.0f, 0.0f, 0U);
        }
    }

    double lat_deg, lon_deg;
    float h;
    geodesy_from_enu(e, n, 0.0f, &lat_deg, &lon_deg, &h);
    (void) flash_store_save_gate(index, deg_to_1e7(lat_deg),
                                  deg_to_1e7(lon_deg), heading_rad);

    canbc_state_set_gate(index, e, n, heading_rad, 1U);
}

static void
apply_gate_commands(quat_t q)
{
    app_cmd_t cmd;
    float e, n, ve, vn;

    while (xQueueReceive(g_gate_cmd_queue, &cmd, 0) == pdTRUE) {
        if (cmd.cmd == CAN_CMD_GATE_SET) {
            /* A gate is placed at the current fused position, which only
             * means anything once the ENU frame origin exists. */
            if (!s_frame_origin_set) {
                continue;
            }
            kf6_get_state(&e, &n, NULL, &ve, &vn, NULL);
            /* Orient the gate perpendicular to the direction of travel.
             * Course over ground (the velocity vector) is the truest
             * crossing direction and needs no magnetometer, but it's pure
             * noise at a standstill - so at or below walking pace fall
             * back to the AHRS yaw (magnetometer-referenced heading, valid
             * stationary), letting a gate be planted with the car parked
             * and nose pointed down the track. */
            float heading_rad;
            if (sqrtf(ve * ve + vn * vn) >= LAP_GATE_COURSE_MIN_SPEED_MPS) {
                heading_rad = atan2f(vn, ve); /* travel direction */
            } else {
                vec3_t fwd = quat_rotate(q, (vec3_t) {1.0f, 0.0f, 0.0f});
                heading_rad = atan2f(fwd.y, fwd.x); /* vehicle heading */
            }
            set_and_persist_gate(cmd.arg0, e, n, heading_rad);
        } else if (cmd.cmd == CAN_CMD_GATE_CLEAR) {
            if (cmd.arg0 == 0xFFU) {
                gates_clear_all();
                (void) flash_store_save_gates_cleared_all();
                for (uint8_t i = 0U; i < LAP_MAX_GATES; i++) {
                    canbc_state_set_gate(i, 0.0f, 0.0f, 0.0f, 0U);
                }
            } else {
                gates_clear(cmd.arg0);
                (void) flash_store_save_gate_cleared(cmd.arg0);
                canbc_state_set_gate(cmd.arg0, 0.0f, 0.0f, 0.0f, 0U);
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

    double lat_deg = (double) pvt.lat_1e7 * 1e-7;
    double lon_deg = (double) pvt.lon_1e7 * 1e-7;
    float height_m = (float) pvt.hmsl_mm * 0.001f;

    if (!s_frame_origin_set) {
        if (pvt.hacc_mm > GEODESY_ORIGIN_MIN_HACC_MM) {
            return; /* wait for a better fix before anchoring the frame */
        }
        geodesy_set_origin(lat_deg, lon_deg, height_m);
        s_frame_origin_set = true;
        app_set_events(SYS_EVT_ORIGIN_SET);

        /* The frame now exists, so the absolute-lat/lon gates restored from
         * flash can finally be resolved to ENU and placed. Publish the
         * origin and every gate slot for the dash broadcast (invalid slots
         * too, so a dash that just connected learns which are empty). */
        canbc_state_set_origin(pvt.lat_1e7, pvt.lon_1e7);
        for (uint8_t i = 0U; i < LAP_MAX_GATES; i++) {
            if (s_stored_gates[i].valid) {
                float e_g, n_g, u_g;
                geodesy_to_enu((double) s_stored_gates[i].lat_1e7 * 1e-7,
                               (double) s_stored_gates[i].lon_1e7 * 1e-7,
                               height_m, &e_g, &n_g, &u_g);
                gates_set(i, e_g, n_g, s_stored_gates[i].heading_rad);
                canbc_state_set_gate(i, e_g, n_g,
                                     s_stored_gates[i].heading_rad, 1U);
            } else {
                canbc_state_set_gate(i, 0.0f, 0.0f, 0.0f, 0U);
            }
        }
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
    /* Loads the persisted gate table (absolute lat/lon) into
     * s_stored_gates and the mag calibration; the gates are placed into
     * gates.c later, once the first fix sets the ENU origin. Falls back to
     * identity calibration / no gates if nothing was saved. */
    mag_cal_result_t loaded_cal;
    flash_store_init(s_stored_gates, LAP_MAX_GATES, &loaded_cal);
    /* A persisted cal seeds the background calibrator at GOOD so heading
     * works from boot; the continuous sweep refines it and re-validates
     * against course from there. */
    mag_cal_cont_init(&s_mag_cont, &loaded_cal);
    s_last_mag_save = 0U;

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

                mag_cal_cont_feed(&s_mag_cont, mv.x, mv.y, mv.z);
                mag_cal_apply(mag_cal_cont_result(&s_mag_cont), mv.x, mv.y,
                              mv.z, &mx, &my, &mz);

                canbc_state_set_mag(
                    mx, my, mz, (uint8_t) mag_cal_cont_quality(&s_mag_cont));
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
        apply_gate_commands(q);

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

        /* Cross-check the mag-derived heading against course-over-ground,
         * but only when moving fast enough for course to be meaningful
         * (same floor as gate placement). This is what promotes the cal
         * to VALIDATED. */
        if (fused_speed_mps >= LAP_GATE_COURSE_MIN_SPEED_MPS) {
            mag_cal_cont_observe_heading(
                &s_mag_cont, yaw_deg * ((float) M_PI / 180.0f),
                fused_course_deg * ((float) M_PI / 180.0f));
        }

        /* Persist an improved cal, but rate-limited: the background sweep
         * only marks dirty on a material change, and once converged it
         * stops changing - so in steady state this never writes. The
         * interval floor caps flash wear during initial convergence. */
        bool save_due =
            (s_last_mag_save == 0U) ||
            (xTaskGetTickCount() - s_last_mag_save) >=
                pdMS_TO_TICKS(MAG_CAL_SAVE_MIN_INTERVAL_MS);
        if (save_due && mag_cal_cont_take_dirty(&s_mag_cont)) {
            flash_store_save_mag_cal(mag_cal_cont_result(&s_mag_cont));
            s_last_mag_save = xTaskGetTickCount();
        }

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
