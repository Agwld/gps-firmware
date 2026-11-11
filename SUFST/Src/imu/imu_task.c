/**
 * @file    imu_task.c
 * @brief   IMU sample loop: read -> AHRS -> KF predict/correct -> lap
 *          timing -> publish. Owns gates.c/laptimer.c/kf6.c/ahrs.c, the
 *          geodesy ENU origin and flash_store.c - no other task touches
 *          them.
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
#include "fusion/airdata.h"
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

#define GEODESY_ORIGIN_MIN_HACC_MM 5000U /* don't anchor the origin on a poor fix */

/* The LSM6DSO32 needs up to ~10 ms after power-on before it answers, and
 * this task can get there first; probe a few times before calling it a
 * fault. */
#define IMU_INIT_ATTEMPTS        5U
#define IMU_INIT_RETRY_MS        20U

/* Consecutive failed sample reads (each one a DMA timeout or SPI error)
 * before the IMU is flagged faulted. */
#define IMU_FAULT_CONSECUTIVE_READS 10U

/* How often (in loop iterations, ~1 s) the sensor's configuration is read
 * back, its die temperature published, and - while faulted - a re-init
 * attempted. */
#define IMU_HEALTH_PERIOD_LOOPS  IMU_ODR_HZ

/* A magnetometer that has produced no new sample for this long is treated
 * as failed (its nominal rate is 100 Hz). */
#define MAG_STALE_MS             1000U

/* Flash compaction erases pages, stalling the CPU for tens of ms: only
 * done while the fused speed says the car is stopped. */
#define COMPACT_MAX_SPEED_MPS    0.5f

#define RAD_TO_DEG (180.0f / (float) M_PI)
#define DEG_TO_RAD ((float) M_PI / 180.0f)

static TaskHandle_t s_imu_task_handle;

/* Failed IMU reads since boot, for GPS_Status (read by sys_task; a single
 * byte, so the cross-task read needs no lock). */
static volatile uint8_t s_read_error_count;

/* Magnetometer calibration runs continuously in the background: every
 * raw sample is fed in as the car drives, the quality flag climbs as the
 * heading circle fills, and it's cross-checked against GPS course. Seeded
 * from flash at boot. imu_task owns it - it's the only task with raw mag
 * samples each cycle. s_last_mag_save rate-limits flash writes. */
static mag_cal_cont_t s_mag_cont;
static TickType_t s_last_mag_save;

/* The gate table in ABSOLUTE lat/lon - the form that is persisted
 * (flash_store) and that survives the ENU origin moving between boots.
 * Loaded from flash at boot and kept current on every set/clear, so it
 * is always what flash should hold. gates.c holds the same gates in ENU
 * for crossing detection, but only from the moment the first fix anchors
 * the frame (s_frame_origin_set): until then a restored gate exists here
 * alone. */
static flash_store_gate_t s_gate_table[LAP_MAX_GATES];
static bool s_frame_origin_set;

/* Flash housekeeping, serviced when the car is stationary. A compaction
 * is requested by CAN_CMD_CONFIG_SAVE or by a save finding the page full;
 * in the latter case that record was dropped, so the whole current state
 * is re-saved afterwards. */
static bool s_compact_pending;
static bool s_resave_pending;
static bool s_flash_error; /* the last flash write failed (not just full) */

/* Latest GPS status, updated at fix rate by handle_gps_pvt() and published
 * every loop alongside the fused speed/course. The tick stamps drive the
 * staleness checks: a fix that stops arriving must read as lost. */
static uint8_t s_fix_type;
static uint8_t s_num_sv;
static bool s_pvt_seen;
static bool s_fix_seen;
static TickType_t s_last_pvt_tick;
static TickType_t s_last_fix_tick;

/* Set whenever the fused position may have jumped (frame just anchored,
 * time base just locked): the next sample must not be paired with the
 * previous one for gate-crossing detection. */
static bool s_lap_sample_valid;

uint8_t
imu_task_get_error_count(void)
{
    return s_read_error_count;
}

/* deg -> i32 1e-7 deg (u-blox convention), round half away from zero. */
static int32_t
deg_to_1e7(double deg)
{
    double scaled = deg * 1e7;
    return (int32_t) (scaled >= 0.0 ? scaled + 0.5 : scaled - 0.5);
}

static double
deg_from_1e7(int32_t val_1e7)
{
    return (double) val_1e7 * 1e-7;
}

/* Record the outcome of a flash append. A full page means the record was
 * NOT stored: schedule a compaction and a full re-save so flash catches
 * up with RAM once the car stops, instead of silently diverging. Any
 * other failure is left alone (retrying a failing program in a loop would
 * only add wear). */
static void
note_save(status_t st)
{
    if (st == STATUS_FULL) {
        s_compact_pending = true;
        s_resave_pending = true;
    } else {
        s_flash_error = (st != STATUS_OK);
    }
}

/* Publish the flash store's state in the event group, which GPS_Status
 * carries, so a dash can show whether gate edits have reached flash. */
static void
publish_flash_state(void)
{
    uint32_t set = 0U;
    if (s_resave_pending) {
        set |= SYS_EVT_FLASH_UNSAVED;
    }
    if (s_flash_error) {
        set |= SYS_EVT_FLASH_ERROR;
    }
    if (s_compact_pending) {
        set |= SYS_EVT_FLASH_TIDY;
    }
    const uint32_t all =
        SYS_EVT_FLASH_UNSAVED | SYS_EVT_FLASH_ERROR | SYS_EVT_FLASH_TIDY;
    app_clear_events(all & ~set);
    if (set != 0U) {
        app_set_events(set);
    }
}

/* Calibration is continuous, so the old START/STOP pass is repurposed:
 * START forces a from-scratch rebuild (throw the current cal away and
 * re-sweep - use it if the magnetic environment changed, e.g. new
 * hardware bolted near the sensor); STOP is a no-op, kept so old senders
 * aren't broken. */
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
static const float k_mag_mount[3][3] = MAG_MOUNT_MATRIX;

static vec3_t
apply_matrix(const float m[3][3], float x, float y, float z)
{
    vec3_t v;
    v.x = m[0][0] * x + m[0][1] * y + m[0][2] * z;
    v.y = m[1][0] * x + m[1][1] * y + m[1][2] * z;
    v.z = m[2][0] * x + m[2][1] * y + m[2][2] * z;
    return v;
}

/* Accel/gyro use the LSM6DSO32 orientation; the magnetometer is a
 * physically separate part with its own axis orientation, so it gets its
 * own matrix rather than borrowing the accel/gyro one. */
static vec3_t
apply_mount(float x, float y, float z)
{
    return apply_matrix(k_mount, x, y, z);
}

static vec3_t
apply_mag_mount(float x, float y, float z)
{
    return apply_matrix(k_mag_mount, x, y, z);
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
    *roll_deg = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    sinp = (sinp > 1.0f) ? 1.0f : ((sinp < -1.0f) ? -1.0f : sinp);
    *pitch_deg = asinf(sinp) * RAD_TO_DEG;

    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    float yaw = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG;
    *yaw_deg = (yaw < 0.0f) ? (yaw + 360.0f) : yaw;
}

/* ------------------------------------------------------------------ */
/* Gate table                                                          */
/* ------------------------------------------------------------------ */

/* Remove one gate from RAM (both representations) and from the dash
 * broadcast. Persisting the removal is the caller's job, since a single
 * clear and a clear-all are recorded differently. */
static void
drop_gate(uint8_t index)
{
    (void) gates_clear(index);
    s_gate_table[index].valid = false;
    canbc_state_set_gate(index, 0.0f, 0.0f, 0.0f, 0U);
}

/* The slot a lap-button short press fills: the first free sector, or the
 * last one once all are taken. Derived from the gate table rather than a
 * press counter, so it stays right across reboots (restored gates) and
 * gates set or cleared over CAN. */
static uint8_t
next_sector_slot(void)
{
    for (uint8_t i = 1U; i < LAP_MAX_GATES; i++) {
        if (!s_gate_table[i].valid) {
            return i;
        }
    }
    return (uint8_t) (LAP_MAX_GATES - 1U);
}

/* Place a gate at an ENU position, persist it as absolute lat/lon
 * (reproducible next boot, unlike the per-boot ENU) and publish it for
 * the dash broadcast. Flash is best-effort: if the page is full the gate
 * still lives in RAM and on the bus, and note_save() arranges for it to
 * reach flash once the car stops. */
static void
place_gate(uint8_t index, float e, float n, float heading_rad)
{
    if (index == 0U) {
        /* A new start/finish defines a new lap, so the old sector split
         * no longer belongs to it (gates.c drops it in RAM as well):
         * mirror that on flash and on the broadcast so neither a reboot
         * nor the dash resurrects sectors from the previous layout. And
         * the lap in progress was timed from a line that has just moved -
         * timing restarts from the new one. */
        for (uint8_t i = 1U; i < LAP_MAX_GATES; i++) {
            drop_gate(i);
        }
        note_save(flash_store_save_gates_cleared_all());
        laptimer_init();
    }

    (void) gates_set(index, e, n, heading_rad);

    double lat_deg, lon_deg;
    float h;
    geodesy_from_enu(e, n, 0.0f, &lat_deg, &lon_deg, &h);

    s_gate_table[index].lat_1e7 = deg_to_1e7(lat_deg);
    s_gate_table[index].lon_1e7 = deg_to_1e7(lon_deg);
    s_gate_table[index].heading_rad = heading_rad;
    s_gate_table[index].valid = true;

    note_save(flash_store_save_gate(index, s_gate_table[index].lat_1e7,
                                    s_gate_table[index].lon_1e7,
                                    heading_rad));

    canbc_state_set_gate(index, e, n, heading_rad, 1U);
}

/* Gate set/clear (from CAN or the lap button) and config-save. */
static void
apply_gate_commands(quat_t q)
{
    app_cmd_t cmd;

    while (xQueueReceive(g_gate_cmd_queue, &cmd, 0) == pdTRUE) {
        if (cmd.cmd == CAN_CMD_GATE_SET) {
            /* A gate is placed at the current fused position, which only
             * means anything once the ENU frame origin exists. */
            if (!s_frame_origin_set) {
                continue;
            }

            uint8_t slot = cmd.arg0;
            if (slot == CAN_GATE_SLOT_NEXT_SECTOR) {
                slot = next_sector_slot();
            }
            if (slot >= LAP_MAX_GATES) {
                continue; /* not a slot: ignore rather than persist junk */
            }

            float e, n, ve, vn;
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
            place_gate(slot, e, n, heading_rad);
        } else if (cmd.cmd == CAN_CMD_GATE_CLEAR) {
            /* Clears work before the frame is anchored too: they act on
             * the absolute-coordinate table, so a gate restored from
             * flash can be removed before the first fix would place it. */
            if (cmd.arg0 == CAN_GATE_SLOT_ALL) {
                for (uint8_t i = 0U; i < LAP_MAX_GATES; i++) {
                    drop_gate(i);
                }
                note_save(flash_store_save_gates_cleared_all());
                laptimer_init();
            } else if (cmd.arg0 < LAP_MAX_GATES) {
                drop_gate(cmd.arg0);
                note_save(flash_store_save_gate_cleared(cmd.arg0));
                if (cmd.arg0 == 0U) {
                    laptimer_init(); /* no start/finish: nothing to time */
                }
            }
        } else if (cmd.cmd == CAN_CMD_CONFIG_SAVE) {
            s_compact_pending = true;
        }
    }

    /* GPS_Gate_Set: a gate at a position chosen on a map, rather than
     * wherever the car is. Same placement and persistence path as above,
     * so it behaves exactly like a gate set from the car - including slot
     * 0 clearing the sectors. */
    can_gps_gate_t at;
    while (xQueueReceive(g_gate_set_queue, &at, 0) == pdTRUE) {
        if (!s_frame_origin_set) {
            continue; /* the position is relative to an origin not yet set */
        }
        uint8_t slot = at.index;
        if (slot == CAN_GATE_SLOT_NEXT_SECTOR) {
            slot = next_sector_slot();
        }
        if (slot >= LAP_MAX_GATES) {
            continue;
        }
        /* Broadcast as [0, 360); stored as an atan2 result in (-pi, pi]. */
        float heading_deg = at.heading_deg;
        if (heading_deg > 180.0f) {
            heading_deg -= 360.0f;
        }
        place_gate(slot, at.east_m, at.north_m, heading_deg * DEG_TO_RAD);
    }
}

/* The ENU frame has just been anchored: resolve the absolute-lat/lon
 * gates to ENU, place them, and publish the origin and every slot for the
 * dash broadcast (invalid slots too, so a dash that just connected learns
 * which are empty). */
static void
place_restored_gates(float origin_height_m)
{
    for (uint8_t i = 0U; i < LAP_MAX_GATES; i++) {
        if (s_gate_table[i].valid) {
            float e_g, n_g, u_g;
            geodesy_to_enu(deg_from_1e7(s_gate_table[i].lat_1e7),
                           deg_from_1e7(s_gate_table[i].lon_1e7),
                           origin_height_m, &e_g, &n_g, &u_g);
            /* Ascending order, so placing slot 0 first can't wipe the
             * sectors placed after it (gates_set(0) clears slots 1+). */
            (void) gates_set(i, e_g, n_g, s_gate_table[i].heading_rad);
            canbc_state_set_gate(i, e_g, n_g, s_gate_table[i].heading_rad,
                                 1U);
        } else {
            canbc_state_set_gate(i, 0.0f, 0.0f, 0.0f, 0U);
        }
    }
}

/* ------------------------------------------------------------------ */
/* Flash housekeeping                                                  */
/* ------------------------------------------------------------------ */

/* Bring flash back in line with RAM after records were dropped on a full
 * page. Runs straight after a compaction, so there is room: a clear-all
 * marker voids whatever flash still believes, then every live gate and
 * the current mag-cal are written fresh. */
static void
resave_all(void)
{
    (void) flash_store_save_gates_cleared_all();
    for (uint8_t i = 0U; i < LAP_MAX_GATES; i++) {
        if (s_gate_table[i].valid) {
            (void) flash_store_save_gate(i, s_gate_table[i].lat_1e7,
                                         s_gate_table[i].lon_1e7,
                                         s_gate_table[i].heading_rad);
        }
    }
    (void) flash_store_save_mag_cal(mag_cal_cont_result(&s_mag_cont));
}

static void
service_flash(float fused_speed_mps)
{
    if (!s_compact_pending || fused_speed_mps >= COMPACT_MAX_SPEED_MPS) {
        return;
    }
    s_compact_pending = false;

    if (flash_store_erase_and_compact() == STATUS_OK && s_resave_pending) {
        resave_all();
    }
    s_resave_pending = false;
}

/* ------------------------------------------------------------------ */
/* GPS / wheel-speed corrections                                       */
/* ------------------------------------------------------------------ */

static void
handle_gps_pvt(void)
{
    ubx_nav_pvt_t pvt;
    if (xQueueReceive(g_gps_pvt_queue, &pvt, 0) != pdTRUE) {
        return;
    }

    TickType_t now = xTaskGetTickCount();
    bool fix_ok = (pvt.flags & UBX_PVT_FLAG_GNSS_FIX_OK) != 0U;

    s_pvt_seen = true;
    s_last_pvt_tick = now;
    /* The receiver can report a fix type while flagging the solution as
     * outside its accuracy masks; that is not a fix anyone should use. */
    s_fix_type = fix_ok ? pvt.fix_type : 0U;
    s_num_sv = pvt.num_sv;

    if (!fix_ok) {
        return;
    }

    double lat_deg = deg_from_1e7(pvt.lat_1e7);
    double lon_deg = deg_from_1e7(pvt.lon_1e7);
    float height_m = (float) pvt.hmsl_mm * 0.001f;

    if (!s_frame_origin_set) {
        if (pvt.hacc_mm > GEODESY_ORIGIN_MIN_HACC_MM) {
            return; /* wait for a better fix before anchoring the frame */
        }
        geodesy_set_origin(lat_deg, lon_deg, height_m);
        s_frame_origin_set = true;
        app_set_events(SYS_EVT_ORIGIN_SET);
        canbc_state_set_origin(pvt.lat_1e7, pvt.lon_1e7);

        /* Until now the filter has been integrating the IMU with nothing
         * to hold it, and may sit anywhere. Restart it at this fix - the
         * frame's (0,0,0) - so the gates about to be placed aren't swept
         * across by the state snapping in from wherever it had drifted. */
        kf6_init();
        s_lap_sample_valid = false;

        place_restored_gates(height_m);

        s_fix_seen = true;
        s_last_fix_tick = now;
        return; /* this fix defines (0,0,0); nothing to correct against yet */
    }

    float e_m, n_m, u_m;
    geodesy_to_enu(lat_deg, lon_deg, height_m, &e_m, &n_m, &u_m);

    /* Before the first PPS edge disciplines the timebase, iTOW can't be
     * mapped into the tick domain: timebase_itow_ms_to_tick() returns 0,
     * which kf6's rewind would anchor to the oldest (~150 ms stale)
     * history entry, injecting a position error at speed. Fall back to
     * the current tick - treating the fix as ~now is a far smaller epoch
     * error and keeps GPS aiding alive even if PPS never locks. */
    uint32_t fix_tick = timebase_is_disciplined()
                            ? timebase_itow_ms_to_tick(pvt.itow_ms)
                            : timebase_get_tick();
    float sigma_pos_m = (float) pvt.hacc_mm * 0.001f;
    if (sigma_pos_m < 0.5f) {
        sigma_pos_m = 0.5f; /* floor: hAcc alone understates real error */
    }
    if (kf6_correct_pos(fix_tick, e_m, n_m, u_m, sigma_pos_m)) {
        /* The filter had diverged (GPS outage) and just snapped back onto
         * GPS: that jump is not motion, and must not be tested against
         * the gates as if the car had driven it. */
        s_lap_sample_valid = false;
    }

    float ve_mps = (float) pvt.vel_e_mms * 0.001f;
    float vn_mps = (float) pvt.vel_n_mms * 0.001f;
    float vu_mps = -(float) pvt.vel_d_mms * 0.001f;
    float sigma_vel_mps = (float) pvt.sacc_mms * 0.001f;
    if (sigma_vel_mps < 0.1f) {
        sigma_vel_mps = 0.1f;
    }
    kf6_correct_vel(fix_tick, ve_mps, vn_mps, vu_mps, sigma_vel_mps);

    s_fix_seen = true;
    s_last_fix_tick = now;
}

#if KF_WHEELSPEED_AIDING
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
#endif

/* ------------------------------------------------------------------ */
/* Pitot airspeed / wind (broadcast only)                              */
/* ------------------------------------------------------------------ */

static airdata_t s_air;
static bool s_pitot_seen;
static TickType_t s_last_pitot_tick;
static uint32_t s_last_pitot_us;

/* Feed the newest pitot sample, if one arrived, to the air-data estimator
 * together with the fused ground velocity, and publish the result. Runs
 * after the KF so the velocity is this loop's. Nothing here feeds back
 * into fusion or lap timing. */
static void
handle_pitot(float ve_mps, float vn_mps, bool ground_valid, TickType_t now)
{
    app_pitot_sample_t smp;
    if (xQueueReceive(g_pitot_queue, &smp, 0) == pdTRUE) {
        uint32_t t_us = timebase_get_tick();
        float dt = s_pitot_seen ? (float) (t_us - s_last_pitot_us) * 1e-6f
                                : 0.01f;
        if (dt <= 0.0f || dt > 0.5f) {
            dt = 0.01f; /* first sample, or after a gap: nominal period */
        }
        s_last_pitot_us = t_us;
        s_last_pitot_tick = now;
        s_pitot_seen = true;

        airdata_input_t in = {
            .p_total_mbar = smp.ch_mbar[PITOT_TOTAL_CHANNEL - 1U],
            .p_static_mbar =
                airdata_mean_channels(smp.ch_mbar, PITOT_STATIC_CHANNEL_MASK),
            .temp_c = smp.temp_c,
            .temp_valid = smp.temp_valid,
            .ve_mps = ve_mps,
            .vn_mps = vn_mps,
            .ground_valid = ground_valid,
            .dt_s = dt,
        };
        airdata_update(&s_air, &in);
    }

    bool fresh = s_pitot_seen &&
                 (now - s_last_pitot_tick) < pdMS_TO_TICKS(PITOT_STALE_MS);

    uint8_t flags = 0U;
    float airspeed = 0.0f;
    if (fresh) {
        flags |= CAN_GPS_AIR_FLAG_PITOT_OK;
        airspeed = airdata_airspeed_mps(&s_air);
        if (s_air.temp_used) {
            flags |= CAN_GPS_AIR_FLAG_SENSOR_TEMP;
        }
    }
    if (s_air.tared) {
        flags |= CAN_GPS_AIR_FLAG_TARED;
    }
    if (airdata_wind_valid(&s_air)) {
        flags |= CAN_GPS_AIR_FLAG_WIND_VALID;
    }

    float we, wn;
    airdata_wind(&s_air, &we, &wn);
    float wind_speed = sqrtf(we * we + wn * wn);
    /* Compass bearing the wind blows FROM: that of the reversed vector. */
    float wind_from = atan2f(-we, -wn) * RAD_TO_DEG;
    if (wind_from < 0.0f) {
        wind_from += 360.0f;
    }
    if (wind_from >= 360.0f) {
        wind_from -= 360.0f;
    }

    canbc_state_set_air(airspeed, wind_speed, wind_from,
                        airdata_scale(&s_air), flags);
}

/* ------------------------------------------------------------------ */
/* IMU bring-up / health                                               */
/* ------------------------------------------------------------------ */

static void
set_mag_flags(bool ok)
{
    if (ok) {
        app_clear_events(SYS_EVT_MAG_FAULT);
        app_set_events(SYS_EVT_MAG_READY);
    } else {
        app_clear_events(SYS_EVT_MAG_READY);
        app_set_events(SYS_EVT_MAG_FAULT);
    }
}

/* Probe and configure the accel/gyro, then the magnetometer behind its
 * sensor hub, and set the ready/fault flags to match. Returns whether the
 * accel/gyro came up; *mag_ok reports the magnetometer. */
static bool
imu_bring_up(uint32_t attempts, bool *mag_ok)
{
    *mag_ok = false;

    status_t st = STATUS_ERROR;
    for (uint32_t i = 0U; i < attempts && st != STATUS_OK; i++) {
        if (i > 0U) {
            vTaskDelay(pdMS_TO_TICKS(IMU_INIT_RETRY_MS));
        }
        st = lsm6dso32_init();
    }

    if (st != STATUS_OK) {
        app_clear_events(SYS_EVT_IMU_READY | SYS_EVT_MAG_READY);
        app_set_events(SYS_EVT_IMU_FAULT);
        return false;
    }
    app_clear_events(SYS_EVT_IMU_FAULT);
    app_set_events(SYS_EVT_IMU_READY);

    *mag_ok = (lsm6dso32_mag_init() == STATUS_OK);
    set_mag_flags(*mag_ok);
    return true;
}

static void
count_read_error(void)
{
    if (s_read_error_count < UINT8_MAX) {
        s_read_error_count++;
    }
}

void
imu_task_main(void *argument)
{
    (void) argument;

    s_imu_task_handle = xTaskGetCurrentTaskHandle();

    /* imu_ok: accel/gyro configured and reading. mag_live: magnetometer
     * configured and (once past its start-up grace period) delivering
     * samples. */
    bool mag_live;
    bool imu_ok = imu_bring_up(IMU_INIT_ATTEMPTS, &mag_live);

    ahrs_init();
    airdata_init(&s_air);
    kf6_init();
    gates_init();
    laptimer_init();
    /* Loads the persisted gate table (absolute lat/lon) and the mag
     * calibration; the gates are placed into gates.c later, once the
     * first fix sets the ENU origin. Falls back to identity calibration /
     * no gates if nothing was saved. */
    mag_cal_result_t loaded_cal;
    flash_store_init(s_gate_table, LAP_MAX_GATES, &loaded_cal);
    /* A persisted cal seeds the background calibrator at GOOD so heading
     * works from boot; the continuous sweep refines it and re-validates
     * against course from there. */
    mag_cal_cont_init(&s_mag_cont, &loaded_cal);
    s_last_mag_save = 0U;

    uint32_t prev_tick = timebase_get_tick();
    float prev_e = 0.0f, prev_n = 0.0f;
    uint32_t prev_itow_ms = 0U;
    uint32_t loop_count = 0U;
    uint32_t read_fail_streak = 0U;
    TickType_t last_mag_tick = xTaskGetTickCount();

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
        loop_count++;

        /* About once a second: confirm the sensor still holds its
         * configuration (a sensor that reset itself keeps answering SPI
         * with dead samples), and if it is faulted for any reason, try to
         * bring it back. */
        bool health_tick = (loop_count % IMU_HEALTH_PERIOD_LOOPS) == 0U;
        if (health_tick) {
            if (imu_ok && !lsm6dso32_is_configured()) {
                count_read_error();
                imu_ok = false;
                app_clear_events(SYS_EVT_IMU_READY | SYS_EVT_MAG_READY);
                app_set_events(SYS_EVT_IMU_FAULT);
            }
            if (!imu_ok) {
                imu_ok = imu_bring_up(1U, &mag_live);
                read_fail_streak = 0U;
                last_mag_tick = xTaskGetTickCount();
            }
        }

        lsm6dso32_sample_t s;
        bool have_sample = false;
        if (imu_ok) {
            if (lsm6dso32_read(&s) == STATUS_OK) {
                have_sample = true;
                read_fail_streak = 0U;
            } else {
                count_read_error();
                if (++read_fail_streak < IMU_FAULT_CONSECUTIVE_READS) {
                    continue; /* a glitch: skip this cycle */
                }
                imu_ok = false;
                app_clear_events(SYS_EVT_IMU_READY);
                app_set_events(SYS_EVT_IMU_FAULT);
            }
        }

        apply_mag_cal_commands();

        TickType_t now = xTaskGetTickCount();

        /* Without the IMU the node degrades to GPS-only rather than dying:
         * a level, non-rotating, unaccelerated sample makes the filter a
         * constant-velocity model between fixes, paced by the 20 ms
         * notification timeout above. Position, speed and lap timing keep
         * working at GPS quality; attitude is meaningless and the fusion
         * status says so (IMU_OK clear). */
        vec3_t a = {0.0f, 0.0f, 1.0f};
        vec3_t g = {0.0f, 0.0f, 0.0f};
        float mx = 0.0f, my = 0.0f, mz = 0.0f;

        if (have_sample) {
            a = apply_mount(s.ax_g, s.ay_g, s.az_g);
            g = apply_mount(s.gx_dps, s.gy_dps, s.gz_dps);

            if (health_tick) {
                canbc_state_set_imu_temp(s.temp_c);
            }

            lsm6dso32_mag_sample_t m;
            if (lsm6dso32_read_mag(&m) == STATUS_OK && m.valid) {
                vec3_t mv = apply_mag_mount(m.mx_ut, m.my_ut, m.mz_ut);

                mag_cal_cont_feed(&s_mag_cont, mv.x, mv.y, mv.z);
                mag_cal_apply(mag_cal_cont_result(&s_mag_cont), mv.x, mv.y,
                              mv.z, &mx, &my, &mz);

                canbc_state_set_mag(
                    mx, my, mz, (uint8_t) mag_cal_cont_quality(&s_mag_cont));

                last_mag_tick = now;
                if (!mag_live) {
                    mag_live = true;
                    set_mag_flags(true);
                }
            } else if (mag_live && (now - last_mag_tick) >=
                                       pdMS_TO_TICKS(MAG_STALE_MS)) {
                /* Configured, but it has stopped converting. The AHRS
                 * carries on from gyro + accel (yaw unreferenced). */
                mag_live = false;
                set_mag_flags(false);
            }
        }

        quat_t q;
        ahrs_update(g.x * DEG_TO_RAD, g.y * DEG_TO_RAD, g.z * DEG_TO_RAD,
                    a.x, a.y, a.z, mx, my, mz, dt, &q);

        kf6_predict(tick, a.x * IMU_GRAVITY_MPS2, a.y * IMU_GRAVITY_MPS2,
                    a.z * IMU_GRAVITY_MPS2, q, dt);

        handle_gps_pvt();
#if KF_WHEELSPEED_AIDING
        handle_wheelspeed(q);
#endif
        apply_gate_commands(q);

        /* GPS freshness. A receiver that stops talking altogether can't
         * report its own loss of fix, so the last status must expire. */
        bool pvt_fresh =
            s_pvt_seen &&
            (now - s_last_pvt_tick) < pdMS_TO_TICKS(GPS_FIX_STALE_MS);
        bool gps_aided =
            s_fix_seen &&
            (now - s_last_fix_tick) < pdMS_TO_TICKS(GPS_FIX_STALE_MS);
        if (!pvt_fresh) {
            s_fix_type = 0U;
            s_num_sv = 0U;
        }

        float e_m, n_m, u_m, ve_mps, vn_mps;
        kf6_get_state(&e_m, &n_m, &u_m, &ve_mps, &vn_mps, NULL);

        double fused_lat_deg, fused_lon_deg;
        float fused_height_m;
        geodesy_from_enu(e_m, n_m, u_m, &fused_lat_deg, &fused_lon_deg,
                          &fused_height_m);
        /* Only publish an absolute position once the ENU frame is
         * anchored; before that geodesy has no origin and the fused
         * lat/lon are a meaningless (0,0)-relative value (the divide-by-
         * zero guard in geodesy_from_enu keeps it finite regardless). */
        if (s_frame_origin_set) {
            canbc_state_set_position(fused_lat_deg, fused_lon_deg);
        }

        float fused_speed_mps = sqrtf(ve_mps * ve_mps + vn_mps * vn_mps);
        float fused_course_deg = atan2f(ve_mps, vn_mps) * RAD_TO_DEG;
        if (fused_course_deg < 0.0f) {
            fused_course_deg += 360.0f;
        }
        canbc_state_set_velocity(fused_speed_mps, fused_course_deg,
                                  fused_height_m, s_fix_type, s_num_sv);

        /* Lap timing. Crossings are stamped in GPS time, so nothing is
         * timed until the time base is locked: before that iTOW reads 0,
         * and a lap started then would be measured against the real iTOW
         * once it appears - a "lap" of days. The first sample after the
         * lock (or after the frame is anchored) only primes prev_*. */
        bool time_ok = timebase_is_disciplined();
        uint32_t itow_ms = timebase_tick_to_itow_ms(tick);
        uint16_t lap_before = laptimer_get_lap_count();
        uint8_t lap_events = 0U;

        if (time_ok && s_lap_sample_valid) {
            lap_events = laptimer_update(e_m, n_m, prev_e, prev_n, itow_ms,
                                          prev_itow_ms);
        }
        s_lap_sample_valid = time_ok && s_frame_origin_set;

        /* A lap's final sector closes with the lap: report it first, and
         * against the lap it belongs to (the count before this crossing),
         * as every other sector of that lap was. */
        if ((lap_events & LAPTIMER_EVT_SECTOR) != 0U) {
            can_lap_event_t evt = {CAN_LAP_EVENT_SECTOR, lap_before,
                                    laptimer_get_last_sector_ms(), 0U};
            xQueueSend(g_lap_event_queue, &evt, 0);
        }
        if ((lap_events & LAPTIMER_EVT_LAP) != 0U) {
            can_lap_event_t evt = {CAN_LAP_EVENT_LAP,
                                    laptimer_get_lap_count(),
                                    laptimer_get_last_lap_ms(), 0U};
            xQueueSend(g_lap_event_queue, &evt, 0);
        }

        uint8_t lap_flags = 0U;
        if (laptimer_is_running()) {
            lap_flags |= CAN_LAP_FLAG_RUNNING;
        }
        if (s_frame_origin_set && s_gate_table[0].valid) {
            lap_flags |= CAN_LAP_FLAG_ARMED;
        }
        if (!gps_aided || !time_ok) {
            lap_flags |= CAN_LAP_FLAG_DEGRADED;
        }
        canbc_state_set_lap(laptimer_get_lap_count(),
                             laptimer_get_current_elapsed_ms(itow_ms),
                             laptimer_get_current_sector(), lap_flags);

        uint8_t fusion_status = 0U;
        if (have_sample) {
            fusion_status |= CAN_FUSION_STATUS_IMU_OK;
        }
        if (have_sample && mag_live) {
            fusion_status |= CAN_FUSION_STATUS_MAG_OK;
            if (mag_cal_cont_quality(&s_mag_cont) >= MAG_CAL_GOOD) {
                fusion_status |= CAN_FUSION_STATUS_MAG_CAL_OK;
            }
        }
        if (s_frame_origin_set) {
            fusion_status |= CAN_FUSION_STATUS_ORIGIN_SET;
        }
        if (gps_aided) {
            fusion_status |= CAN_FUSION_STATUS_GPS_AIDED;
        }
        if (time_ok) {
            fusion_status |= CAN_FUSION_STATUS_TIME_LOCKED;
        }

        float yaw_deg, pitch_deg, roll_deg;
        quat_to_euler_deg(q, &yaw_deg, &pitch_deg, &roll_deg);
        canbc_state_set_attitude(yaw_deg, pitch_deg, roll_deg,
                                  fusion_status);

        /* Cross-check the mag-derived heading against course-over-ground,
         * but only when moving fast enough for course to be meaningful
         * (same floor as gate placement) and while GPS is actually
         * steering the velocity estimate. This is what promotes the cal
         * to VALIDATED. */
        if (gps_aided && fused_speed_mps >= LAP_GATE_COURSE_MIN_SPEED_MPS) {
            mag_cal_cont_observe_heading(&s_mag_cont, yaw_deg * DEG_TO_RAD,
                                         fused_course_deg * DEG_TO_RAD);
        }

        /* Persist an improved cal, but rate-limited: the background sweep
         * only marks dirty on a material change, and once converged it
         * stops changing - so in steady state this never writes. The
         * interval floor caps flash wear during initial convergence. */
        bool save_due =
            (s_last_mag_save == 0U) ||
            (now - s_last_mag_save) >=
                pdMS_TO_TICKS(MAG_CAL_SAVE_MIN_INTERVAL_MS);
        if (save_due && mag_cal_cont_take_dirty(&s_mag_cont)) {
            note_save(
                flash_store_save_mag_cal(mag_cal_cont_result(&s_mag_cont)));
            s_last_mag_save = now;
        }

        canbc_state_set_imu_accel(a.x * 1000.0f, a.y * 1000.0f,
                                   a.z * 1000.0f);
        canbc_state_set_imu_gyro(g.x, g.y, g.z);

        handle_pitot(ve_mps, vn_mps, gps_aided, now);

        service_flash(fused_speed_mps);
        publish_flash_state();

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
    /* MX_GPIO_Init() enables this EXTI line before the scheduler (and
     * hence imu_task_main(), which sets s_imu_task_handle as its first
     * line) ever runs. After a warm MCU reset the sensor still holds its
     * configuration and keeps asserting INT1 at 104 Hz, so an edge can
     * land in that window - which would otherwise notify a NULL handle
     * and HardFault. */
    if (s_imu_task_handle == NULL) {
        return;
    }
    BaseType_t hp_woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_imu_task_handle, &hp_woken);
    portYIELD_FROM_ISR(hp_woken);
}
