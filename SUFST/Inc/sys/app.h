/**
 * @file    app.h
 * @brief   Application start-up and system-wide event flags.
 */

#ifndef APP_H
#define APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "queue.h"

#include "canbus/can_defs.h"
#include "gps/ubx.h"

/* System event group bits (owned by app.c, set/cleared by the owning
 * tasks). Broadcast verbatim as GPS_Status.fault_bits, so these values
 * are part of the CAN interface - don't renumber them. */
#define SYS_EVT_GPS_READY   (1U << 0) /* receiver reports a valid fix */
#define SYS_EVT_IMU_READY   (1U << 1) /* accel/gyro configured and reading */
#define SYS_EVT_MAG_READY   (1U << 2) /* magnetometer configured */
#define SYS_EVT_TIME_LOCKED (1U << 3) /* PPS has disciplined the timebase */
#define SYS_EVT_ORIGIN_SET  (1U << 4) /* ENU frame anchored on a fix */
#define SYS_EVT_GPS_FAULT   (1U << 5) /* no NAV-PVT stream from the receiver */
#define SYS_EVT_IMU_FAULT   (1U << 6) /* IMU absent or reads failing */
#define SYS_EVT_MAG_FAULT   (1U << 7) /* magnetometer bring-up failed */

/* Task priorities (higher = more urgent, per FreeRTOSConfig.h's 8 levels) */
#define APP_PRIO_IMU  6
#define APP_PRIO_GPS  5
#define APP_PRIO_CAN  3
#define APP_PRIO_AUX  2
#define APP_PRIO_SYS  1

/* Task stack sizes, in words (FreeRTOS StackType_t = uint32_t here).
 *
 * Sized from -fstack-usage frame sizes plus ~240 bytes for the context
 * the kernel and an interrupt entry park on a task's own stack (core +
 * FPU registers). The two deep chains, in bytes (Release / Debug):
 *
 *   imu:  imu_task_main -> kf6_correct_pos -> correct_with_rewind ->
 *         kf_update (several 6x6 float matrices)       ~1600 / ~1900
 *   gps:  gps_task_main -> gps_config boot sequence (VALSET frame and
 *         payload buffers) -> HAL I2C                   ~1100 / ~1250
 *
 * Re-measure before shrinking any of these, and after adding large
 * locals to those paths: configCHECK_FOR_STACK_OVERFLOW turns an
 * overflow into a reset loop, not a silent corruption, but it is still a
 * dead node. */
#define APP_STACK_IMU  (3072U / 4U)
#define APP_STACK_GPS  (2048U / 4U)
#define APP_STACK_CAN  (1024U / 4U)
#define APP_STACK_AUX  (1024U / 4U)
#define APP_STACK_SYS  (1280U / 4U)

/* Generic 3-byte command, same shape as can_gps_command_t, reused for
 * every dispatch queue below so a single struct covers gate/mag-cal/
 * config/nmea commands without type proliferation. */
typedef struct {
    uint8_t cmd;
    uint8_t arg0;
    uint8_t arg1;
} app_cmd_t;

/*
 * Inter-task queues (created in app_init(), never resized/deleted).
 * Direction is producer -> consumer; every queue has exactly one consumer,
 * so none of these need a mutex on top of the queue itself.
 */

/** FDCAN RX ISR -> can_task: raw incoming GPS_Command frames. */
extern QueueHandle_t g_can_rx_cmd_queue;

/** imu_task -> can_task: Lap_Event frames to transmit immediately
 * (lap/sector completion, TM2 time-marks). */
extern QueueHandle_t g_lap_event_queue;

/** can_task / sys_task -> imu_task: gate set/clear (CAN_CMD_GATE_*, from
 * CAN or the lap button) and config-save (CAN_CMD_CONFIG_SAVE). imu_task
 * owns gates.c - it's the only task calling gates_check_crossing() - and
 * is the sole user of flash_store.c, so every command that changes or
 * persists the gate table funnels through this one queue. The only queue
 * with two producers; FreeRTOS queues are multi-producer safe. */
extern QueueHandle_t g_gate_cmd_queue;

/** can_task -> imu_task: mag-cal start/stop (CAN_CMD_MAG_CAL_*).
 * imu_task, not sys_task, owns this because it's the only task with
 * raw magnetometer samples each cycle to feed the accumulator. */
extern QueueHandle_t g_mag_cal_cmd_queue;

/** can_task -> aux_task: NMEA output rate/enable (CAN_CMD_NMEA_CFG). */
extern QueueHandle_t g_nmea_cfg_queue;

/** FDCAN RX ISR -> imu_task: scalar wheelspeed (m/s, the front-left wheel
 * from the VCU's Wheel_Speeds frame, 0x251) for kf6_correct_speed(). Only
 * consumed when KF_WHEELSPEED_AIDING is enabled (board_config.h). */
extern QueueHandle_t g_wheelspeed_queue;

/** One EvoScann P4-A pitot sample as forwarded from the CAN RX ISR: the
 * four pressures of the latest 0x190 frame, plus the sensor temperature
 * from the latest 0x191 status frame (if one has arrived). */
typedef struct {
    float ch_mbar[4];
    float temp_c;
    bool temp_valid;
} app_pitot_sample_t;

/** FDCAN RX ISR -> imu_task: latest pitot sample, overwritten (length 1).
 * imu_task runs the air-data estimator because it owns the fused ground
 * velocity the wind fit needs. */
extern QueueHandle_t g_pitot_queue;

/** gps_task -> imu_task: latest NAV-PVT, overwritten (length 1) since
 * only the newest one matters - imu_task owns the geodesy origin and kf6
 * correction. Every NAV-PVT is forwarded, fix or not: the consumer checks
 * gnssFixOK itself, so a lost fix is seen as such instead of the last
 * good one lingering. */
extern QueueHandle_t g_gps_pvt_queue;

/** gps_task -> aux_task: same NAV-PVT, duplicated to its own queue
 * rather than shared with g_gps_pvt_queue - two independent
 * xQueueReceive() consumers on one queue would steal each other's
 * fixes. */
extern QueueHandle_t g_gps_pvt_queue_nmea;

/**
 * @brief Create all tasks, queues and IPC objects (static) - called from
 *        main() before the scheduler starts.
 */
void app_init(void);

/** @brief Lock/unlock the I2C2 bus, shared between gps_task's boot
 *         config (F9P DDC port, gps_i2c.c) and sys_task's MCP9800
 *         temperature reads. One of this firmware's two mutexes (the
 *         other is canbc_state's). */
void app_i2c_lock(void);
void app_i2c_unlock(void);

/** @brief Set / clear / read system event bits. ISR-safe variants are
 *         absent by design: call from task context only. */
void app_set_events(uint32_t bits);
void app_clear_events(uint32_t bits);
uint32_t app_get_events(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
