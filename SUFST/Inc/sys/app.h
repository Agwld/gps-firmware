/**
 * @file    app.h
 * @brief   Application start-up and system-wide event flags.
 */

#ifndef APP_H
#define APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "FreeRTOS.h"
#include "queue.h"

#include "canbus/can_defs.h"
#include "gps/ubx.h"

/* System event group bits (owned by app.c, set by the owning tasks) */
#define SYS_EVT_GPS_READY   (1U << 0)
#define SYS_EVT_IMU_READY   (1U << 1)
#define SYS_EVT_MAG_READY   (1U << 2)
#define SYS_EVT_TIME_LOCKED (1U << 3)
#define SYS_EVT_ORIGIN_SET  (1U << 4)
#define SYS_EVT_GPS_FAULT   (1U << 5)
#define SYS_EVT_IMU_FAULT   (1U << 6)
#define SYS_EVT_MAG_FAULT   (1U << 7)
#define SYS_EVT_PVT_TICK    (1U << 8) /* pulsed each published PVT (aux) */

/* Task priorities (higher = more urgent, per FreeRTOSConfig.h's 8 levels) */
#define APP_PRIO_IMU  6
#define APP_PRIO_GPS  5
#define APP_PRIO_CAN  3
#define APP_PRIO_AUX  2
#define APP_PRIO_SYS  1

/* Task stack sizes, in words (FreeRTOS StackType_t = uint32_t here) */
#define APP_STACK_IMU  (2048U / 4U)
#define APP_STACK_GPS  (1536U / 4U)
#define APP_STACK_CAN  (1024U / 4U)
#define APP_STACK_AUX  (1024U / 4U)
#define APP_STACK_SYS  (1280U / 4U)

/* Generic 3-byte command, same shape as can_gps_command_t, reused for
 * every can_task -> owning-task dispatch queue below so a single struct
 * covers gate/mag-cal/config/nmea commands without type proliferation. */
typedef struct {
    uint8_t cmd;
    uint8_t arg0;
    uint8_t arg1;
} app_cmd_t;

/*
 * Inter-task queues (created in app_init(), never resized/deleted).
 * Direction is producer -> consumer; every queue has exactly one task on
 * each end, so none of these need a mutex on top of the queue itself.
 */

/** FDCAN RX ISR -> can_task: raw incoming GPS_Command frames. */
extern QueueHandle_t g_can_rx_cmd_queue;

/** imu_task -> can_task: Lap_Event frames to transmit immediately
 * (lap/sector completion, TM2 time-marks). */
extern QueueHandle_t g_lap_event_queue;

/** can_task -> imu_task: gate set/clear commands (CAN_CMD_GATE_*). imu_task
 * owns gates.c since it's the only task calling gates_check_crossing(). */
extern QueueHandle_t g_gate_cmd_queue;

/** can_task -> sys_task: config-save commands (CAN_CMD_CONFIG_SAVE).
 * sys_task owns flash_store.c and judges when it's safe (car stationary)
 * to erase-and-compact. */
extern QueueHandle_t g_sys_cmd_queue;

/** can_task -> imu_task: mag-cal start/stop (CAN_CMD_MAG_CAL_*).
 * imu_task, not sys_task, owns this because it's the only task with
 * raw magnetometer samples each cycle to feed the accumulator. */
extern QueueHandle_t g_mag_cal_cmd_queue;

/** can_task -> aux_task: NMEA output rate/enable (CAN_CMD_NMEA_CFG). */
extern QueueHandle_t g_nmea_cfg_queue;

/** can_task -> imu_task: scalar wheelspeed (m/s, averaged from the VCU's
 * Wheel_Speeds frame, 0x251) for kf6_correct_speed() aiding. */
extern QueueHandle_t g_wheelspeed_queue;

/** gps_task -> imu_task: latest valid-fix NAV-PVT, overwritten (length
 * 1) since only the newest fix matters - imu_task owns the geodesy
 * origin and kf6 correction. */
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

/** @brief Set (and never clear) system event bits. ISR-safe variant absent
 *         by design: call from task context only. */
void app_set_events(uint32_t bits);
void app_clear_events(uint32_t bits);
uint32_t app_get_events(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
