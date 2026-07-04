/**
 * @file    app.c
 * @brief   Static task/queue/IPC creation and system event flags.
 *
 * Fully static: every task, queue and mutex used anywhere in the
 * firmware is created here up front, before the scheduler starts (except
 * canbc_state's own mutex, created by canbc_state_init() - called from
 * here regardless, so it still happens before any task runs). Nothing is
 * created or destroyed at runtime.
 */

#include "sys/app.h"

#include "FreeRTOS.h"
#include "event_groups.h"
#include "semphr.h"
#include "task.h"

#include "canbus/canbc.h"
#include "canbus/canbc_task.h"
#include "gps/gps_task.h"
#include "imu/imu_task.h"
#include "nmea/aux_task.h"
#include "sys/sys_task.h"

/* ------------------------------------------------------------------ */
/* Queues                                                              */
/* ------------------------------------------------------------------ */

QueueHandle_t g_can_rx_cmd_queue;
QueueHandle_t g_lap_event_queue;
QueueHandle_t g_gate_cmd_queue;
QueueHandle_t g_sys_cmd_queue;
QueueHandle_t g_mag_cal_cmd_queue;
QueueHandle_t g_nmea_cfg_queue;
QueueHandle_t g_wheelspeed_queue;
QueueHandle_t g_gps_pvt_queue;
QueueHandle_t g_gps_pvt_queue_nmea;

#define CAN_RX_CMD_QUEUE_LEN 4U
#define LAP_EVENT_QUEUE_LEN  4U
#define GATE_CMD_QUEUE_LEN   4U
#define SYS_CMD_QUEUE_LEN    4U
#define MAG_CAL_CMD_QUEUE_LEN 2U
#define NMEA_CFG_QUEUE_LEN   2U
#define WHEELSPEED_QUEUE_LEN 1U /* overwritten with the latest reading */
#define GPS_PVT_QUEUE_LEN    1U /* overwritten with the latest fix */
#define GPS_PVT_QUEUE_NMEA_LEN 1U

static StaticQueue_t s_can_rx_cmd_qcb;
static uint8_t s_can_rx_cmd_storage[CAN_RX_CMD_QUEUE_LEN *
                                    sizeof(can_gps_command_t)];

static StaticQueue_t s_lap_event_qcb;
static uint8_t s_lap_event_storage[LAP_EVENT_QUEUE_LEN *
                                   sizeof(can_lap_event_t)];

static StaticQueue_t s_gate_cmd_qcb;
static uint8_t s_gate_cmd_storage[GATE_CMD_QUEUE_LEN * sizeof(app_cmd_t)];

static StaticQueue_t s_sys_cmd_qcb;
static uint8_t s_sys_cmd_storage[SYS_CMD_QUEUE_LEN * sizeof(app_cmd_t)];

static StaticQueue_t s_mag_cal_cmd_qcb;
static uint8_t
    s_mag_cal_cmd_storage[MAG_CAL_CMD_QUEUE_LEN * sizeof(app_cmd_t)];

static StaticQueue_t s_nmea_cfg_qcb;
static uint8_t s_nmea_cfg_storage[NMEA_CFG_QUEUE_LEN * sizeof(app_cmd_t)];

static StaticQueue_t s_wheelspeed_qcb;
static uint8_t s_wheelspeed_storage[WHEELSPEED_QUEUE_LEN * sizeof(float)];

static StaticQueue_t s_gps_pvt_qcb;
static uint8_t s_gps_pvt_storage[GPS_PVT_QUEUE_LEN * sizeof(ubx_nav_pvt_t)];

static StaticQueue_t s_gps_pvt_nmea_qcb;
static uint8_t
    s_gps_pvt_nmea_storage[GPS_PVT_QUEUE_NMEA_LEN * sizeof(ubx_nav_pvt_t)];

/* ------------------------------------------------------------------ */
/* Tasks                                                               */
/* ------------------------------------------------------------------ */

static StaticTask_t s_imu_tcb;
static StackType_t s_imu_stack[APP_STACK_IMU];

static StaticTask_t s_gps_tcb;
static StackType_t s_gps_stack[APP_STACK_GPS];

static StaticTask_t s_can_tcb;
static StackType_t s_can_stack[APP_STACK_CAN];

static StaticTask_t s_aux_tcb;
static StackType_t s_aux_stack[APP_STACK_AUX];

static StaticTask_t s_sys_tcb;
static StackType_t s_sys_stack[APP_STACK_SYS];

/* ------------------------------------------------------------------ */
/* System event flags                                                  */
/* ------------------------------------------------------------------ */

static StaticEventGroup_t s_events_buf;
static EventGroupHandle_t s_events;

static StaticSemaphore_t s_gps_tx_mutex_buf;
static SemaphoreHandle_t s_gps_tx_mutex;

void
app_gps_tx_lock(void)
{
    xSemaphoreTake(s_gps_tx_mutex, portMAX_DELAY);
}

void
app_gps_tx_unlock(void)
{
    xSemaphoreGive(s_gps_tx_mutex);
}

void
app_set_events(uint32_t bits)
{
    xEventGroupSetBits(s_events, bits);
}

void
app_clear_events(uint32_t bits)
{
    xEventGroupClearBits(s_events, bits);
}

uint32_t
app_get_events(void)
{
    return xEventGroupGetBits(s_events);
}

void
app_init(void)
{
    s_events = xEventGroupCreateStatic(&s_events_buf);
    s_gps_tx_mutex = xSemaphoreCreateMutexStatic(&s_gps_tx_mutex_buf);

    canbc_state_init();

    g_can_rx_cmd_queue =
        xQueueCreateStatic(CAN_RX_CMD_QUEUE_LEN, sizeof(can_gps_command_t),
                            s_can_rx_cmd_storage, &s_can_rx_cmd_qcb);
    g_lap_event_queue =
        xQueueCreateStatic(LAP_EVENT_QUEUE_LEN, sizeof(can_lap_event_t),
                            s_lap_event_storage, &s_lap_event_qcb);
    g_gate_cmd_queue =
        xQueueCreateStatic(GATE_CMD_QUEUE_LEN, sizeof(app_cmd_t),
                            s_gate_cmd_storage, &s_gate_cmd_qcb);
    g_sys_cmd_queue =
        xQueueCreateStatic(SYS_CMD_QUEUE_LEN, sizeof(app_cmd_t),
                            s_sys_cmd_storage, &s_sys_cmd_qcb);
    g_mag_cal_cmd_queue =
        xQueueCreateStatic(MAG_CAL_CMD_QUEUE_LEN, sizeof(app_cmd_t),
                            s_mag_cal_cmd_storage, &s_mag_cal_cmd_qcb);
    g_nmea_cfg_queue =
        xQueueCreateStatic(NMEA_CFG_QUEUE_LEN, sizeof(app_cmd_t),
                            s_nmea_cfg_storage, &s_nmea_cfg_qcb);
    g_wheelspeed_queue =
        xQueueCreateStatic(WHEELSPEED_QUEUE_LEN, sizeof(float),
                            s_wheelspeed_storage, &s_wheelspeed_qcb);
    g_gps_pvt_queue =
        xQueueCreateStatic(GPS_PVT_QUEUE_LEN, sizeof(ubx_nav_pvt_t),
                            s_gps_pvt_storage, &s_gps_pvt_qcb);
    g_gps_pvt_queue_nmea =
        xQueueCreateStatic(GPS_PVT_QUEUE_NMEA_LEN, sizeof(ubx_nav_pvt_t),
                            s_gps_pvt_nmea_storage, &s_gps_pvt_nmea_qcb);

    xTaskCreateStatic(imu_task_main, "imu", APP_STACK_IMU, NULL,
                       APP_PRIO_IMU, s_imu_stack, &s_imu_tcb);
    xTaskCreateStatic(gps_task_main, "gps", APP_STACK_GPS, NULL,
                       APP_PRIO_GPS, s_gps_stack, &s_gps_tcb);
    xTaskCreateStatic(canbc_task_main, "can", APP_STACK_CAN, NULL,
                       APP_PRIO_CAN, s_can_stack, &s_can_tcb);
    xTaskCreateStatic(aux_task_main, "aux", APP_STACK_AUX, NULL,
                       APP_PRIO_AUX, s_aux_stack, &s_aux_tcb);
    xTaskCreateStatic(sys_task_main, "sys", APP_STACK_SYS, NULL,
                       APP_PRIO_SYS, s_sys_stack, &s_sys_tcb);
}

/* ------------------------------------------------------------------ */
/* FreeRTOS static-allocation / diagnostic hooks                       */
/* ------------------------------------------------------------------ */

static StaticTask_t s_idle_tcb;
static StackType_t s_idle_stack[configMINIMAL_STACK_SIZE];

void
vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                               StackType_t **ppxIdleTaskStackBuffer,
                               uint32_t *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer = &s_idle_tcb;
    *ppxIdleTaskStackBuffer = s_idle_stack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void
vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void) xTask;
    (void) pcTaskName;
    taskDISABLE_INTERRUPTS();
    for (;;) {
    }
}

void
vApplicationIdleHook(void)
{
}
