/**
 * @file    gps_task.h
 * @brief   Entry point for gps_task (see app.c for creation).
 */

#ifndef GPS_TASK_H
#define GPS_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void gps_task_main(void *argument);

/** @brief GPS link recoveries since boot - UART RX re-arms plus receiver
 *         re-configurations (saturating), for GPS_Status. Safe to call
 *         from any task. */
uint8_t gps_task_get_retry_count(void);

#ifdef __cplusplus
}
#endif

#endif /* GPS_TASK_H */
