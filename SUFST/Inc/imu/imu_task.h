/**
 * @file    imu_task.h
 * @brief   Entry point for imu_task (see app.c for creation).
 */

#ifndef IMU_TASK_H
#define IMU_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void imu_task_main(void *argument);

/** @brief Failed IMU reads since boot (saturating), for GPS_Status.
 *         Safe to call from any task. */
uint8_t imu_task_get_error_count(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_TASK_H */
