/**
 * @file    gps_config.h
 * @brief   ZED-F9P boot configuration: UBX-CFG-VALSET over I2C.
 *
 * The board dictates the transport: there is no MCU->GPS UART path (see
 * gps_i2c.h), so all configuration - including retuning the F9P's UART1
 * itself - rides the I2C port. Every CFG-* key ID used has been checked
 * against the u-blox Interface Description (UBX-22008968) except
 * CFG-UART1INPROT-RTCM3X, added with the I2C rework - give that one the
 * same treatment before trusting RTK input on the bench. The VALGET
 * readback verify step (rather than trusting the VALSET ACK) is the
 * safety net for a wrong key silently configuring the wrong thing.
 */

#ifndef GPS_CONFIG_H
#define GPS_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "gps/ubx.h"
#include "sys/status.h"

/**
 * @brief Run the boot configuration sequence over I2C: probe until the
 *        receiver answers (it takes up to ~1 s from power-up), then one
 *        VALSET batch - UART1 to GPS_UART_BAUD, UBX-only out, RTCM3X in
 *        (corrections arrive on UART1 straight from the RS232 input via
 *        JP7), GPS+Galileo only, 20 Hz measurement rate, automotive
 *        dynamic model, NAV-PVT + TIM-TM2 on UART1 - verified by VALGET
 *        readback of the baud key, never by the ACK alone.
 *
 * Blocking but yielding (polls sleep via vTaskDelay): call from
 * gps_task before entering its main receive loop, after the scheduler
 * has started.
 *
 * @return STATUS_OK once fully configured and verified; STATUS_TIMEOUT
 *         if the receiver never answered on I2C; STATUS_ERROR if the
 *         config was NAK'd or the VALGET readback didn't match.
 */
status_t gps_config_run_boot_sequence(void);

#ifdef __cplusplus
}
#endif

#endif /* GPS_CONFIG_H */
