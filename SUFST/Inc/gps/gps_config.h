/**
 * @file    gps_config.h
 * @brief   ZED-F9P boot configuration: auto-baud probe + UBX-CFG-VALSET.
 *
 * IMPORTANT: the CFG-* key IDs and their byte widths in gps_config.c are
 * transcribed from memory of the u-blox "Interface Description" for
 * M9/F9 receivers, not re-checked against the actual document in this
 * session. Cross-check every key in gps_config.c's kv table against the
 * ZED-F9P interface manual before relying on this on the bench - the
 * VALGET readback verify step this module performs (rather than trusting
 * the VALSET ACK) is exactly the safety net for a wrong key silently
 * configuring the wrong thing.
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
 * @brief Run the boot configuration sequence: probe baud (try
 *        GPS_BAUD_TARGET first, fall back to GPS_BAUD_DEFAULT), raise it
 *        to GPS_BAUD_TARGET if needed (VALSET, verified by VALGET
 *        readback - never by the baud-change ACK, which is transmitted
 *        at the old baud and reads as garbage at the new one), then
 *        apply the steady-state configuration (UBX-only out, GPS+Galileo
 *        only, 20 Hz measurement rate, automotive dynamic model,
 *        NAV-PVT + TIM-TM2 enabled on UART1).
 *
 * Blocking: sends frames and waits for ACK/VALGET responses with
 * bounded timeouts, retrying a bounded number of times. Call from
 * gps_task before entering its main receive loop.
 *
 * @return STATUS_OK once fully configured and verified; STATUS_TIMEOUT
 *         if the receiver never responded at any baud; STATUS_ERROR if
 *         VALGET readback didn't match what was requested.
 */
status_t gps_config_run_boot_sequence(void);

#ifdef __cplusplus
}
#endif

#endif /* GPS_CONFIG_H */
