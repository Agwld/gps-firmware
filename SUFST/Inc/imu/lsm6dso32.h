/**
 * @file    lsm6dso32.h
 * @brief   LSM6DSO32 accel+gyro driver, plus IIS2MDC magnetometer bring-up
 *          via the LSM6DSO32's sensor-hub I2C master (AN5192).
 *
 * Register addresses/bit positions (WHO_AM_I=0x6C, CTRL1_XL/CTRL2_G/
 * CTRL3_C layout and reset defaults, OUT_TEMP_L..OUTZ_H_A map, the SHUB
 * bank's FUNC_CFG_ACCESS/SLV0_ADD/SLV0_SUBADD/SLAVE0_CONFIG/
 * MASTER_CONFIG registers, and the IIS2MDC's 0x1E address /
 * OUTX_L_REG=0x68 / 1.5 mgauss-per-LSB sensitivity) have been checked
 * bit-by-bit against the LSM6DSO32 and IIS2MDC datasheets, and the
 * sensor-hub bring-up sequence against AN5192 (Section 7.2/7.4) and the
 * gps-mainboard netlist. This caught two real bugs, both now fixed:
 * CTRL3_C's IF_INC bit being cleared by the BDU-only write (would have
 * broken every multi-byte burst read), and MASTER_CONFIG's WRITE_ONCE
 * bit not being set - AN5192 states this is mandatory for slave 0 read
 * transactions despite the name, and its own reference example sets it.
 * SHUB_PU_EN is deliberately left clear: R27/R28 on the board already
 * pull the sensor-hub I2C lines up to 3V3 (confirmed in the netlist),
 * matching the "external pull-ups present" case AN5192 describes for
 * that bit, not its example (which has no external pull-ups).
 *
 * lsm6dso32_mag_init() is a one-shot call (never re-run to reconfigure
 * a running sensor hub), so AN5192's disable-master-then-wait-300us
 * procedure - required only when changing an *already-running*
 * configuration - doesn't apply here; if a retry-after-fault path is
 * ever added that calls this a second time, add that procedure first.
 * Still worth a logic analyzer trace of the IIS2MDC actually responding
 * before trusting mag data on the bench, as with any first bring-up.
 */

#ifndef LSM6DSO32_H
#define LSM6DSO32_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "sys/status.h"

typedef struct {
    float ax_g, ay_g, az_g; /* calibrated, board mount matrix NOT applied */
    float gx_dps, gy_dps, gz_dps;
    float temp_c;
} lsm6dso32_sample_t;

typedef struct {
    float mx_ut, my_ut, mz_ut; /* board mount matrix NOT applied */
    bool valid; /* sensor-hub mag word freshness (SENSORHUB updated) */
} lsm6dso32_mag_sample_t;

/**
 * @brief Probe WHO_AM_I and configure the accel/gyro at the ODR/FS in
 *        board_config.h (BDU on, so a read always returns a consistent
 *        accel+gyro pair even if it races the sensor's own update).
 * @return STATUS_OK, or STATUS_TIMEOUT if WHO_AM_I didn't match.
 */
status_t lsm6dso32_init(void);

/** @brief Bring up the sensor-hub I2C master and configure it to poll
 *         the IIS2MDC magnetometer (I2C address 0x1E) once per
 *         accel/gyro ODR cycle, per AN5192. Call after lsm6dso32_init(). */
status_t lsm6dso32_mag_init(void);

/** @brief Read one accel+gyro sample over SPI via DMA. The calling task
 *         blocks on a completion semaphore (yielding the CPU) for the
 *         duration of the transfer rather than spinning; must be called
 *         from a FreeRTOS task context, after lsm6dso32_init(). */
status_t lsm6dso32_read(lsm6dso32_sample_t *out);

/** @brief DMA read of the sensor-hub's latched IIS2MDC output (same
 *         task-blocking semantics as lsm6dso32_read()). */
status_t lsm6dso32_read_mag(lsm6dso32_mag_sample_t *out);

#ifdef __cplusplus
}
#endif

#endif /* LSM6DSO32_H */
