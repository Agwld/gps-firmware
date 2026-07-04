/**
 * @file    lsm6dso32.h
 * @brief   LSM6DSO32 accel+gyro driver, plus IIS2MDC magnetometer bring-up
 *          via the LSM6DSO32's sensor-hub I2C master (AN5192).
 *
 * IMPORTANT: register addresses/bit positions below are transcribed from
 * memory of the LSM6DSO32 datasheet and AN5192 (sensor-hub application
 * note), not re-checked against either document in this session. The
 * core WHO_AM_I / CTRL1_XL / CTRL2_G / output-register set is common and
 * stable across the whole ST LSM6DSx family and is held with reasonably
 * high confidence; the sensor-hub bring-up sequence (register bank
 * switch, SLV0 config, disable-master/settle timing) is the fiddlier
 * part AN5192 exists specifically to document precisely, and MUST be
 * checked against it - and against a logic analyzer trace of the actual
 * IIS2MDC responding - before trusting mag data from this on the bench.
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

/** @brief Blocking read of one accel+gyro sample over SPI. */
status_t lsm6dso32_read(lsm6dso32_sample_t *out);

/** @brief Blocking read of the sensor-hub's latched IIS2MDC output. */
status_t lsm6dso32_read_mag(lsm6dso32_mag_sample_t *out);

#ifdef __cplusplus
}
#endif

#endif /* LSM6DSO32_H */
