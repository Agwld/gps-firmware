/**
 * @file    gps_i2c.c
 * @brief   UBX-over-I2C transport for the ZED-F9P (see gps_i2c.h).
 */

#include "gps/gps_i2c.h"

#include "main.h"

#include "sys/app.h"

/* 7-bit DDC address (fixed on the MicroMod GNSS board), shifted for HAL. */
#define GPS_I2C_ADDR      (0x42U << 1)

/* Output-stream registers (ZED-F9P integration manual, "DDC port"). */
#define GPS_I2C_REG_COUNT 0xFDU /* 2 bytes, big-endian pending count */
#define GPS_I2C_REG_DATA  0xFFU /* output byte stream */

/* Generous for the longest VALSET frame (~120 B) at the 100 kHz bus. */
#define GPS_I2C_TIMEOUT_MS 100U

status_t
gps_i2c_write(const uint8_t *frame, uint16_t len)
{
    app_i2c_lock();
    HAL_StatusTypeDef hal = HAL_I2C_Master_Transmit(
        &hi2c2, GPS_I2C_ADDR, (uint8_t *) frame, len, GPS_I2C_TIMEOUT_MS);
    app_i2c_unlock();

    return (hal == HAL_OK) ? STATUS_OK : STATUS_ERROR;
}

status_t
gps_i2c_read(uint8_t *buf, uint16_t max, uint16_t *n_read)
{
    *n_read = 0U;

    app_i2c_lock();

    uint8_t count_be[2];
    if (HAL_I2C_Mem_Read(&hi2c2, GPS_I2C_ADDR, GPS_I2C_REG_COUNT,
                          I2C_MEMADD_SIZE_8BIT, count_be, 2U,
                          GPS_I2C_TIMEOUT_MS) != HAL_OK) {
        app_i2c_unlock();
        return STATUS_ERROR;
    }

    uint16_t pending = (uint16_t) (((uint16_t) count_be[0] << 8) |
                                    count_be[1]);
    /* 0xFFFF here means "receiver not ready" per the manual, and reads
     * of the data register would return 0xFF filler - treat as empty. */
    if (pending == 0U || pending == 0xFFFFU) {
        app_i2c_unlock();
        return STATUS_OK;
    }

    uint16_t n = (pending < max) ? pending : max;
    if (HAL_I2C_Mem_Read(&hi2c2, GPS_I2C_ADDR, GPS_I2C_REG_DATA,
                          I2C_MEMADD_SIZE_8BIT, buf, n,
                          GPS_I2C_TIMEOUT_MS) != HAL_OK) {
        app_i2c_unlock();
        return STATUS_ERROR;
    }

    app_i2c_unlock();
    *n_read = n;
    return STATUS_OK;
}
