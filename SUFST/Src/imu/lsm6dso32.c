/**
 * @file    lsm6dso32.c
 * @brief   LSM6DSO32 + sensor-hub IIS2MDC driver (see lsm6dso32.h for what
 *          has and hasn't been checked against the datasheets/AN5192).
 */

#include "imu/lsm6dso32.h"

#include "main.h"

#include "board/board_config.h"

/* Main register bank */
#define REG_FUNC_CFG_ACCESS 0x01U
#define REG_STATUS_REG      0x1EU
#define REG_OUT_TEMP_L      0x20U
#define REG_OUTX_L_G        0x22U
#define REG_OUTX_L_A        0x28U
#define REG_WHO_AM_I        0x0FU
#define REG_CTRL1_XL        0x10U
#define REG_CTRL2_G         0x11U
#define REG_CTRL3_C         0x12U
#define REG_SENSOR_HUB1     0x02U /* IIS2MDC OUTX_L .. OUTZ_H, 6 bytes */

#define WHO_AM_I_VALUE 0x6CU

/* FUNC_CFG_ACCESS bits */
#define FUNC_CFG_SHUB_REG_ACCESS (1U << 6)

/* Sensor-hub (SHUB) bank registers, valid only while
 * FUNC_CFG_SHUB_REG_ACCESS is set */
#define REG_MASTER_CONFIG 0x14U
#define REG_SLV0_ADD      0x15U
#define REG_SLV0_SUBADD   0x16U
#define REG_SLV0_CONFIG   0x17U

#define MASTER_CONFIG_MASTER_ON   (1U << 2)
/* AN5192 Section 7.2.1: "The WRITE_ONCE bit must be set to 1 if slave 0
 * is used for read transactions" - mandatory, not just for repeated
 * writes despite the name. The app note's own continuous-read reference
 * example (Section 7.4) sets this bit alongside MASTER_ON. */
#define MASTER_CONFIG_WRITE_ONCE (1U << 6)

/* IIS2MDC (magnetometer) I2C address and output register */
#define IIS2MDC_I2C_ADDR   0x1EU
#define IIS2MDC_OUTX_L_REG 0x68U
#define IIS2MDC_READ_LEN   6U

/* CTRL3_C: BDU (block data update, bit6) so a multi-byte read can't
 * straddle a sensor-side update, and IF_INC (bit2, register address
 * auto-increment on multi-byte access - default-on out of reset, but
 * this is a full-register write, not read-modify-write, so it must be
 * set explicitly here too or the burst reads below silently re-read
 * OUT_TEMP_L 14/6 times instead of walking the register map). */
#define CTRL3_C_BDU    (1U << 6)
#define CTRL3_C_IF_INC (1U << 2)

#define IMU_ACCEL_SENSITIVITY_G_PER_LSB (IMU_ACCEL_FS_G / 32768.0f)
#define IMU_GYRO_SENSITIVITY_DPS_PER_LSB (IMU_GYRO_FS_DPS / 32768.0f)
#define IIS2MDC_SENSITIVITY_UT_PER_LSB 0.15f /* datasheet: 1.5 mG/LSB */

static void
cs_low(void)
{
    HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET);
}

static void
cs_high(void)
{
    HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);
}

static status_t
write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = {(uint8_t) (reg & 0x7FU), val};

    cs_low();
    HAL_StatusTypeDef st = HAL_SPI_Transmit(&hspi1, tx, sizeof(tx), 10U);
    cs_high();

    return (st == HAL_OK) ? STATUS_OK : STATUS_ERROR;
}

static status_t
read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t addr = (uint8_t) (reg | 0x80U); /* MSB=1 => read */

    cs_low();
    HAL_StatusTypeDef st = HAL_SPI_Transmit(&hspi1, &addr, 1U, 10U);
    if (st == HAL_OK) {
        st = HAL_SPI_Receive(&hspi1, buf, len, 10U);
    }
    cs_high();

    return (st == HAL_OK) ? STATUS_OK : STATUS_ERROR;
}

status_t
lsm6dso32_init(void)
{
    uint8_t who_am_i = 0U;
    if (read_regs(REG_WHO_AM_I, &who_am_i, 1U) != STATUS_OK) {
        return STATUS_TIMEOUT;
    }
    if (who_am_i != WHO_AM_I_VALUE) {
        return STATUS_TIMEOUT;
    }

    /* CTRL1_XL = 0x4C: ODR[3:0]=0100 (104 Hz), FS[1:0]=11 (+-16 g,
     * matching board_config.h's IMU_ACCEL_FS_G), LPF2_XL_EN=0 (first
     * digital filter stage, not LPF2). +-16 g, not +-32 g - the DSO32's
     * extended range isn't used here; verified bit-by-bit against the
     * LSM6DSO32 datasheet CTRL1_XL/Table 44/45 encoding. */
    if (write_reg(REG_CTRL1_XL, 0x4CU) != STATUS_OK) { /* 104 Hz, +-16 g */
        return STATUS_ERROR;
    }
    /* CTRL2_G = 0x4C: ODR[3:0]=0100 (104 Hz), FS[1:0]=11 (+-2000 dps,
     * matching IMU_GYRO_FS_DPS), FS_125=0. */
    if (write_reg(REG_CTRL2_G, 0x4CU) != STATUS_OK) { /* 104 Hz, +-2000 dps */
        return STATUS_ERROR;
    }
    if (write_reg(REG_CTRL3_C, CTRL3_C_BDU | CTRL3_C_IF_INC) != STATUS_OK) {
        return STATUS_ERROR;
    }

    return STATUS_OK;
}

status_t
lsm6dso32_mag_init(void)
{
    if (write_reg(REG_FUNC_CFG_ACCESS, FUNC_CFG_SHUB_REG_ACCESS) !=
        STATUS_OK) {
        return STATUS_ERROR;
    }

    status_t st = write_reg(REG_SLV0_ADD, (uint8_t) ((IIS2MDC_I2C_ADDR << 1) |
                                                       0x01U));
    if (st == STATUS_OK) {
        st = write_reg(REG_SLV0_SUBADD, IIS2MDC_OUTX_L_REG);
    }
    if (st == STATUS_OK) {
        st = write_reg(REG_SLV0_CONFIG, IIS2MDC_READ_LEN);
    }
    if (st == STATUS_OK) {
        /* SHUB_PU_EN deliberately left clear: R27/R28 on the board
         * already pull MAG_SCL/MAG_SDA up to 3V3 (confirmed in the
         * gps-mainboard netlist), so the internal pull-up isn't needed -
         * matches the "external pull-ups present" case in AN5192's
         * SHUB_PU_EN description, as opposed to its reference example
         * (which sets it because *that* example assumes no external
         * pull-ups). */
        st = write_reg(REG_MASTER_CONFIG,
                        MASTER_CONFIG_MASTER_ON | MASTER_CONFIG_WRITE_ONCE);
    }

    /* Back to the main register bank regardless of the above outcome -
     * leaving SHUB_REG_ACCESS set would break normal accel/gyro reads. */
    write_reg(REG_FUNC_CFG_ACCESS, 0x00U);

    return st;
}

status_t
lsm6dso32_read(lsm6dso32_sample_t *out)
{
    uint8_t buf[14];
    if (read_regs(REG_OUT_TEMP_L, buf, sizeof(buf)) != STATUS_OK) {
        return STATUS_ERROR;
    }

    int16_t temp_raw = (int16_t) (uint16_t) (buf[0] | (buf[1] << 8));
    int16_t gx = (int16_t) (uint16_t) (buf[2] | (buf[3] << 8));
    int16_t gy = (int16_t) (uint16_t) (buf[4] | (buf[5] << 8));
    int16_t gz = (int16_t) (uint16_t) (buf[6] | (buf[7] << 8));
    int16_t ax = (int16_t) (uint16_t) (buf[8] | (buf[9] << 8));
    int16_t ay = (int16_t) (uint16_t) (buf[10] | (buf[11] << 8));
    int16_t az = (int16_t) (uint16_t) (buf[12] | (buf[13] << 8));

    out->temp_c = 25.0f + (float) temp_raw / 256.0f;
    out->gx_dps = (float) gx * IMU_GYRO_SENSITIVITY_DPS_PER_LSB;
    out->gy_dps = (float) gy * IMU_GYRO_SENSITIVITY_DPS_PER_LSB;
    out->gz_dps = (float) gz * IMU_GYRO_SENSITIVITY_DPS_PER_LSB;
    out->ax_g = (float) ax * IMU_ACCEL_SENSITIVITY_G_PER_LSB;
    out->ay_g = (float) ay * IMU_ACCEL_SENSITIVITY_G_PER_LSB;
    out->az_g = (float) az * IMU_ACCEL_SENSITIVITY_G_PER_LSB;

    return STATUS_OK;
}

status_t
lsm6dso32_read_mag(lsm6dso32_mag_sample_t *out)
{
    uint8_t buf[6];
    if (read_regs(REG_SENSOR_HUB1, buf, sizeof(buf)) != STATUS_OK) {
        out->valid = false;
        return STATUS_ERROR;
    }

    int16_t mx = (int16_t) (uint16_t) (buf[0] | (buf[1] << 8));
    int16_t my = (int16_t) (uint16_t) (buf[2] | (buf[3] << 8));
    int16_t mz = (int16_t) (uint16_t) (buf[4] | (buf[5] << 8));

    out->mx_ut = (float) mx * IIS2MDC_SENSITIVITY_UT_PER_LSB;
    out->my_ut = (float) my * IIS2MDC_SENSITIVITY_UT_PER_LSB;
    out->mz_ut = (float) mz * IIS2MDC_SENSITIVITY_UT_PER_LSB;
    out->valid = true;

    return STATUS_OK;
}
