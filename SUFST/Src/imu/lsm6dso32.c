/**
 * @file    lsm6dso32.c
 * @brief   LSM6DSO32 + sensor-hub IIS2MDC driver (see lsm6dso32.h for what
 *          has and hasn't been checked against the datasheets/AN5192).
 */

#include "imu/lsm6dso32.h"

#include <string.h>

#include "main.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

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
#define REG_SENSOR_HUB1     0x02U /* IIS2MDC STATUS_REG, then OUTX_L..OUTZ_H
                                   * (7 bytes total, see lsm6dso32_mag_init) */

#define WHO_AM_I_VALUE 0x6CU

/* FUNC_CFG_ACCESS bits */
#define FUNC_CFG_SHUB_REG_ACCESS (1U << 6)

/* Sensor-hub (SHUB) bank registers, valid only while
 * FUNC_CFG_SHUB_REG_ACCESS is set */
#define REG_MASTER_CONFIG 0x14U
#define REG_SLV0_ADD      0x15U
#define REG_SLV0_SUBADD   0x16U
#define REG_SLV0_CONFIG   0x17U
#define REG_DATAWRITE_SLV0 0x21U

/* STATUS_MASTER_MAINPAGE: STATUS_MASTER (0x22, SHUB bank) mirrored into
 * the main bank at 0x39, readable without FUNC_CFG_SHUB_REG_ACCESS set
 * (AN5192 Section 7.2.2) - used to poll a one-shot write's completion
 * without repeatedly toggling FUNC_CFG_ACCESS. */
#define REG_STATUS_MASTER_MAINPAGE 0x39U
#define STATUS_MASTER_WR_ONCE_DONE (1U << 7)

#define MASTER_CONFIG_MASTER_ON   (1U << 2)
/* AN5192 Section 7.2.1: "The WRITE_ONCE bit must be set to 1 if slave 0
 * is used for read transactions" - mandatory, not just for repeated
 * writes despite the name. The app note's own continuous-read reference
 * example (Section 7.4) sets this bit alongside MASTER_ON. */
#define MASTER_CONFIG_WRITE_ONCE (1U << 6)

/* IIS2MDC (magnetometer) I2C address and registers. The sensor-hub SLV0
 * read starts at STATUS_REG rather than straight at OUTX_L_REG: the
 * IIS2MDC's own ODR (100 Hz) doesn't line up 1:1 with the 104 Hz
 * accel/gyro trigger driving the sensor-hub read, so without a
 * data-ready check some read cycles would re-fetch the same latched
 * output word and lsm6dso32_read_mag() would report it as a fresh
 * sample - Zyxda in STATUS_REG is exactly the bit the datasheet provides
 * to detect that. */
#define IIS2MDC_I2C_ADDR    0x1EU
#define IIS2MDC_STATUS_REG  0x67U
#define IIS2MDC_OUTX_L_REG  0x68U
#define IIS2MDC_READ_LEN    7U /* STATUS_REG + 6 data bytes */
#define IIS2MDC_STATUS_ZYXDA (1U << 3)

/* IIS2MDC bring-up registers, one-shot-written via the sensor-hub master
 * before the continuous read is set up - see AN5192 Section 7.4's worked
 * LIS2MDL example (register-identical to the IIS2MDC). Without this the
 * sensor stays in its power-on-default idle mode (CFG_REG_A MD[1:0]=11)
 * and never converts. */
#define IIS2MDC_CFG_REG_A 0x60U
#define IIS2MDC_CFG_REG_B 0x61U
#define IIS2MDC_CFG_REG_C 0x62U

/* CFG_REG_A = COMP_TEMP_EN(b7, mandatory per the datasheet) |
 * ODR[1:0]=11 (100 Hz, b3:2) | MD[1:0]=00 (continuous mode, b1:0). */
#define IIS2MDC_CFG_REG_A_VAL 0x8CU
/* CFG_REG_B = OFF_CANC(b1): enable offset cancellation. */
#define IIS2MDC_CFG_REG_B_VAL 0x02U
/* CFG_REG_C = BDU(b4): block data update, matching CTRL3_C's BDU on the
 * main accel/gyro. */
#define IIS2MDC_CFG_REG_C_VAL 0x10U

/* Poll bound for a one-shot sensor-hub write's WR_ONCE_DONE: the write
 * completes on the next accel/gyro trigger (<=104 Hz => <=~10 ms since
 * lsm6dso32_init() already started them), so 100 iters * 2 ms is
 * generous headroom against a NACKed/absent IIS2MDC hanging bring-up
 * forever rather than a real timing bound. */
#define MAG_WRITE_POLL_MAX_ITERS 100U

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

/* Largest single burst is the 14-byte accel/gyro read plus its leading
 * address byte. */
#define SPI_MAX_XFER 15U

/* DMA-completion timeout: a 15-byte transfer at 5.3 MHz is ~23 us, so a
 * few ms is enormous headroom - it exists only to keep the loop alive if
 * a transfer never completes (unplugged sensor, bus fault) rather than
 * blocking the imu_task forever. */
#define SPI_DMA_TIMEOUT_MS 5U

/* DMA source/sink buffers. File-scope (not stack-local) so they always
 * live in .bss/SRAM, which DMA1 can reach - CCMRAM (where the linker
 * script intends task stacks to eventually live) is NOT DMA-accessible
 * on the G4, so a stack buffer would be a latent trap. */
static uint8_t s_spi_tx[SPI_MAX_XFER];
static uint8_t s_spi_rx[SPI_MAX_XFER];

/* Given once from the SPI DMA complete/error ISR, taken by the (only)
 * caller - imu_task - so the task blocks (yielding the CPU) for the
 * duration of each transfer instead of spinning in a blocking HAL call.
 * SPI1 is touched by no other task, so no bus mutex is needed. */
static StaticSemaphore_t s_spi_done_buf;
static SemaphoreHandle_t s_spi_done;

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

/* Full-duplex DMA register read: clock out [addr | 0x80, dummy...] while
 * clocking in [junk, data...]. One transfer keeps CS asserted across the
 * whole burst; the sensor's register auto-increment (CTRL3_C.IF_INC)
 * walks the map. CS is raised in the completion ISR for tight timing;
 * the RX byte received during the address phase (s_spi_rx[0]) is
 * discarded. */
static status_t
read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (len == 0U || len > SPI_MAX_XFER - 1U) {
        return STATUS_ERROR;
    }

    s_spi_tx[0] = (uint8_t) (reg | 0x80U); /* MSB=1 => read */
    for (uint16_t i = 1U; i <= len; i++) {
        s_spi_tx[i] = 0xFFU; /* dummy clocks for the read phase */
    }

    cs_low();
    /* HAL_SPI_TransmitReceive_DMA clears hspi1.ErrorCode at entry, so the
     * post-transfer error check below reflects only this transfer. */
    if (HAL_SPI_TransmitReceive_DMA(&hspi1, s_spi_tx, s_spi_rx,
                                     (uint16_t) (len + 1U)) != HAL_OK) {
        cs_high();
        return STATUS_ERROR;
    }

    if (xSemaphoreTake(s_spi_done, pdMS_TO_TICKS(SPI_DMA_TIMEOUT_MS)) !=
        pdTRUE) {
        /* Transfer stalled: tear it down and raise CS ourselves (the ISR
         * that normally does it never ran). Abort quiesces the peripheral
         * so no further give can occur; the zero-wait take then drains a
         * give that may have raced in just before the abort, so a stale
         * token can't satisfy the next read's take without a real
         * transfer. */
        HAL_SPI_Abort(&hspi1);
        (void) xSemaphoreTake(s_spi_done, 0);
        cs_high();
        return STATUS_TIMEOUT;
    }

    /* Woke on the semaphore, but that fires from the error callback too;
     * reject a transfer that faulted (overrun etc.) rather than copying
     * out partial data. CS was already raised by whichever callback ran. */
    if (HAL_SPI_GetError(&hspi1) != HAL_SPI_ERROR_NONE) {
        return STATUS_ERROR;
    }

    memcpy(buf, &s_spi_rx[1], len);
    return STATUS_OK;
}

/* HAL SPI DMA callbacks (weak in the HAL, overridden here). SPI1 is the
 * only SPI in use; guard anyway so this stays correct if another is
 * added. Both raise CS and wake the waiting task - on error the task's
 * take() would otherwise burn the full timeout before noticing. */
void
HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
        cs_high();
        BaseType_t hp_woken = pdFALSE;
        xSemaphoreGiveFromISR(s_spi_done, &hp_woken);
        portYIELD_FROM_ISR(hp_woken);
    }
}

void
HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
        cs_high();
        BaseType_t hp_woken = pdFALSE;
        xSemaphoreGiveFromISR(s_spi_done, &hp_woken);
        portYIELD_FROM_ISR(hp_woken);
    }
}

status_t
lsm6dso32_init(void)
{
    /* Create the DMA-completion semaphore before the first read_regs()
     * uses it (idempotent so a retry-after-fault re-init is safe). */
    if (s_spi_done == NULL) {
        s_spi_done = xSemaphoreCreateBinaryStatic(&s_spi_done_buf);
    }

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

/* One-shot pass-through write of `val` to IIS2MDC register `reg` via the
 * sensor-hub master, per AN5192 Section 7.4. Blocks (polling
 * STATUS_MASTER_MAINPAGE) until the write actually lands on the far side
 * of the I2C bus, since the caller chains several of these and each one
 * must complete before the next reconfigures SLV0. Leaves FUNC_CFG_ACCESS
 * cleared (main bank) on return either way. */
static status_t
mag_write_reg(uint8_t reg, uint8_t val)
{
    status_t st = write_reg(REG_FUNC_CFG_ACCESS, FUNC_CFG_SHUB_REG_ACCESS);
    if (st == STATUS_OK) {
        st = write_reg(REG_SLV0_ADD, (uint8_t) (IIS2MDC_I2C_ADDR << 1));
    }
    if (st == STATUS_OK) {
        st = write_reg(REG_SLV0_SUBADD, reg);
    }
    if (st == STATUS_OK) {
        st = write_reg(REG_SLV0_CONFIG, 0x00U);
    }
    if (st == STATUS_OK) {
        st = write_reg(REG_DATAWRITE_SLV0, val);
    }
    if (st == STATUS_OK) {
        st = write_reg(REG_MASTER_CONFIG,
                        MASTER_CONFIG_MASTER_ON | MASTER_CONFIG_WRITE_ONCE);
    }
    /* STATUS_MASTER_MAINPAGE (0x39) is a main-bank register - drop back
     * regardless of the above outcome so it's pollable next. */
    write_reg(REG_FUNC_CFG_ACCESS, 0x00U);
    if (st != STATUS_OK) {
        return st;
    }

    uint8_t status = 0U;
    for (uint32_t iters = 0U;; iters++) {
        if (read_regs(REG_STATUS_MASTER_MAINPAGE, &status, 1U) !=
            STATUS_OK) {
            return STATUS_ERROR;
        }
        if ((status & STATUS_MASTER_WR_ONCE_DONE) != 0U) {
            break;
        }
        if (iters >= MAG_WRITE_POLL_MAX_ITERS) {
            return STATUS_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(2U));
    }

    /* Disable the I2C master before the next reconfiguration - AN5192's
     * one-shot write routine (Section 7.4, steps 8-10) requires this plus
     * a >=300 us settle before FUNC_CFG_ACCESS is touched again; the
     * scheduler tick below is far more than that. SHUB_PU_EN stays clear
     * throughout (external pull-ups, see lsm6dso32_mag_init()). */
    st = write_reg(REG_FUNC_CFG_ACCESS, FUNC_CFG_SHUB_REG_ACCESS);
    if (st == STATUS_OK) {
        st = write_reg(REG_MASTER_CONFIG, 0x00U);
    }
    write_reg(REG_FUNC_CFG_ACCESS, 0x00U);
    vTaskDelay(pdMS_TO_TICKS(1U));
    return st;
}

status_t
lsm6dso32_mag_init(void)
{
    /* Bring the IIS2MDC out of its power-on-default idle mode
     * (CFG_REG_A default MD[1:0]=11, datasheet Table 26) into continuous
     * conversion - without this the sensor never converts and the
     * sensor-hub read below just latches a constant/zero word forever.
     * Three one-shot writes, per AN5192 Section 7.4's worked LIS2MDL
     * example (register-identical to the IIS2MDC). */
    status_t st = mag_write_reg(IIS2MDC_CFG_REG_A, IIS2MDC_CFG_REG_A_VAL);
    if (st == STATUS_OK) {
        st = mag_write_reg(IIS2MDC_CFG_REG_B, IIS2MDC_CFG_REG_B_VAL);
    }
    if (st == STATUS_OK) {
        st = mag_write_reg(IIS2MDC_CFG_REG_C, IIS2MDC_CFG_REG_C_VAL);
    }
    if (st != STATUS_OK) {
        return st;
    }

    if (write_reg(REG_FUNC_CFG_ACCESS, FUNC_CFG_SHUB_REG_ACCESS) !=
        STATUS_OK) {
        return STATUS_ERROR;
    }

    st = write_reg(REG_SLV0_ADD, (uint8_t) ((IIS2MDC_I2C_ADDR << 1) |
                                             0x01U));
    if (st == STATUS_OK) {
        st = write_reg(REG_SLV0_SUBADD, IIS2MDC_STATUS_REG);
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
    uint8_t buf[7]; /* IIS2MDC STATUS_REG, then OUTX_L..OUTZ_H */
    if (read_regs(REG_SENSOR_HUB1, buf, sizeof(buf)) != STATUS_OK) {
        out->valid = false;
        return STATUS_ERROR;
    }

    /* Zyxda (new-data) gate: the IIS2MDC's 100 Hz ODR doesn't line up
     * 1:1 with the 104 Hz accel/gyro trigger driving this sensor-hub
     * read, so without this check some cycles would silently re-read
     * the same latched output word and report it as a fresh sample. */
    if ((buf[0] & IIS2MDC_STATUS_ZYXDA) == 0U) {
        out->valid = false;
        return STATUS_OK;
    }

    int16_t mx = (int16_t) (uint16_t) (buf[1] | (buf[2] << 8));
    int16_t my = (int16_t) (uint16_t) (buf[3] | (buf[4] << 8));
    int16_t mz = (int16_t) (uint16_t) (buf[5] | (buf[6] << 8));

    out->mx_ut = (float) mx * IIS2MDC_SENSITIVITY_UT_PER_LSB;
    out->my_ut = (float) my * IIS2MDC_SENSITIVITY_UT_PER_LSB;
    out->mz_ut = (float) mz * IIS2MDC_SENSITIVITY_UT_PER_LSB;
    out->valid = true;

    return STATUS_OK;
}
