/**
 * @file    i2c.c
 * @brief   I2C2 init: F9P I2C, MCP9800 temp sensor, MicroMod EEPROM.
 */

#include "main.h"

I2C_HandleTypeDef hi2c2;

void MX_I2C2_Init(void)
{
    hi2c2.Instance = I2C2;
    /* 100 kHz standard mode at 170 MHz PCLK1; TIMING value per the
     * reference manual's I2C timing tables for this input clock - worth
     * re-deriving with STM32CubeMX's I2C timing calculator against the
     * bench-measured PCLK1 before trusting the bus speed precisely. */
    hi2c2.Init.Timing = 0x10707DBCU;
    hi2c2.Init.OwnAddress1 = 0U;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0U;
    hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
        Error_Handler();
    }
}
