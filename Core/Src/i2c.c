/**
 * @file    i2c.c
 * @brief   I2C2 init: F9P I2C, MCP9800 temp sensor, MicroMod EEPROM.
 */

#include "main.h"

I2C_HandleTypeDef hi2c2;

void MX_I2C2_Init(void)
{
    hi2c2.Instance = I2C2;
    /* 100 kHz standard mode from the 170 MHz I2C kernel clock (PCLK1,
     * the reset-default source). TIMINGR fields (RM0440, I2C_TIMINGR):
     *
     *   PRESC  = 3   -> tPRESC  = 4 / 170 MHz     = 23.5 ns
     *   SCLDEL = 10  -> tSU;DAT = 11  * 23.5 ns   = 259 ns  (>= 250 ns)
     *   SDADEL = 0
     *   SCLH   = 167 -> tHIGH   = 168 * 23.5 ns   = 3.95 us (+ sync >= 4.0)
     *   SCLL   = 251 -> tLOW    = 252 * 23.5 ns   = 5.93 us (>= 4.7 us)
     *
     * i.e. a ~10 us SCL period. This is the value STM32CubeMX generates
     * for 170 MHz / standard mode / analog filter on. (The previous
     * 0x10707DBC is the 100 kHz constant for a 64 MHz kernel clock; at
     * 170 MHz it ran the bus at ~265 kHz with a 94 ns data setup time.) */
    hi2c2.Init.Timing = 0x30A0A7FBU;
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
