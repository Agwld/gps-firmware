/**
 * @file    gpio.c
 * @brief   GPIO init for all non-peripheral pins (netlist rev 1.0.0).
 */

#include "main.h"

void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    /* Defaults before configuring outputs */
    HAL_GPIO_WritePin(GPS_NRESET_PORT, GPS_NRESET_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPS_TURN_OFF_PORT, GPS_TURN_OFF_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPS_EEPROM_WP_PORT, GPS_EEPROM_WP_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(USER_LED_PORT, USER_LED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ERROR_LED_PORT, ERROR_LED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);

    /* Outputs: LEDs, GPS power/EEPROM control (push-pull) */
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;

    g.Pin = GPS_TURN_OFF_PIN | ERROR_LED_PIN;
    HAL_GPIO_Init(GPIOA, &g);

    g.Pin = GPS_EEPROM_WP_PIN | USER_LED_PIN;
    HAL_GPIO_Init(GPIOB, &g);

    /* IMU chip select: fast push-pull, idle high */
    g.Pin = IMU_CS_PIN;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(IMU_CS_PORT, &g);

    /* GPS reset: open drain, idle released (F9P has internal pull-up) */
    g.Pin = GPS_NRESET_PIN;
    g.Mode = GPIO_MODE_OUTPUT_OD;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPS_NRESET_PORT, &g);

    /* Plain inputs (all externally driven/pulled) */
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_NOPULL;

    g.Pin = GPS_GEO_STAT_PIN | GPS_RTK_STAT_PIN | GPS_EXTINT_PIN;
    HAL_GPIO_Init(GPIOA, &g);

    g.Pin = GPS_TX_READY_PIN | TOO_HOT_PIN;
    HAL_GPIO_Init(GPIOB, &g);

    /* Buttons: polled by sys_task (hardware debounced) */
    g.Pin = LAP_BUTTON_PIN;
    HAL_GPIO_Init(LAP_BUTTON_PORT, &g);

    g.Pin = USER_BUTTON_PIN;
    g.Pull = GPIO_NOPULL; /* external 10k pull-up */
    HAL_GPIO_Init(USER_BUTTON_PORT, &g);

    /* IMU INT1 data-ready: rising edge EXTI, notifies imu_task */
    g.Pin = IMU_INT_PIN;
    g.Mode = GPIO_MODE_IT_RISING;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(IMU_INT_PORT, &g);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
