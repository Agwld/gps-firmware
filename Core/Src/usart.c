/**
 * @file    usart.c
 * @brief   UART init: USART1 debug, USART3 GPS in / MoTeC out.
 *
 * USART2 is deliberately not initialised: its pins go nowhere on this
 * board (PA2/MCU_UART_TX2 is unrouted; PA15/MCU_UART_RX2 only reaches
 * solder jumper JP7, which is bridged 2-3 to route the RS232 input
 * straight to the F9P's UART1 RX for RTCM corrections instead).
 */

#include "main.h"

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200U;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;

    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
}

void MX_USART3_UART_Init(void)
{
    huart3.Instance = USART3;
    /* RX = F9P UBX stream, TX = NMEA to the MoTeC (JP6 bridged 2-3);
     * one peripheral, one shared baud. The F9P's UART1 is switched from
     * its 38400 power-on default to this rate over I2C by gps_config at
     * boot, so RX reads garbage until that completes - the UBX parser
     * just resyncs. Must match GPS_UART_BAUD (board_config.h). */
    huart3.Init.BaudRate = 115200U;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;

    if (HAL_UART_Init(&huart3) != HAL_OK) {
        Error_Handler();
    }
}
