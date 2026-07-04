/**
 * @file    dma.c
 * @brief   DMA controller clock + interrupt enable.
 *
 * Channel-to-request routing (DMAMUX) is bound in the peripheral MSP inits:
 *   DMA1_CH1  USART3_RX (circular, GPS UBX stream)
 *   DMA1_CH2  USART2_RX (circular, RTCM in)
 *   DMA1_CH3  USART2_TX (NMEA out)
 *   DMA1_CH4  SPI1_RX
 *   DMA1_CH5  SPI1_TX
 *   DMA1_CH6  USART3_TX (GPS config / RTCM forward)
 */

#include "main.h"

void MX_DMA_Init(void)
{
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* Sensor path (GPS RX, SPI) at 5; aux path at 6 */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
}
