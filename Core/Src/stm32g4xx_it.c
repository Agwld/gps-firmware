/**
 * @file    stm32g4xx_it.c
 * @brief   Interrupt handlers. SVC/PendSV are mapped to the FreeRTOS port
 *          in FreeRTOSConfig.h.
 */

#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

#include "fusion/timebase.h"

void NMI_Handler(void)
{
    for (;;) {
    }
}

void HardFault_Handler(void)
{
    for (;;) {
    }
}

void MemManage_Handler(void)
{
    for (;;) {
    }
}

void BusFault_Handler(void)
{
    for (;;) {
    }
}

void UsageFault_Handler(void)
{
    for (;;) {
    }
}

void DebugMon_Handler(void)
{
}

/**
 * @brief SysTick: HAL tick and FreeRTOS tick share the handler.
 */
void SysTick_Handler(void)
{
    HAL_IncTick();

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        extern void xPortSysTickHandler(void);
        xPortSysTickHandler();
    }
}

void DMA1_Channel1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart3_rx);
}

void DMA1_Channel4_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_spi1_rx);
}

void DMA1_Channel5_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_spi1_tx);
}

void DMA1_Channel6_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart3_tx);
}

/* SPI1 error interrupt (ERRIE, enabled by the HAL DMA transfer): routes
 * overrun/mode-fault to HAL_SPI_ErrorCallback so a failed IMU read is
 * reported promptly instead of only via the caller's DMA timeout. Normal
 * completion is signalled off the SPI1_RX DMA channel (CH4), not here. */
void SPI1_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&hspi1);
}

void USART3_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart3);
}

void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(IMU_INT_PIN);
}

void TIM3_IRQHandler(void)
{
    timebase_tim3_irq();
}

void FDCAN1_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&hfdcan1);
}

void FDCAN1_IT1_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&hfdcan1);
}
