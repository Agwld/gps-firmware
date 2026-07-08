/**
 * @file    main.c
 * @brief   SUFST GPS node entry point: clocks, peripherals, RTOS start.
 */

#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

#include "sys/app.h"

IWDG_HandleTypeDef hiwdg;

void SystemClock_Config(void);

extern void MX_GPIO_Init(void);
extern void MX_DMA_Init(void);
extern void MX_USART1_UART_Init(void);
extern void MX_USART3_UART_Init(void);
extern void MX_SPI1_Init(void);
extern void MX_I2C2_Init(void);
extern void MX_FDCAN1_Init(void);
extern void MX_TIM3_Init(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART1_UART_Init();
    MX_USART3_UART_Init();
    MX_SPI1_Init();
    MX_I2C2_Init();
    MX_FDCAN1_Init();
    MX_TIM3_Init();

    /* Start the independent watchdog here, before the scheduler, so the
     * whole task-boot window is covered: once running, the IWDG resets
     * the MCU on ANY subsequent hang (a spin, a failed configASSERT, an
     * init that never returns) because nothing refreshes it during a
     * hang. sys_task refreshes it in its loop. ~1.25 s at LSI/32; the
     * boot config yields often enough (gps_config) for sys_task to run
     * well within that. */
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
    hiwdg.Init.Reload = 1250U; /* ~1.25 s at 32 kHz LSI / 32 */
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
        Error_Handler();
    }

    app_init(); /* creates all tasks and IPC objects (static) */

    vTaskStartScheduler();

    /* Only reached if the scheduler failed to start */
    Error_Handler();
}

/**
 * @brief System clock: HSE 25 MHz -> PLL -> 170 MHz SYSCLK (boost mode).
 *
 * PLL: /M=5 (5 MHz) *N=68 (340 MHz VCO) /R=2 -> 170 MHz, /Q=4 -> 85 MHz.
 * FDCAN kernel clock = PCLK1 = 170 MHz.
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = RCC_PLLM_DIV5;
    osc.PLL.PLLN = 68;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = RCC_PLLQ_DIV4;
    osc.PLL.PLLR = RCC_PLLR_DIV2;

    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        Error_Handler();
    }

    /* RM0440 boost-mode entry: switch to PLL with AHB /2, wait 1 us, then
     * bring AHB back to /1. */
    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV2;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }

    /* > 1 us at 85 MHz */
    for (volatile uint32_t i = 0U; i < 200U; i++) {
    }

    clk.ClockType = RCC_CLOCKTYPE_HCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;

    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    HAL_GPIO_WritePin(ERROR_LED_PORT, ERROR_LED_PIN, GPIO_PIN_SET);
    for (;;) {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
    Error_Handler();
}
#endif
