/**
 * @file    tim.c
 * @brief   TIM3 (LL driver): free-running 1 MHz counter, CH2 input capture
 *          of the GPS PPS rising edge on PA7.
 *
 * The update interrupt extends the 16-bit counter to 32 bits in software
 * (timebase.c); the CC2 interrupt latches the PPS edge time.
 */

#include "main.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_tim.h"

void MX_TIM3_Init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* PA7 -> TIM3_CH2, AF2 */
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_7, LL_GPIO_AF_2);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_7, LL_GPIO_PULL_NO);

    /* TIM3 kernel clock = 170 MHz (APB1 x1); PSC 169 -> 1 MHz count */
    LL_TIM_SetPrescaler(TIM3, 169U);
    LL_TIM_SetAutoReload(TIM3, 0xFFFFU);
    LL_TIM_SetCounterMode(TIM3, LL_TIM_COUNTERMODE_UP);

    LL_TIM_IC_SetActiveInput(TIM3, LL_TIM_CHANNEL_CH2,
                             LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH2,
                          LL_TIM_IC_POLARITY_RISING);
    LL_TIM_IC_SetFilter(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV1_N4);
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);

    LL_TIM_EnableIT_UPDATE(TIM3);
    LL_TIM_EnableIT_CC2(TIM3);

    HAL_NVIC_SetPriority(TIM3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);

    LL_TIM_EnableCounter(TIM3);
}
