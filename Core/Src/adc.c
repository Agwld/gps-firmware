/**
 * @file    adc.c
 * @brief   ADC1: the MCU's internal temperature sensor (GPS_Temp
 *          mcu_temp_c). No external pins are used.
 */

#include "main.h"

/* Supply the factory calibration is scaled against. The board runs VDDA
 * straight off the 3V3 rail with no separate reference, so the nominal
 * value is used rather than measuring VREFINT: a 1 % rail error moves the
 * reading by about 3 degC, which is fine for a health indicator. */
#define ADC_VDDA_MV 3300U

/* One conversion is ~260 ADC clock cycles (~6 us); this only bounds a
 * peripheral that never finishes. */
#define ADC_CONVERSION_TIMEOUT_MS 2U

static ADC_HandleTypeDef hadc1;
static bool s_adc_ready;

void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef ch = {0};

    hadc1.Instance = ADC1;
    /* Synchronous clock: HCLK 170 MHz / 4 = 42.5 MHz, inside the 60 MHz
     * limit and needing no kernel-clock mux setup in RCC. */
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.GainCompensation = 0U;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 1U;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    hadc1.Init.OversamplingMode = DISABLE;

    /* A failure here only costs the MCU temperature reading - not worth
     * halting the node over, so it is reported as "not ready" instead of
     * going to Error_Handler(). */
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        return;
    }
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
        return;
    }

    /* The temperature sensor needs >= 5 us of sampling (datasheet
     * ts_temp): 247.5 cycles at 42.5 MHz is 5.8 us. */
    ch.Channel = ADC_CHANNEL_TEMPSENSOR_ADC1;
    ch.Rank = ADC_REGULAR_RANK_1;
    ch.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
    ch.SingleDiff = ADC_SINGLE_ENDED;
    ch.OffsetNumber = ADC_OFFSET_NONE;
    ch.Offset = 0U;
    if (HAL_ADC_ConfigChannel(&hadc1, &ch) != HAL_OK) {
        return;
    }

    s_adc_ready = true;
}

bool adc_read_mcu_temp_c(float *temp_c)
{
    if (!s_adc_ready) {
        return false;
    }

    bool ok = false;
    if (HAL_ADC_Start(&hadc1) == HAL_OK) {
        if (HAL_ADC_PollForConversion(&hadc1, ADC_CONVERSION_TIMEOUT_MS) ==
            HAL_OK) {
            uint32_t raw = HAL_ADC_GetValue(&hadc1);
            /* Two-point factory calibration (TS_CAL1 @ 30 degC, TS_CAL2 @
             * 130 degC, both taken at 3.0 V), rescaled to this board's
             * supply by the HAL macro. Whole degrees. */
            int32_t deg_c = __HAL_ADC_CALC_TEMPERATURE(ADC_VDDA_MV, raw,
                                                        ADC_RESOLUTION_12B);
            if (deg_c != (int32_t) LL_ADC_TEMPERATURE_CALC_ERROR) {
                *temp_c = (float) deg_c;
                ok = true;
            }
        }
        (void) HAL_ADC_Stop(&hadc1);
    }
    return ok;
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        __HAL_RCC_ADC12_CLK_ENABLE();
    }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        __HAL_RCC_ADC12_CLK_DISABLE();
    }
}
