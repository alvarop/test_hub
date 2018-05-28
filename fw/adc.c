#include <stdio.h>
#include "stm32f0xx.h"
#include "stm32f0xx_conf.h"
#include "stm32f0xx_adc.h"
#include "board.h"
#include "adc.h"

// TODO - Use internal reference for slightly higher resolution at low voltages

void adc_init() {
	ADC_InitTypeDef adc_cfg;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	GPIO_Init(_VBUS_DET_PORT, &(GPIO_InitTypeDef){(1 << _VBUS_DET_PIN), GPIO_Mode_AN, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL});
	GPIO_Init(_VBUS_EXT_DET_PORT, &(GPIO_InitTypeDef){(1 << _VBUS_EXT_DET_PIN), GPIO_Mode_AN, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL});
	GPIO_Init(_SNS1_PORT, &(GPIO_InitTypeDef){(1 << _SNS1_PIN), GPIO_Mode_AN, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL});
	GPIO_Init(_SNS2_PORT, &(GPIO_InitTypeDef){(1 << _SNS2_PIN), GPIO_Mode_AN, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL});
	GPIO_Init(_SNS3_PORT, &(GPIO_InitTypeDef){(1 << _SNS3_PIN), GPIO_Mode_AN, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL});

	ADC_DeInit(ADC1);

	ADC_StructInit(&adc_cfg);

	ADC_Init(ADC1, &adc_cfg);

	ADC_GetCalibrationFactor(ADC1);

	ADC_Cmd(ADC1, ENABLE);

	// ADC_ChannelConfig(ADC1,
	// 	(ADC_Channel_0 |
	// 		ADC_Channel_1 |
	// 		ADC_Channel_2 |
	// 		ADC_Channel_3 |
	// 		ADC_Channel_4),
	// 	ADC_SampleTime_28_5Cycles);
}

int16_t adc_read_single(uint8_t channel) {
	// TODO: Figure out why this returns the previous channel...

	ADC1->CHSELR = 0; // Because ADC_ChannelConfig doesn't clear it...
	ADC_ChannelConfig(ADC1, 1 << channel, ADC_SampleTime_28_5Cycles);

	ADC_DiscModeCmd(ADC1, ENABLE);

	ADC_StartOfConversion(ADC1);

	// TODO - add timeout
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOSEQ) == RESET) {
		// TODO - use WFI and interrupts
		__NOP();
	}

	return ADC_GetConversionValue(ADC1);
}
