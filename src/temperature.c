#include "mcu.h"
#include "temperature.h"
#include "stm32l1xx_adc.h"

double temperature_p;
double temperature_q;

void temperature_init()
{
	 //RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); //enable clock for GPIOA
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //enable clock for ADC1

	  ADC_InitTypeDef ADC_InitStructure;
	  ADC_CommonInitTypeDef ADC_CommonInitStructure;
	  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
	  ADC_CommonInit(&ADC_CommonInitStructure);

	  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;
	  ADC_InitStructure.ADC_NbrOfConversion = 1;

	  ADC_Init(ADC1, &ADC_InitStructure);
	  ADC_Cmd(ADC1, ENABLE);

	  temperature_p = 1.0;
	  temperature_q = 0.0;
}

void temperature_calibrate(double p, double q)
{
	temperature_p = p;
	temperature_q = q;
}

double temperature_getp() {
	return temperature_p;
}

double temperature_getq() {
	return temperature_q;
}

uint16_t temperature_raw()
{
	ADC_RegularChannelConfig(ADC1, 16, 1, ADC_SampleTime_16Cycles);
	ADC_SoftwareStartConv(ADC1);
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	return ADC_GetConversionValue(ADC1);
}

double temperature(uint16_t raw)
{
	return ((double)raw * temperature_p + temperature_q);
}
