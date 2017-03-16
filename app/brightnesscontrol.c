#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "system_stm32f10x.h"
//Initialize internal ADC for ambient light measurements
void adc_init()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	ADC_InitTypeDef ADC_InitStructure;
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;       // single channel
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // single cycle measurement
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_Cmd(ADC1, ENABLE);

	// channel select
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_55Cycles5);

	// calibration
	ADC_ResetCalibration(ADC1);
	while (ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1));
}
uint16_t get_adc_value()
{
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	return ADC_GetConversionValue(ADC1);
}
//Smooth of the ADC readings
int expfilter(int count)
{
	const int N_AVG = 8; //2^N averages
	static int j = 0;
	static int round = 0;
	j = (j << N_AVG) - j + count + round; //rounding error avoidance
	round = j % (1 << N_AVG);
	j = (j >> N_AVG);
	return j;
}
//LDR resistance to irradiance conversion
int insolation(void)
{
	int j = expfilter(get_adc_value());
	j = (1 << 24)/j - (1 << 12); //rounding error tolerant reciprocal value
	return (j >> 6); //adjust for sensitivity
}

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;

//Dimming PWM initialization
void PWM_init()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	uint16_t CCR4_Val = 10;
	uint16_t PrescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;

	TIM_TimeBaseStructure.TIM_Period = 1023; //Time base configuration
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//PWM1 Mode configuration: Channel4
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR4_Val;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC4Init(TIM3, &TIM_OCInitStructure);

	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM3, ENABLE);

	TIM_Cmd(TIM3, ENABLE);
}
//Duty cycle calculation and dimming
void nixie_dimming_init(void)
{
	adc_init();
	PWM_init();
}
void nixie_dimming()
{
	int p = insolation();
	if(p > 1023) p = 1023;
	if(p < 40) p = 40;
	TIM_OCInitStructure.TIM_Pulse = (1024 - p); //inverted for _OE_ 74hc595's pins
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
}



