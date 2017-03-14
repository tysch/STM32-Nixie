#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "system_stm32f10x.h"


//GPS sync or manual changes occurred
int timesettings_ischanged = 0;

//GPS synchronization flag
int syncgps = 0;
int display_alarm; // 0 -- display time, 1 -- display alarm time flag

//Buttons for time and alarm setting
void buttons_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}
//Alarm indication input (alarm on/off switch is not a part of MCU program)
//MCU reads external status to form a signal on display
void alarm_indication_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_5);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}
int alarm_indication(void)
{
	return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5);
}
//Buttons polling timer initialization
void buttons_timer_init(void)
{
	// TIMER4
	TIM_TimeBaseInitTypeDef TIMER_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  	TIM_TimeBaseStructInit(&TIMER_InitStructure);
    TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up; //Counting mode
    TIMER_InitStructure.TIM_Prescaler = 8000;                 //System frequency (72MHz) division
    TIMER_InitStructure.TIM_Period = 1000;                    //Overflow interrupt period, Fint = 72000000/8000/1000 = 9 Hz
    TIM_TimeBaseInit(TIM4, &TIMER_InitStructure);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);                // Enable overflow interrupt
    TIM_Cmd(TIM4, ENABLE);                                    // Enable timer

    /* NVIC Configuration */
    /* Enable the TIM4_IRQn Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
//Minute button action
void minincr()
{
	int m = bcdtodec(date.minutes);
	if(m >= 59) m = 0;
	else m++;
	date.minutes = dectobcd(m);
}
//Hour button action
void hourincr()
{
	int h = bcdtodec(date.hours);
	if(h >= 23) h = 0;
	else h++;
	date.hours = dectobcd(h);
}
//Alarm + Minute button action
void alarmminincr()
{
	int m = bcdtodec(date.alarm2_minutes);
	if(m >= 59) m = 0;
	else m++;
	date.alarm2_minutes = dectobcd(m);
}
//Alarm + Hour button action
void alarmhourincr()
{
	int h = bcdtodec(date.alarm2_hours);
	if(h >= 23) h = 0;
	else h++;
	date.alarm2_hours = dectobcd(h);
}
//Buttons for time and alarm setting actions
void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
    	// Clear timer interrupt flag
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) != 0) display_alarm = 1;      //Alarm button pressed
        else display_alarm = 0;
    	if((GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2) != 0) && (display_alarm == 0))//Min button pressed
    	{
    		minincr();
    		timesettings_ischanged = 1;
    	}
    	if((GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) != 0) && (display_alarm == 0))//Hour button pressed
    	{
    		hourincr();
    		timesettings_ischanged = 1;
    	}
    	if((GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2) != 0) && (display_alarm == 1))//Alarm + Min button pressed
    	{
    		alarmminincr();
    		timesettings_ischanged = 1;
    	}

    	if((GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) != 0) && (display_alarm == 1))//Alarm + Hour button pressed
    	{
    		alarmhourincr();
    		timesettings_ischanged = 1;
    	}
    	syncgps = 0;
    }
}







