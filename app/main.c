#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "system_stm32f10x.h"
#include <string.h>
#include <stdlib.h>
#include "display.h"
#include "brightnesscontrol.h"

void watchdog_timer_init (void)
{
  /* Enable the LSI OSC */
	RCC_LSICmd(ENABLE);   //Enable LSI Oscillator
	while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET) {}// Wait till LSI is ready
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);       // Enable Watchdog
	IWDG_SetPrescaler(IWDG_Prescaler_64);                // 4, 8, 16 ... 256
	IWDG_SetReload(0x0FFF);                     //This parameter must be a number between 0 and 0x0FFF.
	IWDG_ReloadCounter();                       //Initial watchdog reset
	IWDG_Enable();
}

//GPS sync or manual changes occurred
int timesettings_ischanged = 0;

#define DS3231_addr     0xD0 // I2C 7-bit slave address shifted for 1 bit to the left
#define DS3231_seconds  0x00 // DS3231 seconds address
#define DS3231_control  0x0E // DS3231 control register address
#define DS3231_tmp_MSB  0x11 // DS3231 temperature MSB

volatile uint8_t tm_ready = 0;
volatile uint8_t conf_mode = 0;

//DS3231's BCD  format to decimal conversion
int bcdtodec (int bcd)
{
	return (bcd & 0x0f) + 10*(bcd  >> 4);
}
//decimal conversion to DS3231's BCD
int dectobcd (int dec)
{
	return (((dec/10) << 4) + (dec % 10));
}

// All DS3231 registers
typedef struct
{
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t day_of_week;
	uint8_t date;
	uint8_t month_century;
	uint8_t year;
	uint8_t alarm1_seconds;
	uint8_t alarm1_minutes;
	uint8_t alarm1_hours;
	uint8_t alarm1_day_date;
	uint8_t alarm2_minutes;
	uint8_t alarm2_hours;
	uint8_t alarm2_day_date;
	uint8_t control_status;
	uint8_t status;
	uint8_t aging;
	uint8_t msb_temp;
	uint8_t lsb_temp;
} DS3231_date_TypeDef;

DS3231_date_TypeDef date; //global DS3231 registers copy

//Copy from DS3231 to local registers
void DS3231_ReadDateRAW(void)
{
	unsigned int i;
	char buffer[19];

	I2C_AcknowledgeConfig(I2C1,ENABLE); // Enable I2C acknowledge

	I2C_GenerateSTART(I2C1,ENABLE); // Send START condition
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5

	I2C_Send7bitAddress(I2C1,DS3231_addr,I2C_Direction_Transmitter); // Send DS3231 slave address
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6

	I2C_SendData(I2C1,DS3231_seconds); // Send DS3231 seconds register address
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8

	I2C_GenerateSTART(I2C1,ENABLE); // Send repeated START condition (aka Re-START)
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5

	I2C_Send7bitAddress(I2C1,DS3231_addr,I2C_Direction_Receiver); // Send DS3231 slave address for READ
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); // Wait for EV6

	for (i = 0; i < 18; i++)
	{
		while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
		buffer[i] = I2C_ReceiveData(I2C1); // Receive byte
	}

	I2C_AcknowledgeConfig(I2C1,DISABLE); // Disable I2C acknowledgement
	I2C_GenerateSTOP(I2C1,ENABLE); // Send STOP condition

	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
	buffer[i] = I2C_ReceiveData(I2C1); // Receive last byte

	memcpy(&date,&buffer[0],19);
}
//Copy from local registers to DS3231
void DS3231_WriteDateRAW(void)
{
	unsigned int i;
	char buffer[17];

	memcpy(&buffer[0],&date,17);

	I2C_AcknowledgeConfig(I2C1,ENABLE); // Enable I2C acknowledge

	I2C_GenerateSTART(I2C1,ENABLE); // Send START condition
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5

	I2C_Send7bitAddress(I2C1,DS3231_addr,I2C_Direction_Transmitter); // Send DS3231 slave address
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6

	I2C_SendData(I2C1,DS3231_seconds); // Send DS3231 seconds register address
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8

	for (i = 0; i < 16; i++)
	{
		I2C_SendData(I2C1,buffer[i]); // Send DS3231 seconds register address
		while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	}

	I2C_GenerateSTOP(I2C1,ENABLE);
}
//I2C setup for DS3231
void DS3231_init(void)
{
	// Init I2C
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	I2C_InitTypeDef I2CInit;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE); // Enable I2C clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

	I2C_DeInit(I2C1); // I2C reset to initial state
	I2CInit.I2C_Mode = I2C_Mode_I2C; // I2C mode is I2C *-*
	I2CInit.I2C_DutyCycle = I2C_DutyCycle_2; // I2C fast mode duty cycle (WTF is this?)
	I2CInit.I2C_OwnAddress1 = 1; // This device address (7-bit or 10-bit)
	I2CInit.I2C_Ack = I2C_Ack_Enable; // Acknowledgement enable
	I2CInit.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // choose 7-bit address for acknowledgement
	I2CInit.I2C_ClockSpeed = 100000; // 100kHz
	I2C_Cmd(I2C1,ENABLE); // Enable I2C
	I2C_Init(I2C1,&I2CInit); // Configure I2C

	while (I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY)); // Wait until I2C free

	// Check connection to DS3231
	I2C_GenerateSTART(I2C1,ENABLE); // Send START condition
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C1,DS3231_addr,I2C_Direction_Transmitter); // Send DS3231 slave address
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6
	I2C_GenerateSTOP(I2C1,ENABLE);

	// Wait for 250ms for DS3231 startup
	for(volatile int i = 0; i < 10000000;i++);
}
//Check if DS3231 is powered for the first time
int DS3231_is_reset(void)
{
	return (date.status & (1 << 7));
}
//Configure alarm and calibration of DS3231
void DS3231_startsettings(void)
{
	date.seconds = dectobcd(50);          //0x00
	date.minutes = dectobcd(59);          //0x01
	date.hours = dectobcd(23);            //0x02
	date.day_of_week = 1;                 //0x03
	date.date = 0;                        //0x04
	date.month_century = 1;               //0x05
	date.year = 1;                        //0x06
	date.alarm1_seconds = 0;              //0x07
	date.alarm1_minutes = 0;              //0x08
	date.alarm1_hours = 0;                //0x09
	date.alarm1_day_date = 1;             //0x0a
	date.alarm2_minutes = dectobcd(00);   //0x0b
	date.alarm2_hours = dectobcd(00);     //0x0c
	date.alarm2_day_date = (1<<7) | 1;    //0x0d (alarm when hour and munutes match)
	date.control_status = 0x6;            //0x0e (no SQW, alarm 2 interrupt enable)
	date.status = 0;                      //0x0f
	date.aging = 0;                       //0x10
	date.msb_temp = 0;                    //0x11
	date.lsb_temp = 0;                    //0x12
	DS3231_WriteDateRAW();
}
//Compose decimal hhmmss single number time
int hhmmss (void)
{
	return bcdtodec(date.seconds) + 100*bcdtodec(date.minutes) + 100*100*bcdtodec(date.hours);
}
//Compose decimal hhmmss single number alarm time
int alarm_hhmmss (void)
{
	return 100*bcdtodec(date.alarm2_minutes) + 100*100*bcdtodec(date.alarm2_hours);
}
//resets alarm after 1 minute if it was not switched off manually
void alarmreset(int time, int alarm)
{
	if(date.status & 10)//alarm is triggered now
		if ((time - alarm > 100) || (alarm - time > 100)) //more than 1 minute passed (time is in decimal hhmmss format)
		{
			date.status &= 0xfd;
			DS3231_WriteDateRAW();
		}
}
//GPS synchronization flag
int syncgps = 0;
//GPS's UART initialization
void USART1_Init(int BaudRate)
{
	GPIO_InitTypeDef PORT;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1,ENABLE);

	PORT.GPIO_Pin = GPIO_Pin_9;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	PORT.GPIO_Mode = GPIO_Mode_AF_PP; // TX as AF with Push-Pull
	GPIO_Init(GPIOA,&PORT);
	PORT.GPIO_Pin = GPIO_Pin_10;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	PORT.GPIO_Mode = GPIO_Mode_IN_FLOATING; // RX as in without pull-up
	GPIO_Init(GPIOA,&PORT);

	USART_InitTypeDef UART;

	UART.USART_BaudRate = BaudRate;
	UART.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // No flow control
	UART.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // RX+TX mode
	UART.USART_Parity = USART_Parity_No; // No parity check
	UART.USART_StopBits = USART_StopBits_1; // 1 stop bit
	UART.USART_WordLength = USART_WordLength_8b; // 8-bit frame

	USART_Init(USART1,&UART);
	USART_Cmd(USART1,ENABLE);

	NVIC_EnableIRQ (USART1_IRQn);           //USART1 interrupt enable
	USART1->CR1  |= USART_CR1_RXNEIE;       //Receive complete interrupt
}
void USART1_SendChar(char ch)
{
   while (!(USART1->SR & USART_SR_TXE)); // Wait while transmit data register not empty
   USART1->DR = ch;                       // Transmit character (TXE flag cleared automatically)
}
//GPS VK2828U7G5LF module configuration
void GPS_init(void)
{
	USART1_Init(9600);
	int j;
	//Disable default output strings except GGA
	const char disGGL[26] = {0x24, 0x45, 0x49, 0x47, 0x50, 0x51, 0x2c, 0x47, 0x4c, 0x4c, 0x2a, 0x32,
			0x31, 0x0d, 0x0a, 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xf0, 0x01, 0x00, 0xfb, 0x11};
	const char disGSA[26] = {0x24, 0x45, 0x49, 0x47, 0x50, 0x51, 0x2c, 0x47, 0x53, 0x41, 0x2a, 0x33,
			0x33, 0x0d, 0x0a, 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xf0, 0x02, 0x00, 0xfc, 0x13};
	const char disGSV[26] = {0x24, 0x45, 0x49, 0x47, 0x50, 0x51, 0x2c, 0x47, 0x53, 0x56, 0x2a, 0x32,
			0x34, 0x0d, 0x0a, 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xf0, 0x03, 0x00, 0xfd, 0x15};
	const char disRMC[26] = {0x24, 0x45, 0x49, 0x47, 0x50, 0x51, 0x2c, 0x52, 0x4d, 0x43, 0x2a, 0x33,
			0x41, 0x0d, 0x0a, 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xf0, 0x04, 0x00, 0xfe, 0x17};
	const char disVTG[26] = {0x24, 0x45, 0x49, 0x47, 0x50, 0x51, 0x2c, 0x56, 0x54, 0x47, 0x2a, 0x32,
			0x33, 0x0d, 0x0a, 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xf0, 0x05, 0x00, 0xff, 0x19};
	//Set 10 samples per second
	const char sps10[22] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00,
			0x7A, 0x12, 0xB5, 0x62, 0x06, 0x08, 0x00, 0x00, 0x0E, 0x30};
	//Set 115200 baud rate
	const char baud115[37] = {0xb5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xd0, 0x08, 0x00, 0x00,
			            0x00, 0xc2, 0x01, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc4, 0x96,
			            0xb5, 0x62, 0x06, 0x00, 0x01, 0x00, 0x01, 0x08, 0x22};

	for(j = 0; j < 26;j++)  USART1_SendChar(disGGL[j]);
	for(j = 0; j < 26;j++)  USART1_SendChar(disGSA[j]);
	for(j = 0; j < 26;j++)  USART1_SendChar(disGSV[j]);
	for(j = 0; j < 26;j++)  USART1_SendChar(disRMC[j]);
	for(j = 0; j < 26;j++)  USART1_SendChar(disVTG[j]);
	for(j = 0; j < 22;j++)  USART1_SendChar(sps10[j]);
	for(j = 0; j < 37;j++)  USART1_SendChar(baud115[j]);
	USART1_Init(115200);
}
//Checks for sync conditions and set GPS derived time
void synctime(int utctime)
{
	int time_seconds = bcdtodec(date.seconds);
	int time_minutes = bcdtodec(date.minutes);
	int time_hours =   bcdtodec(date.hours);
	int time_s = time_seconds + 60*time_minutes + 3600*time_hours;

	int alarm_minutes = bcdtodec(date.alarm2_hours);
	int alarm_hours = bcdtodec(date.alarm2_hours);
	int alarm_s = 60* alarm_minutes + 3600* alarm_hours;

	if(abs(alarm_s - time_s) > 600) // forbid synchronization near alarm to prevent alarm skipping
	{
		int utctime_s = (utctime % 100) + 60*((utctime / 100) % 100);
		time_s %= 3600;
		if(abs(utctime_s - time_s) < 1200) //sync only when difference is less than 20 minutes to sync only mid-hour
		{
			date.seconds = dectobcd(utctime_s % 60); //sync only minutes and seconds to preserve time zone hours shift
			date.minutes = dectobcd(utctime_s / 60);
	  		timesettings_ischanged = 1;
	  		syncgps = 1;
		}
	}
}

//Read and parse GPS output; GPS sync happens here
void USART1_IRQHandler(void)
{
	static int cnt = 0;          //String position
	static int utctime = 0;
	static char n_sat = 0;       //Number of satellites detected
	unsigned char tmp;
	if((USART1->SR & USART_SR_RXNE)!=0) //Check if interrupt is caused by receiving of character
	{
		tmp = USART1->DR;               //Receive new character
		//$GPGGA,060556.00,2236.91418,N,11403.24669,E,2,08,1.02,115.1,M,-2.4,M,,0000*43 -- GPS input example
		if(tmp == '$') cnt = 0; //Set start marker
		else
		{
			cnt++;
			tmp -= 48;     //ASCII to decimal digit conversion of character
			switch (cnt)
			{
				//UTC Time positions
			    case 7: utctime += 10*1000*1000*tmp;
					break;
				case 8: utctime += 1000*1000*tmp;
					break;
				case 9: utctime += 100*1000*tmp;
					break;
				case 10: utctime += 10*1000*tmp;
					break;
				case 11: utctime += 1000*tmp;
					break;
				case 12: utctime += 100*tmp;
					break;
				case 14: utctime += 10*tmp;
				    break;
				case 15: utctime += tmp;
				    break;
				//Satellite number positions
				case 46: n_sat += 10*tmp;
					break;
				case 47:
				    {
					    n_sat += tmp;
					    if((utctime % 1000 == 990)  // Synchronize at hh:mm:s9.90, 0.10s in advance to compensate all delays
					    		&& (n_sat > 2))    // Enough satellites are visible
					    {
					    	utctime/= 100;         //Omit .xx seconds part and floor hh:mm:s9.90 t0 hh:mm:s9
					    	utctime++;             //Add one second to hh:mm:(s+1)0
					    	if(utctime % 100 != 60)//if !hh:mm:60
					    	{
						    	synctime(utctime);
					    	}
					    }
					    n_sat = 0;
					    utctime = 0;
				    }
					break;
			}
		}
	}
}
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
int display_alarm; // 0 -- display time, 1 -- display alarm time flag
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

int main(void)
{
	int time,alarm;         //hhmmss decimal values
	//watchdog_timer_init();
	DS3231_init();          //DS3231 communication settings
	DS3231_ReadDateRAW();   //Check if DS3231 was set before power up
	if(DS3231_is_reset()) DS3231_startsettings();
	GPS_init();
	display_init();
	nixie_dimming_init();
	buttons_init();
	buttons_timer_init();
	alarm_indication_init();
	while(1)
	{
		//IWDG_ReloadCounter();      //Watchdog reset
		if(timesettings_ischanged) //Time settings was changed manually or via GPS synchronization
    	{
    		DS3231_WriteDateRAW();
    		timesettings_ischanged = 0;
    	}
    	else DS3231_ReadDateRAW();
		time = hhmmss();
		alarm =  alarm_hhmmss();
		alarmreset(time, alarm);
		nixie_dimming();
		for(volatile int i = 0; i < 100000;i++);
		if(display_alarm) display_output(alarm, alarm_indication(), syncgps);
		else display_output(time, alarm_indication(), syncgps);
	}
}



