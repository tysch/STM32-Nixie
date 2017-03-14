#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "system_stm32f10x.h"

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
