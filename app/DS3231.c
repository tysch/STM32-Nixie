DS3231_date_TypeDef date; //global DS3231 registers copy

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
