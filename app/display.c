#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "system_stm32f10x.h"

//Initialize display shift registers driver
void display_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin   = (GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14);
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;            //Push-Pull output
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
//Send data to single shift register
void sendspibyte(uint8_t data)
{
	for(int i = 0; i < 8; i++)
	{
		if(data & (1 << 7)) GPIO_WriteBit(GPIOB, GPIO_Pin_14, Bit_SET); //Set DIO
		GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_SET);                      //Set SCLK
		GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_RESET);                    //Reset SCLK
		GPIO_WriteBit(GPIOB, GPIO_Pin_14, Bit_RESET);                    //Reset DIO
		data <<= 1;
	}
}
void senddigit(uint8_t digit) //Encode and send digit according to PCB wiring
{
	switch(digit)
	{
	case 0:
		sendspibyte(1 << 2);
		sendspibyte(0);
		break;
	case 1:
		sendspibyte(1 << 1);
		sendspibyte(0);
		break;
	case 2:
		sendspibyte(0);
		sendspibyte(1 << 7);
		break;
	case 3:
		sendspibyte(0);
		sendspibyte(1 << 4);
		break;
	case 4:
		sendspibyte(1 << 5);
		sendspibyte(0);
		break;
	case 5:
		sendspibyte(1 << 7);
		sendspibyte(0);
		break;
	case 6:
		sendspibyte(0);
		sendspibyte(1 << 1);
		break;
	case 7:
		sendspibyte(0);
		sendspibyte(1 << 2);
		break;
	case 8:
		sendspibyte(0);
		sendspibyte(1 << 5);
		break;
	case 9:
		sendspibyte(1 << 4);
		sendspibyte(0);
		break;
	}
}
//Send data to display, static 74HC595 shift register-controlled nixie driver
void display_output(int hhmmss, int isalarmset, int issynchronized)
{
	uint8_t data13 = 0; //13th shift register data; controls alarm, sync and dots
	for(int div = 100000; div > 0; div /= 10) //Decompose and send number to display, from hh to s
	{
		senddigit(hhmmss / div);
		hhmmss %= div;
	}
	if(isalarmset)      data13 |= (1 << 1); //Enable "ohm" symbol
	if(issynchronized)  data13 |= (1 << 3); //Enable "S" symbol
	data13 |= (1 << 5); //Enable dots; dots are wired to HC595 for PWM brightness control via _OE_ pin
	sendspibyte(data13);
	GPIO_WriteBit(GPIOB, GPIO_Pin_13, Bit_SET); //Set RCLK up and update display
	GPIO_WriteBit(GPIOB, GPIO_Pin_13, Bit_RESET); //Reset RCLK
}

void display_init(void);
void display_output(int hhmmss, int isalarmset, int issynchronized);









