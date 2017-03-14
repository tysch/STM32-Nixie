#ifndef _BUTTONS_H
#include "button_controls.h"
#endif

#define _DS3231_H
#define DS3231_addr     0xD0 // I2C 7-bit slave address shifted for 1 bit to the left
#define DS3231_seconds  0x00 // DS3231 seconds address
#define DS3231_control  0x0E // DS3231 control register address
#define DS3231_tmp_MSB  0x11 // DS3231 temperature MSB

volatile uint8_t tm_ready = 0;
volatile uint8_t conf_mode = 0;

// All DS3231 registers
typedef struct {
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

extern DS3231_date_TypeDef date; //global DS3231 registers copy

void DS3231_ReadDateRAW(void);
//Copy from local registers to DS3231
void DS3231_WriteDateRAW(void);
//I2C setup for DS3231
void DS3231_init(void);
//Check if DS3231 is powered for the first time
int DS3231_is_reset(void);
//Configure alarm and calibration of DS3231
void DS3231_startsettings(void);
//Compose decimal hhmmss single number time
int hhmmss (void);
//Compose decimal hhmmss single number alarm time
int alarm_hhmmss (void);
//resets alarm after 1 minute if it was not switched off manually
void alarmreset(int time, int alarm);
