#ifndef _DS3231_H
#include "DS3231.h"
#endif

#define _BUTTONS_H
void buttons_init(void );
void buttons_timer_init(void );
void alarm_indication_init(void );
//GPS sync or manual changes occurred
extern int timesettings_ischanged;
//GPS synchronization flag
extern int syncgps;
extern int display_alarm; // 0 -- display time, 1 -- display alarm time flag
