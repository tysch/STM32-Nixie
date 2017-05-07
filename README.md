# STM32-Nixie
GPS-disciplined STM32-based alarm clock, built to earn some STM32F103-series MCU programming and PCB design skills.

STM32-Nixie features include:

- HH:MM:SS display, built with 6 large IN-18 nixie tubes
- Static drive for maximum tubes longevity and brightness
- DS3231 high precision (±2 ppm) RTC
- Intelligent GPS synchronization allows for less than ±0.1s error provided satellites are visible
- Daily Alarm with simple on-off control
- Alarm and RTC is supercapacitor backed-up, so alarm would work (and switchable!) even in case of power outage
- Unlike batteries, supercapacitors can last for decades without degradation while having enough power to supply RTC for month
- Stylish IN-15B-based Alarm and sync events indication
- Light sensor-based automatic brightness correction
- Works off 12V power supply
- Designed for maximum longevity and use of commonly available parts instead of simplicity




PCBoverview.PNG shows overall design. 

MCU continuously polls RTC and sends serial data to shift registers chain, two 74HC595 8-bit registers per digit and one for alarm and sync indication. Redundant shift registers simplify PCB layout considerably, shifting routing complexity to MCU firmware domain.

GPS module is configured to perform 10 readings per second via UART. If more than 3 satellites are visible, reading of GPS time are considered as an accurate. 
The following conditions are checked:
- GPS time is hh:mm:s9.90 with s = {0,1,2,3,4} (0.1s forward to compensate UART and I2C latencies and to make clock readings as accurate as possible)
- no alarm is set at the 10 minutes of the accurate gps time to prevent alarm skipping or double firing
- difference between local time and accurate GPS time is less that 20 minute
If all of them takes the place, each 10 seconds RTC time is being updated with a fresh time to ensure it's accuracy. Such a tricky logic is used to not interfere with alarm and to make hours settings independent from GPS to set time zone/daylight saving time with just setting coarse local time.

If no GPS signal is available or GPS module is not installed at all, the clock is capable to work with sole RTC albeit the accuracy will suffer a bit.

Clock is controlled with 4 pushbuttons, alarmSW, alarm, hour and min. Hour and min button are slowly increasing RTC time. 
Alarm button displays current alarm setting and hour+alarm and min+alarm edits alarm time. 
Alarm is always set to fire at alarm time, but buzzer can be disconnected via alarmsw button.

To adapt tubes for different insolation, PWM brightness control is performed. Ambient light is measured with light-dependent resistor, smoothed out via exponential roundof-error compensated filter to damp 100/120Hz light ripple and ensure stability of a system. Capable STM32F103 PWM module allows for 30kHz PWM to ensure that no stroboscopic effect and plasma buzzing would occur.

Shift registers outputs control high-voltage PMBTA42-based switches. Base series resistors limits base drive current, base-to-GND resistors turns switches off in case of shift registers goes to Hi-Z state. The latter is used to apply PWM to all switches to control tubes brightness via _OE_ pins

Tubes are powered via 12/240V IR2153-based push-pull DC-DC converter. Push-pull is more efficient and pushes lower stress to components compared to conventional flybacks, albeit require custom-built transformer. 
The tricky part of push-pull is VD2-VD3-C19 circuit. It serves both to pre-boost 12V to 24V and lower output winding voltage and to clamp high voltage transients when MOSFETS turns off, improving both efficiency and reliability.

MCU, GPS module and shift registers are powered via MP2307 step-down converter. 

74HC14-based pushbutton switch, RC-oscillator and alarm's interrupt RTC output are wired with AND discrete MOSFET gate to a buzzer. All buzzer control logic and RTC are supercapacitor-powered. To separate RTC from MCU power domain, I2C pull-up resistors are wired to MCU's power rail. Also, alarm status is monitored by MCU via open-drain discree MOSFET decoupler.

Actual schematic is available in GIF image ( Schematic.GIF ), sPlan drawing ( PCB1.spl7 ) and Altium .SchDoc (Sheet1.SchDoc)

PCB file (STM32-Nixie-v2.PcbDoc ) is also included. Design is up to date, but some edits may needed if other tubes, GPS module or transformed would be used.


All PCB parts are listed in BOM.xlsx
ИН-18 and ИН-15Б tubes are inserted in female pins, scrapped from DB-25F PCB mount-type connectors. 
Separate dots "tubes" are 12x75mm glass tubes, filled with two NE-2 bulbs wired in series and connected to 6-pin 2-row 2.54mm male connector.

TR1 is designed for 12V input, 210V output at 50kHz. It is winded on 20x4x6 mm ferrite toroid core (M2000HM1 soviet type was used, but other Mn-Zn-type ferrites are ok). Primary winding consist of 2x23 turns, 0,45mm enamel wire, secondary have 400 turns, 0,15mm enamel wire. Secondary windings is splitted into 3 sections, each section is insulated fron the surroundings with kapton strip.
Other power ferrite cores without non-magnnetic gaps are also fine, just ensure that they have cross-section not less that 20mm^2 and all windings can be placed without compromising insulation.

GPS module is VK2828U7G5LF, soldered to 2.54mm double-row connectors as shown in GPS Module.png.

Firmware is written in C with CooCox, compiled with GCC and flashed via ST-link V2, files included.

Photos of assembled PCB's and clock are included in Photo folder.

 



