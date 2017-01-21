# STM32-Nixie
GPS-disciplined STM32-based alarm clock, built to earn some STM32F103-series MCU programming and PCB design skills.

STM32-Nixie features include:

- HH:MM:SS display, built with 6 large IN-18 nixie tubes
- DS3231 high precision (±2 ppm) RTC
- Intelligent GPS synchronization allows for less than ±0.1s error provided satellites are visible
- Daily Alarm with simple on-off control
- Alarm and RTC is supercapacitor backed-up, so alarm would work (and switch is active!) even in case of power outage
- Unlike batteries, supercapacitors can last for decades without degradation while having enough power to supply RTC for month
- Stylish IN-15B-based Alarm and sync events indication
- Light sensor-based automatic brightness correction
- Efficient wide AC-range input TOP264-based integrated power supply
