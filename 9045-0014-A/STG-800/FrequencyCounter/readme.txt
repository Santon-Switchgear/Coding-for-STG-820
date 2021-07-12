FrequencyCounter sample code
============================
External connections/devices:
- Power supply
- PWM generator on DIN5 (Digital input for Frequency Counter)

Used I/O:
- LED
- Digital input with Frequency Counter

Function:
- Setup frequency counter for measure frequency from 0,153 Hz to 100 Hz with <1 % accuracy
- PWM at IN5 will be measured for Frequency and Duty Cycle
- Results stored in variables u16Frequency and u8DutyCycle (main.c:98+99), visible by Debugger
- LED toggle every one second

