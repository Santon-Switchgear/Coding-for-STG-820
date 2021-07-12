EventCounter sample code
========================
External connections/devices:
- Power supply
- PWM generator on DIN4 (Digital input for Counter)
- Switched voltage GND/Power on AIN1 (Analog Input)

Used I/O:
- LED
- Digital input with Event Counter
- Analog input

Function:
- If value at IN1 greater than 5 V, the Counter will be cleared
- With each rising edge (L->H) at the IN4 input, the counter increases by one.
- The LED flashes with a period of 100 ms * Counter value.

