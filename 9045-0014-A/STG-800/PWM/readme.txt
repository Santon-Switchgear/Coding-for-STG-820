PWM sample code
===============
External connections/devices:
- Power supply
- Switched voltage GND/Power on DIN4 (Digital Input)
- DOUT5 (Low side output)

Used I/O:
- LED
- TTL RS232
- Digital input
- PWM output

Function:
- Depending from voltage on DIN4 the frequency of DOUT5 (Low Side) will be changed
    Input low -> set frequency to 200 Hz and Duty Cycle to 25 %
    Input active -> set frequency to 100 Hz and Duty Cycleto 75 %
- LED toggle every 1000 ms

