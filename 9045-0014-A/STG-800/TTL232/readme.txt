TTL232 sample code
==================
External connections/devices:
- Power supply
- PC by VK16 cable on TTL232
- Analog or Switched voltage GND/Power on AIN1 (Analog Input)
- DOUT1 (High side output)

Used I/O:
- LED
- TTL RS232
- Analog input
- Digital output

Function:
- Set up baud-rate to 115200
- Voltage at IN1 (V) will be send by the UART
- if received value from UART greather than 5, the OUT1 will be Hight otherwise Low
- LED toggle every 1000 ms

