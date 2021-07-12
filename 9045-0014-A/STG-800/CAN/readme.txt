CAN sample code
===============
External connections/devices:
- Power supply
- CAN-Monitor by CAN
- Analog or Switched voltage GND/Power on AIN1 (Analog Input)
- DOUT1 (High side output)

Used I/O:
- LED
- CAN-Transceiver
- Analog input
- Digital output

Function:
- Set up CAN baud-rate to 250kBit
- Voltage at IN1 (V) will be send by the CAN (ID=0x3FF, DLC=1, Value in Byte 0)
- if received value from CAN (ID=0x100, DLC=1, Value in Byte 0) greater than 5, the OUT1 will be Hight otherwise Low
- LED toggle every 1000 ms



