Low level complete sample code
==============================
External connections/devices:
- Power supply
- Analog voltage on AIN1 (Analog Input)
- Analog voltage on AIN2 (Analog Input)
- Analog voltage on AIN3 (Analog Input)
- Switched voltage GND/Power or PWM on DIN4 (Digital Input)
- PWM on DIN5 (Digital Input)
- DOUT1 (High side output)
- DOUT2 (High side output)
- DOUT3 (High side output)
- DOUT4 (High side output)
- DOUT5 (Low side output)
- CAN-Monitor by CAN
- PC by VK16 cable on TTL232

Used I/O:
- LED
- TTL RS232
- CAN
- Analog input
- Digital input
- PWM output

Function:
- Set up CAN baud-rate to 250kBit
- Set up baud-rate to 115200
- If received values from CAN (ID=0x100),  data will stored in EEPROM
- If received values from CAN (ID=0x101),  data will read from EEPROM
- Voltage at IN1..3 (mV) will be stored in variable (main.c:108-112)
- The state of digital input DIN4 and DIN5 will be stored in variable (main.c:118-120)
- DOUT1 and DOUT3 is set to High, DOUT2 and DOUT4 is set to Low
- Sending of CAN message 0x3FF cyclic:
    Byte 0: Voltage at IN1 (1/10 V)
    Byte 1: Voltage at IN2 (1/10 V)
    Byte 2: Voltage at IN3 (1/10 V)
    Byte 3: State of DIN4
    Byte 4: State of DIN5
    Byte 5: Value of event counter
    Byte 6: Duty Cycle of frequency counter
    Byte 7: Alive (increment on every message)
- Sending two Bytes (30, 34) by TTL-RS232
- LED toggle every 1000 ms

