EEPROM sample code
==================
External connections/devices:
- Power supply
- Analog or Switched voltage GND/Power on AIN1 (Analog Input)

Used I/O:
- LED
- Analog input

Function:
- Voltage at IN1 (V) will be stored every 3 s at EEPROM address 0x05 if changed
- If the value at EEPROM address 0x05 greater than 5, the LED will be on otherwise off

