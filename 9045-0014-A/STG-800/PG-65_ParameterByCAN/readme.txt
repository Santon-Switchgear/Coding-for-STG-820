PG-65 by CAN sample code
========================
External connections/devices:
- Power supply
- PG-65 by CAN

Used I/O:
- LED
- CAN-Transceiver

Function:
- LED toggle every 1000 ms
- Parameters from the PG65 (main.c:57-65) structure can be showed/changed by CAN
  The Values can be used as PG65[0].i32Value ... PG65[5].i32Value in this sample
  To change the parameter count, set PG65_PARAMETER_COUNT (main.h:72) to the new value
