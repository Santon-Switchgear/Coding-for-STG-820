Analog output sample code
=========================
External connections/devices:
- Power supply
- Voltage meter on AOUT5 (Analog output)

Used I/O:
- LED
- Analog output

Function:
- Output saw tooth signal with ~9.8 Hz
  - Increment Output voltage every one ms by 50 mV
  - If Output voltage overstepped 5100 mV it starts with 0
- LED toggle every one second
