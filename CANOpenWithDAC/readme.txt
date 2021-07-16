CANOpen sample code
===================
External connections/devices:
- Power supply
- CANOpen-device by CAN
- Voltage meter on AOUT5 (Analog output)

Used I/O:
- LED
- CAN-Transceiver
- Analog output


Function:
- Set up CAN baud-rate to 250kBit
- Set up CANOpen-Stack with Node-ID 10
- LED toggle every one second or flicker if error
- Output saw tooth signal
  - Increment Output voltage every one ms by 50 mV
  - If Output voltage overstepped 5100 mV it starts with 0
