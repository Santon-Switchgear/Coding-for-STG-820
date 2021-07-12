J1939 by CAN sample code
========================
External connections/devices:
- Power supply
- J1939-device by CAN

Used I/O:
- LED
- CAN-Transceiver

Function:
- Set up CAN baud-rate to 250kBit
- LED toggle every 1000 ms
- Send the ET1 message (SAE J1939/71) every one second, toggled by u8Flag
