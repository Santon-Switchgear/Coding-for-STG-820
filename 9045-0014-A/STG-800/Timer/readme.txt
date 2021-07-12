Timer sample code
=================
External connections/devices:
- Power supply

Used I/O:
- LED

Function:
- The variable u16Timer decrement every 1 ms down to 0 in interrupt function HAL_SYSTICK_Callback (main.c:50)
  In this sample u16Timer (main.c:78) is set to 100 (ms). If u16Timer equal 0, the LED will toggle

