My take on the firmware for the MightyOhm Geiger Counter kit by Jeff Keyzer:
https://mightyohm.com/geiger

Differences from the original version:

- eliminated compiler warnings
- Removed uSv/h calculation and output
- Removed instantaneous and fast cpm calculations and mode output
- Added array-element overflow indication on cps output
- Added overflow indication on cpm output when the calculation uses an overflowed array-element
- Added beep/flash at power on
- Serial output baud rate is adjustable in the Makefile as UART_BAUD
- Added compile option for high cpm-maximum with reduced resolution. Use -D HIGH_CPM in the DEFINES line in the Makefile.
