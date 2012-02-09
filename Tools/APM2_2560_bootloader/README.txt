Modification of the stock Arduino STK500v2 bootloader specifically for APM2 boards

There are several modifications

1. Correct sketch start if the board was rebooted due to watchdog timer

2. Fast sketch start if the USB cable is not sensed as being connected to the APM2 board

3. Flash the correct (ie, visible) LEDs in a more informative pattern in the bootloader

4. Removal of the !!! CLI feature of the stk500v2 bootloader

LED patterns to look for:

USB Connected, no traffic:
six slow blue flashes (while waiting for a serial character that does not come)
then.. sketch starts

USB Connected, some traffic:
any slow blue flashing is ceased and a short timer starts waiting for valid boot loader protocol
then.. sketch starts

USB Connected, valid boot loader traffic:
STK500 protocol packets (eg flashing a new sketch) cause rapid flashing of the blue LED for each packet

USB Not Connected
quick double blue flash
then.. quick single yellow flash

The sketch will have its own LED flashing pattern.

BUGS

If a USB cable is connected to power up the board, the boot loader starts the sketch instantly.
This is because the USB cable detection takes a short time to stabilize, by then, the bootloader
has moved on. This is not much of an issue as USB connection for the purposes of flashing the
firmware causes another reset when the USB port is opened, thus, the bootloader starts again
anyway.