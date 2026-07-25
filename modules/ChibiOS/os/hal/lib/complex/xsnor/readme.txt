XSNOR driver, usage and makefile inclusion notes.

XSNOR driver uses the HAL WSPI and/or SPI driver for I/O.

This driver is compatible with the previous serial_nor driver except for the
initialization function which is now named xxxObjectInit() where xxx is the
device prefix (n25q for example).

Other functions previously prefixed with "snor" are now prefixed with "xsnor"
in order to avoid conflicts (you can use both in the same application).

Just include the device mk file in the makefile:

For Micron N25Q:

include $(CHIBIOS)/os/hal/lib/complex/xsnor/devices/micron_n25q/hal_xsnor_micron_n25q.mk

For Macronics MX25:

include $(CHIBIOS)/os/hal/lib/complex/xsnor/devices/macronix_mx25/hal_xsnor_macronix_mx25.mk

etc
