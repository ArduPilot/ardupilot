# network (web) firmware updates over PPP on a serial port. The board
# must also define AP_BOOTLOADER_NETWORK_PPP_SERIAL (and optionally
# AP_BOOTLOADER_NETWORK_PPP_BAUD and AP_BOOTLOADER_NETWORK_PPP_FLOW_CONTROL),
# and leave that port out of SERIAL_ORDER
include network_bootloader.inc

# PPP peers send each frame as a burst without pausing for flow
# control, so the serial input queue must hold a whole frame
undef SERIAL_BUFFERS_SIZE
define SERIAL_BUFFERS_SIZE 4096
