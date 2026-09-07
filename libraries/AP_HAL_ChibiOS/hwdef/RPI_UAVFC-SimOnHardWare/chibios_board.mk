# Simulation-on-hardware variant of RPI_UAVFC. chibios.py looks the board
# makefile up by hwdef directory name and has no notion of the hwdef.dat
# include chain, so a derived target needs its own; without one it falls back
# to common/chibios_board.mk and builds the ARMv7-M port.
RP2350_BOARD_DIR = RPI_UAVFC
RP2350_EXTRA_UDEFS = -DRP2350B_QFN80=1

include $(AP_HAL)/hwdef/common/chibios_board_rp2350.mk
