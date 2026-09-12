# Pico2 (RP2350) board makefile. Everything is shared; see
# common/chibios_board_rp2350.mk for what the variables below select.
#
# CRT0_AREAS_NUMBER is 0 here: the Pico2 linker script defines no extra RAM
# areas for crt0 to initialise, unlike the carrier boards.
RP2350_BOARD_DIR = Pico2
RP2350_CRT0_AREAS_NUMBER = 0

include $(AP_HAL)/hwdef/common/chibios_board_rp2350.mk
