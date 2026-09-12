# Laurel (RP2350B QFN-80) board makefile. Everything is shared; see
# common/chibios_board_rp2350.mk for what the variables below select.
RP2350_BOARD_DIR = Laurel
RP2350_EXTRA_UDEFS = -DRP2350B_QFN80=1 -DHAL_ENABLE_THREAD_STATISTICS

include $(AP_HAL)/hwdef/common/chibios_board_rp2350.mk
