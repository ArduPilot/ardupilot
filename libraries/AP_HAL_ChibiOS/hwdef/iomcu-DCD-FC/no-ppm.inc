# pcb1_1: IO_RCIN reaches only U3 pad 29 (PA8); no external PPM input.
# This is the I/O MCU PA8, NOT the FMU PA8 PWM output.
undef PA8
PA8 UNUSED_PPM INPUT PULLDOWN

# The normal I/O image explicitly enables TIM1 input capture in hwdef.inc.
# Remove its settings as well as the pin mapping. DShot inherits this override.
undef HAL_USE_ICU STM32_ICU_USE_TIM1 RCIN_ICU_TIMER RCIN_ICU_CHANNEL
undef STM32_RCIN_DMA_STREAM STM32_RCIN_DMA_CHANNEL
define HAL_USE_ICU FALSE

undef AP_RCPROTOCOL_PPMSUM_ENABLED
define AP_RCPROTOCOL_PPMSUM_ENABLED 0
