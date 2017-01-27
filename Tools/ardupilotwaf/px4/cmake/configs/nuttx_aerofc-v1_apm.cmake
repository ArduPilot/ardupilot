include(configs/nuttx_px4fmu-common_apm)

list(APPEND config_module_list
    drivers/boards/aerofc-v1
    lib/rc
)

list(REMOVE_ITEM config_module_list
    drivers/stm32/adc
    drivers/stm32/tone_alarm
)
