include(configs/nuttx_px4fmu-common_apm)

list(APPEND config_module_list
    drivers/boards/aerofc-v1
    drivers/aerofc_adc
    lib/rc

    # replace stm32 tone_alarm with a dummy one
    modules/dummy
)

list(REMOVE_ITEM config_module_list
    drivers/stm32/adc
    drivers/stm32/tone_alarm
)

list(REMOVE_ITEM config_extra_builtin_cmds
    sercon
)
