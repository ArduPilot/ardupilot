include(configs/nuttx_px4fmu-common_apm)

list(APPEND config_module_list
    drivers/boards/px4fmu-v2
    drivers/pwm_input
    modules/uavcan
    drivers/px4io
    drivers/oreoled
)

list(APPEND config_extra_libs
    uavcan
    uavcan_stm32_driver
)

set(config_io_board
    px4io-v2
)
