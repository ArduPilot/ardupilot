include(configs/nuttx_px4fmu-common_apm)

list(APPEND config_module_list
    drivers/mpu9250
    drivers/boards/px4fmu-v4
    drivers/pwm_input
    modules/uavcan
    lib/mathlib
    lib/rc
)

list(APPEND config_extra_libs
    uavcan
    uavcan_stm32_driver
)
