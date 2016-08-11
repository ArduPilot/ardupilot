include(configs/nuttx_vrbrain-common_apm)

list(APPEND config_module_list
    drivers/boards/vrubrain-v52
    drivers/mpu6000
)
