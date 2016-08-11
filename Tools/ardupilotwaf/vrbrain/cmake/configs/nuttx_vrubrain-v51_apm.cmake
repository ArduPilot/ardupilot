include(configs/nuttx_vrbrain-common_apm)

list(APPEND config_module_list
    drivers/boards/vrubrain-v51
    drivers/mpu6000
)
