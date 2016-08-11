include(configs/nuttx_vrbrain-common_apm)

list(APPEND config_module_list
    drivers/boards/vrbrain-v51
    drivers/mpu6000
)
