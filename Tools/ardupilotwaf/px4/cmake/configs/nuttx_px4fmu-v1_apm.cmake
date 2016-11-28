include(configs/nuttx_px4fmu-common_apm)

list(APPEND config_module_list
    drivers/mpu6000
    drivers/hmc5883
    drivers/ms5611
    drivers/boards/px4fmu-v1
    drivers/px4io
)

set(config_io_board
    px4io-v1
)

