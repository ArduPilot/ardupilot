include(configs/nuttx_px4fmu-common_apm)

list(APPEND config_module_list
    drivers/boards/px4fmu-v1
    drivers/px4io
    drivers/px4flow
)

set(config_io_board
    px4io-v1
)

