include(nuttx/px4_impl_nuttx)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/toolchains/Toolchain-arm-none-eabi.cmake)

set(config_module_list
    platforms/common
    platforms/nuttx
    platforms/nuttx/px4_layer
    modules/param
#
# Board support modules
#
    drivers/device
    drivers/stm32
    drivers/stm32/adc
    drivers/stm32/tone_alarm
    drivers/led
    drivers/px4fmu
    drivers/rgbled
    drivers/mpu6000
    drivers/hmc5883
    drivers/ms5611
    drivers/mb12xx
    drivers/ll40ls
    drivers/trone
    drivers/airspeed
    drivers/ets_airspeed
    drivers/meas_airspeed
    drivers/mkblctrl
    drivers/batt_smbus
    drivers/irlock

#
# System commands
#
    systemcmds/bl_update
    systemcmds/mixer
    systemcmds/perf
    systemcmds/pwm
    systemcmds/reboot
    systemcmds/top
    systemcmds/nshterm
    systemcmds/mtd
    systemcmds/ver
    systemcmds/reflect
    systemcmds/motor_test
    systemcmds/usb_connected
    systemcmds/otp

#
# Library modules
#
    modules/systemlib
    modules/systemlib/mixer
    modules/uORB
    lib/mathlib/math/filter
    lib/conversion
)

set(config_extra_builtin_cmds
    serdis
    sercon
    ArduPilot
)

set(config_extra_libs
    ${APM_PROGRAM_LIB}
)

set(config_uavcan_num_ifaces 2)

add_custom_target(sercon)
set_target_properties(sercon PROPERTIES
    MAIN "sercon"
    STACK "2048"
)

add_custom_target(serdis)
set_target_properties(serdis PROPERTIES
    MAIN "serdis"
    STACK "2048"
)

add_custom_target(ArduPilot)
set_target_properties(ArduPilot PROPERTIES
    MAIN "ArduPilot"
    STACK "4096"
)
