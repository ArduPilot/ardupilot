#
# This files is used by cmake to present options to the 
# user in the cmake-gui, it can also be used directly to
# set options in the cmake command line.
#
# This file generates a C(++) header file, 
# which can be used as an override of other definition header files.
# 

apm_option("CONFIG_HAL_BOARD" TYPE BUILD_FLAG
    DESCRIPTION "APM Hardware?" 
    OPTIONS "HAL_BOARD_APM1" "HAL_BOARD_APM2" "HAL_BOARD_AVR_SITL" "HAL_BOARD_FLYMAPLE" "HAL_BOARD_PX4" "HAL_BOARD_LINUX" "HAL_BOARD_EMPTY" "HAL_BOARD_VRBRAIN" ""
    DEFAULT "HAL_BOARD_LINUX")

apm_option("CONFIG_HAL_BOARD_SUBTYPE" TYPE BUILD_FLAG
    DESCRIPTION "APM Hardware?"  
    OPTIONS "HAL_BOARD_SUBTYPE_LINUX_NONE" "HAL_BOARD_SUBTYPE_LINUX_PXF" "HAL_BOARD_SUBTYPE_LINUX_ERLE" "HAL_BOARD_SUBTYPE_LINUX_NAVIO" "HAL_BOARD_SUBTYPE_LINUX_ZYNQ" "HAL_BOARD_SUBTYPE_LINUX_BBBMINI" "HAL_BOARD_SUBTYPE_VRBRAIN_V45" "HAL_BOARD_SUBTYPE_VRBRAIN_V51" "HAL_BOARD_SUBTYPE_VRBRAIN_V52"
    DEFAULT "HAL_BOARD_SUBTYPE_LINUX_NAVIO")

apm_option("TEST_DUMMYA" TYPE STRING
    DESCRIPTION "foo"  
    OPTIONS "a" "b" "c" "d"
    DEFAULT "d")

apm_option("TEST_DUMMYB" TYPE STRING
    DESCRIPTION "foo"  
    OPTIONS "a" "b" "c" "d"
    DEFAULT "c")