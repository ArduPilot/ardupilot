set(LIB_NAME APO)
	
set(${LIB_NAME}_SRCS 
        AP_Autopilot.cpp
        AP_CommLink.cpp
        AP_Controller.cpp
        AP_Guide.cpp
        AP_HardwareAbstractionLayer.cpp
        AP_MavlinkCommand.cpp
        AP_Navigator.cpp
        AP_RcChannel.cpp
	APO.cpp 
	)  # Firmware sources

set(${LIB_NAME}_HDRS  
        AP_Autopilot.h
        AP_CommLink.h
        AP_Controller.h
        AP_Guide.h
        AP_HardwareAbstractionLayer.h
        AP_MavlinkCommand.h
        AP_Navigator.h
        AP_RcChannel.h
        AP_Var_keys.h
	APO.h
        constants.h
        template.h
)
	
include_directories(
#	${CMAKE_SOURCE_DIR}/libraries/AP_Common 
	${CMAKE_SOURCE_DIR}/libraries/FastSerial
	${CMAKE_SOURCE_DIR}/libraries/ModeFilter
#	
	)

set(${LIB_NAME}_BOARD ${BOARD})

generate_arduino_library(${LIB_NAME})
